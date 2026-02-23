//! MDMA Linked-List transfer support.
//!
//! Linked-list mode allows chaining multiple transfer descriptors so the MDMA
//! automatically sequences through them without CPU intervention.
//!
//! **CRITICAL**: Node descriptors must be placed in AXI-accessible SRAM, NOT DTCM/ITCM.

use core::sync::atomic::{Ordering, fence};

use super::mdma::{
    CHANNEL_STATE, DataSize, HardwareTrigger, IncrementMode, MdmaChannel, MdmaError, MdmaTransferOptions,
    PackingAlignment, Priority, TriggerMode, is_tcm_address,
};
use super::regs;

/// A single linked-list node descriptor.
///
/// Laid out exactly as hardware expects (RM0433 §16.5.6).
/// Must be in AXI-accessible SRAM, NOT DTCM/ITCM.
#[repr(C, align(8))]
#[derive(Clone, Copy)]
pub struct MdmaLinkedListNode {
    pub ctcr: u32,
    pub cbndtr: u32,
    pub csar: u32,
    pub cdar: u32,
    pub cbrur: u32,
    pub clar: u32,
    pub ctbr: u32,
    _reserved0: u32,
    pub cmar: u32,
    pub cmdr: u32,
}

impl MdmaLinkedListNode {
    pub const EMPTY: Self = Self {
        ctcr: 0,
        cbndtr: 0,
        csar: 0,
        cdar: 0,
        cbrur: 0,
        clar: 0,
        ctbr: 0,
        _reserved0: 0,
        cmar: 0,
        cmdr: 0,
    };

    /// Configure for memory (incrementing) → fixed peripheral address.
    pub fn configure_mem_to_fixed<T: Sized>(&mut self, src: &[T], dst_addr: *mut T, config: &LinkedListNodeConfig) {
        let data_size = DataSize::from_bytes(core::mem::size_of::<T>());
        let total_bytes = src.len() * core::mem::size_of::<T>();
        self.configure_raw(
            src.as_ptr() as u32,
            dst_addr as u32,
            total_bytes,
            data_size,
            data_size,
            IncrementMode::Increment,
            IncrementMode::Fixed,
            config,
        );
    }

    /// Configure for fixed peripheral → memory (incrementing).
    pub fn configure_fixed_to_mem<T: Sized>(
        &mut self,
        src_addr: *const T,
        dst: &mut [T],
        config: &LinkedListNodeConfig,
    ) {
        let data_size = DataSize::from_bytes(core::mem::size_of::<T>());
        let total_bytes = dst.len() * core::mem::size_of::<T>();
        self.configure_raw(
            src_addr as u32,
            dst.as_mut_ptr() as u32,
            total_bytes,
            data_size,
            data_size,
            IncrementMode::Fixed,
            IncrementMode::Increment,
            config,
        );
    }

    /// Configure for memory → memory.
    pub fn configure_mem_to_mem<T: Sized>(&mut self, src: &[T], dst: &mut [T], config: &LinkedListNodeConfig) {
        let data_size = DataSize::from_bytes(core::mem::size_of::<T>());
        let total_bytes = src.len().min(dst.len()) * core::mem::size_of::<T>();
        self.configure_raw(
            src.as_ptr() as u32,
            dst.as_mut_ptr() as u32,
            total_bytes,
            data_size,
            data_size,
            IncrementMode::Increment,
            IncrementMode::Increment,
            config,
        );
    }

    /// Low-level node configuration.
    pub fn configure_raw(
        &mut self,
        src: u32,
        dst: u32,
        total_bytes: usize,
        src_size: DataSize,
        dst_size: DataSize,
        src_inc: IncrementMode,
        dst_inc: IncrementMode,
        config: &LinkedListNodeConfig,
    ) {
        assert!(total_bytes > 0 && total_bytes <= 0x10000);

        let buf_len = config.buffer_length.min(128).max(1);

        let (pke, pam) = match config.packing {
            PackingAlignment::Pack => (true, 0b00u32),
            PackingAlignment::RightAlignZeroPad => (false, 0b00),
            PackingAlignment::RightAlignSignExtend => (false, 0b10),
            PackingAlignment::LeftAlignZeroPad => (false, 0b01),
        };

        self.ctcr = regs::build_tcr(
            src_size as u32,
            src_size as u32,
            src_inc as u32,
            dst_size as u32,
            dst_size as u32,
            dst_inc as u32,
            0,
            0,
            (buf_len - 1) as u32,
            pke,
            pam,
            config.trigger_mode as u32,
            config.trigger.is_none(),
        );

        self.cbndtr = total_bytes as u32;
        self.csar = src;
        self.cdar = dst;
        self.cbrur = 0;
        self.clar = 0;

        self.ctbr = regs::build_tbr(
            config.trigger.map(|t| t as u32).unwrap_or(0),
            is_tcm_address(src),
            is_tcm_address(dst),
        );

        self.cmar = 0;
        self.cmdr = 0;
    }

    /// Link a chain of nodes (last CLAR = 0).
    pub fn link_chain(nodes: &mut [Self]) {
        let len = nodes.len();
        if len == 0 {
            return;
        }
        for i in 0..len - 1 {
            nodes[i].clar = &nodes[i + 1] as *const Self as u32;
        }
        nodes[len - 1].clar = 0;
    }

    /// Link a chain of nodes in a circular ring (last → first).
    pub fn link_chain_circular(nodes: &mut [Self]) {
        let len = nodes.len();
        if len == 0 {
            return;
        }
        for i in 0..len - 1 {
            nodes[i].clar = &nodes[i + 1] as *const Self as u32;
        }
        nodes[len - 1].clar = &nodes[0] as *const Self as u32;
    }
}

/// Per-node configuration options.
#[derive(Debug, Clone)]
pub struct LinkedListNodeConfig {
    pub buffer_length: u8,
    pub packing: PackingAlignment,
    pub trigger_mode: TriggerMode,
    pub trigger: Option<HardwareTrigger>,
}

impl Default for LinkedListNodeConfig {
    fn default() -> Self {
        Self {
            buffer_length: 128,
            packing: PackingAlignment::Pack,
            trigger_mode: TriggerMode::Block,
            trigger: None,
        }
    }
}

// ============================================================================
// MdmaChannel extension for linked-list execution
// ============================================================================

impl MdmaChannel<'_> {
    /// Execute a linked-list transfer starting from `first_node`.
    ///
    /// # Safety
    ///
    /// All nodes and their buffers must remain valid for the transfer duration.
    /// Nodes must be in AXI-accessible SRAM (not DTCM/ITCM) and 8-byte aligned.
    pub async unsafe fn execute_linked_list(
        &mut self,
        first_node: &MdmaLinkedListNode,
        options: &MdmaTransferOptions,
    ) -> Result<(), MdmaError> {
        let ch = self.ch();
        let state = &CHANNEL_STATE[ch];

        state.complete_count.store(0, Ordering::Release);
        state.error.store(0, Ordering::Release);

        regs::modify_cr(ch, |v| v & !regs::cr::EN);
        while regs::read_cr(ch) & regs::cr::EN != 0 {}
        self.clear_all_flags();

        fence(Ordering::SeqCst);

        // Load first node's configuration into channel registers
        regs::write_sar(ch, first_node.csar);
        regs::write_dar(ch, first_node.cdar);
        regs::write_tcr(ch, first_node.ctcr);
        regs::write_bndtr(ch, first_node.cbndtr);
        regs::write_brur(ch, first_node.cbrur);
        regs::write_tbr(ch, first_node.ctbr);
        regs::write_mar(ch, first_node.cmar);
        regs::write_mdr(ch, first_node.cmdr);

        // Link address → next node (first is already loaded)
        regs::write_lar(ch, first_node.clar);

        // Control register — enable with interrupts
        regs::write_cr(
            ch,
            regs::build_cr(
                options.priority as u32,
                true,
                true,
                false,
                false,
                false,
                options.byte_swap,
                options.half_word_swap,
                options.word_swap,
                true,
            ),
        );

        // Software trigger for first node
        if options.trigger.is_none() {
            regs::modify_cr(ch, |v| v | regs::cr::SWRQ);
        }

        self.wait().await
    }
}
