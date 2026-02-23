//! Master DMA (MDMA) driver for STM32H7 on Embassy.
//!
//! Uses raw register access — does NOT depend on stm32-metapac having
//! typed MDMA register definitions.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicUsize, Ordering, compiler_fence, fence};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use super::regs;

// ============================================================================
// Enums and configuration types
// ============================================================================

/// MDMA hardware trigger request lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[allow(dead_code)]
pub enum HardwareTrigger {
    Dma1Tcif0 = 0,
    Dma1Tcif1 = 1,
    Dma1Tcif2 = 2,
    Dma1Tcif3 = 3,
    Dma1Tcif4 = 4,
    Dma1Tcif5 = 5,
    Dma1Tcif6 = 6,
    Dma1Tcif7 = 7,
    Dma2Tcif0 = 8,
    Dma2Tcif1 = 9,
    Dma2Tcif2 = 10,
    Dma2Tcif3 = 11,
    Dma2Tcif4 = 12,
    Dma2Tcif5 = 13,
    Dma2Tcif6 = 14,
    Dma2Tcif7 = 15,
    LtdcLineIt = 16,
    JpegIft = 17,
    JpegIfnt = 18,
    JpegOft = 19,
    JpegOfne = 20,
    JpegOec = 21,
    Xspi1Ft = 22,
    Xspi1Tc = 23,
    Dma2dClut = 24,
    Dma2dTc = 25,
    Dma2dTw = 26,
    DsiTe = 27,
    DsiEor = 28,
    Sdmmc1DataEnd = 29,
    Sdmmc1BuffEnd = 30,
    Sdmmc1CmdEnd = 31,
    Octospi2Ft = 32,
    Octospi2Tc = 33,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TriggerMode {
    Buffer = 0b00,
    #[default]
    Block = 0b01,
    RepeatedBlock = 0b10,
    LinkedList = 0b11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Priority {
    Low = 0b00,
    Medium = 0b01,
    High = 0b10,
    #[default]
    VeryHigh = 0b11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataSize {
    Byte = 0b00,
    HalfWord = 0b01,
    Word = 0b10,
    DoubleWord = 0b11,
}

impl DataSize {
    pub const fn bytes(self) -> usize {
        match self {
            DataSize::Byte => 1,
            DataSize::HalfWord => 2,
            DataSize::Word => 4,
            DataSize::DoubleWord => 8,
        }
    }

    pub const fn from_bytes(n: usize) -> Self {
        match n {
            1 => DataSize::Byte,
            2 => DataSize::HalfWord,
            4 => DataSize::Word,
            8 => DataSize::DoubleWord,
            _ => panic!("unsupported data size"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IncrementMode {
    Fixed = 0b00,
    #[default]
    Increment = 0b10,
    Decrement = 0b11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PackingAlignment {
    #[default]
    Pack,
    RightAlignZeroPad,
    RightAlignSignExtend,
    LeftAlignZeroPad,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MdmaTransferOptions {
    pub priority: Priority,
    pub trigger: Option<HardwareTrigger>,
    pub trigger_mode: TriggerMode,
    pub buffer_length: u8,
    pub source_increment: IncrementMode,
    pub destination_increment: IncrementMode,
    pub packing: PackingAlignment,
    pub byte_swap: bool,
    pub half_word_swap: bool,
    pub word_swap: bool,
}

impl Default for MdmaTransferOptions {
    fn default() -> Self {
        Self {
            priority: Priority::VeryHigh,
            trigger: None,
            trigger_mode: TriggerMode::Block,
            buffer_length: 128,
            source_increment: IncrementMode::Increment,
            destination_increment: IncrementMode::Increment,
            packing: PackingAlignment::Pack,
            byte_swap: false,
            half_word_swap: false,
            word_swap: false,
        }
    }
}

// ============================================================================
// Low-level helpers
// ============================================================================

#[inline]
pub fn is_tcm_address(address: u32) -> bool {
    address < 0x0004_0000 || (0x2000_0000..0x2002_0000).contains(&address)
}

pub(crate) struct ChannelState {
    pub(crate) waker: AtomicWaker,
    pub(crate) complete_count: AtomicUsize,
    pub(crate) error: AtomicUsize,
}

impl ChannelState {
    pub(crate) const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            complete_count: AtomicUsize::new(0),
            error: AtomicUsize::new(0),
        }
    }
}

pub(crate) const MDMA_CHANNEL_COUNT: usize = 16;
pub(crate) static CHANNEL_STATE: [ChannelState; MDMA_CHANNEL_COUNT] =
    [const { ChannelState::new() }; MDMA_CHANNEL_COUNT];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MdmaError {
    TransferError,
    BlockRepeatError,
}

// ============================================================================
// The MDMA channel driver
// ============================================================================

pub struct MdmaChannel<'d> {
    pub(crate) channel: u8,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> MdmaChannel<'d> {
    pub unsafe fn new(channel: u8) -> Self {
        assert!(channel < MDMA_CHANNEL_COUNT as u8, "MDMA has channels 0..15");
        Self {
            channel,
            _phantom: PhantomData,
        }
    }

    #[inline]
    pub(crate) fn ch(&self) -> usize {
        self.channel as usize
    }

    pub(crate) fn state(&self) -> &'static ChannelState {
        &CHANNEL_STATE[self.ch()]
    }

    pub fn abort(&mut self) {
        regs::modify_cr(self.ch(), |v| v & !regs::cr::EN);
        while regs::read_cr(self.ch()) & regs::cr::EN != 0 {}
        self.clear_all_flags();
    }

    pub(crate) fn clear_all_flags(&self) {
        regs::write_ifcr(self.ch(), regs::ifcr::ALL);
    }

    pub(crate) unsafe fn start_transfer(
        &mut self,
        src: u32,
        dst: u32,
        total_bytes: usize,
        src_data_size: DataSize,
        dst_data_size: DataSize,
        options: &MdmaTransferOptions,
    ) {
        assert!(total_bytes > 0, "MDMA: zero-length transfer");
        assert!(total_bytes <= 0x10000 * 0x1000, "MDMA: transfer too large");

        let buf_len = options.buffer_length.min(128).max(1);
        let effective_buf = {
            let s = src_data_size.bytes();
            let d = dst_data_size.bytes();
            let lcm = lcm(s, d);
            let mut b = buf_len as usize;
            b = (b / lcm) * lcm;
            if b == 0 {
                b = lcm;
            }
            b.min(128) as u8
        };

        let (block_size, block_count) = factorize_transfer(total_bytes, 0x10000, 0x1000);

        let state = self.state();
        let ch = self.ch();

        state.complete_count.store(0, Ordering::Release);
        state.error.store(0, Ordering::Release);

        regs::modify_cr(ch, |v| v & !regs::cr::EN);
        while regs::read_cr(ch) & regs::cr::EN != 0 {}
        self.clear_all_flags();

        regs::write_sar(ch, src);
        regs::write_dar(ch, dst);

        let is_sw_triggered = options.trigger.is_none();

        let (pke, pam) = match options.packing {
            PackingAlignment::Pack => (true, 0b00),
            PackingAlignment::RightAlignZeroPad => (false, 0b00),
            PackingAlignment::RightAlignSignExtend => (false, 0b10),
            PackingAlignment::LeftAlignZeroPad => (false, 0b01),
        };

        let trgm = if is_sw_triggered {
            0
        } else {
            options.trigger_mode as u32
        };

        regs::write_tcr(
            ch,
            regs::build_tcr(
                src_data_size as u32,
                src_data_size as u32,
                options.source_increment as u32,
                dst_data_size as u32,
                dst_data_size as u32,
                options.destination_increment as u32,
                0,
                0, // single burst
                (effective_buf - 1) as u32,
                pke,
                pam,
                trgm,
                is_sw_triggered,
            ),
        );

        regs::write_bndtr(
            ch,
            regs::build_bndtr(
                block_size as u32,
                if block_count > 1 { (block_count - 1) as u32 } else { 0 },
                false,
                false,
            ),
        );

        regs::write_brur(ch, 0);

        regs::write_tbr(
            ch,
            regs::build_tbr(
                options.trigger.map(|t| t as u32).unwrap_or(0),
                is_tcm_address(src),
                is_tcm_address(dst),
            ),
        );

        regs::write_lar(ch, 0);
        regs::write_mar(ch, 0);
        regs::write_mdr(ch, 0);

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

        if is_sw_triggered {
            regs::modify_cr(ch, |v| v | regs::cr::SWRQ);
        }
    }

    fn poll_complete(&self) -> Poll<Result<(), MdmaError>> {
        let state = self.state();
        compiler_fence(Ordering::SeqCst);

        if state.error.load(Ordering::Acquire) != 0 {
            fence(Ordering::Acquire);
            return Poll::Ready(Err(MdmaError::TransferError));
        }
        if state.complete_count.load(Ordering::Acquire) != 0 {
            fence(Ordering::Acquire);
            return Poll::Ready(Ok(()));
        }
        if regs::read_cr(self.ch()) & regs::cr::EN == 0 {
            fence(Ordering::Acquire);
            if state.complete_count.load(Ordering::Acquire) != 0 {
                return Poll::Ready(Ok(()));
            }
            return Poll::Ready(Err(MdmaError::TransferError));
        }
        Poll::Pending
    }

    pub async fn wait(&self) -> Result<(), MdmaError> {
        poll_fn(|cx| {
            self.state().waker.register(cx.waker());
            self.poll_complete()
        })
        .await
    }

    pub fn blocking_wait(&self) -> Result<(), MdmaError> {
        loop {
            match self.poll_complete() {
                Poll::Ready(result) => return result,
                Poll::Pending => {}
            }
        }
    }

    // ─── High-level transfer methods ─────────────────────────────────

    pub async unsafe fn transfer_mem2mem<T: Sized>(
        &mut self,
        src: &[T],
        dst: *mut T,
        options: &MdmaTransferOptions,
    ) -> Result<(), MdmaError> {
        let data_size = DataSize::from_bytes(core::mem::size_of::<T>());
        let total_bytes = src.len() * core::mem::size_of::<T>();
        let mut opts = options.clone();
        opts.source_increment = IncrementMode::Increment;
        opts.destination_increment = IncrementMode::Increment;
        fence(Ordering::SeqCst);
        self.start_transfer(
            src.as_ptr() as u32,
            dst as u32,
            total_bytes,
            data_size,
            data_size,
            &opts,
        );
        self.wait().await
    }

    pub async fn copy_slice<T: Copy + Sized>(&mut self, src: &[T], dst: &mut [T]) -> Result<(), MdmaError> {
        assert!(dst.len() >= src.len());
        unsafe {
            self.transfer_mem2mem(src, dst.as_mut_ptr(), &MdmaTransferOptions::default())
                .await
        }
    }

    pub async unsafe fn write_to_peripheral<T: Sized>(
        &mut self,
        src: &[T],
        peri_addr: *mut T,
        options: &MdmaTransferOptions,
    ) -> Result<(), MdmaError> {
        let data_size = DataSize::from_bytes(core::mem::size_of::<T>());
        let total_bytes = src.len() * core::mem::size_of::<T>();
        let mut opts = options.clone();
        opts.source_increment = IncrementMode::Increment;
        opts.destination_increment = IncrementMode::Fixed;
        fence(Ordering::SeqCst);
        self.start_transfer(
            src.as_ptr() as u32,
            peri_addr as u32,
            total_bytes,
            data_size,
            data_size,
            &opts,
        );
        self.wait().await
    }

    pub async unsafe fn read_from_peripheral<T: Sized>(
        &mut self,
        peri_addr: *const T,
        dst: &mut [T],
        options: &MdmaTransferOptions,
    ) -> Result<(), MdmaError> {
        let data_size = DataSize::from_bytes(core::mem::size_of::<T>());
        let total_bytes = dst.len() * core::mem::size_of::<T>();
        let mut opts = options.clone();
        opts.source_increment = IncrementMode::Fixed;
        opts.destination_increment = IncrementMode::Increment;
        fence(Ordering::SeqCst);
        self.start_transfer(
            peri_addr as u32,
            dst.as_mut_ptr() as u32,
            total_bytes,
            data_size,
            data_size,
            &opts,
        );
        self.wait().await
        // self.blocking_wait()
    }

    pub async unsafe fn write_to_peripheral_packed<SRC: Sized, DST: Sized>(
        &mut self,
        src: &[SRC],
        peri_addr: *mut DST,
        options: &MdmaTransferOptions,
    ) -> Result<(), MdmaError> {
        let src_size = DataSize::from_bytes(core::mem::size_of::<SRC>());
        let dst_size = DataSize::from_bytes(core::mem::size_of::<DST>());
        let total_bytes = src.len() * core::mem::size_of::<SRC>();
        let mut opts = options.clone();
        opts.source_increment = IncrementMode::Increment;
        opts.destination_increment = IncrementMode::Fixed;
        fence(Ordering::SeqCst);
        self.start_transfer(
            src.as_ptr() as u32,
            peri_addr as u32,
            total_bytes,
            src_size,
            dst_size,
            &opts,
        );
        self.wait().await
    }
}

impl Drop for MdmaChannel<'_> {
    fn drop(&mut self) {
        self.abort();
    }
}

// ============================================================================
// IRQ handler
// ============================================================================

pub unsafe fn mdma_on_irq() {
    let gisr = regs::gisr0();

    for ch_idx in 0..MDMA_CHANNEL_COUNT {
        if gisr & (1 << ch_idx) == 0 {
            continue;
        }

        let isr = regs::read_isr(ch_idx);
        let state = &CHANNEL_STATE[ch_idx];

        if isr & regs::isr::TEIF != 0 {
            regs::write_ifcr(ch_idx, regs::ifcr::CTEIF);
            regs::modify_cr(ch_idx, |v| v & !regs::cr::EN);
            state.error.store(1, Ordering::Release);
            state.waker.wake();
        }
        if isr & regs::isr::CTCIF != 0 {
            regs::write_ifcr(ch_idx, regs::ifcr::CCTCIF);
            state.complete_count.fetch_add(1, Ordering::Release);
            state.waker.wake();
        }
        if isr & regs::isr::TCIF != 0 {
            regs::write_ifcr(ch_idx, regs::ifcr::CLTCIF);
            state.waker.wake();
        }
        if isr & regs::isr::BTIF != 0 {
            regs::write_ifcr(ch_idx, regs::ifcr::CBTIF);
            state.waker.wake();
        }
        if isr & regs::isr::BRTIF != 0 {
            regs::write_ifcr(ch_idx, regs::ifcr::CBRTIF);
            state.waker.wake();
        }
    }
}

// ============================================================================
// Initialization
// ============================================================================

/// MDMA interrupt number (STM32H7 = 122).
#[derive(Copy, Clone)]
struct MdmaIrq;

unsafe impl cortex_m::interrupt::InterruptNumber for MdmaIrq {
    fn number(self) -> u16 {
        122
    }
}

/// Enable MDMA clock and interrupt. Call once before any MDMA operations.
///
/// # Safety
///
/// Must be called once at startup.
pub unsafe fn init() {
    regs::enable_clock();

    let mut nvic = cortex_m::Peripherals::steal().NVIC;
    nvic.set_priority(MdmaIrq, 4);
    cortex_m::peripheral::NVIC::unmask(MdmaIrq);
}

pub fn set_interrupt_priority(priority: u8) {
    unsafe {
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(MdmaIrq, priority);
    }
}

// ============================================================================
// Utility
// ============================================================================

const fn gcd(a: usize, b: usize) -> usize {
    if b == 0 { a } else { gcd(b, a % b) }
}

const fn lcm(a: usize, b: usize) -> usize {
    (a / gcd(a, b)) * b
}

fn factorize_transfer(total_bytes: usize, max_block: usize, max_blocks: usize) -> (usize, usize) {
    if total_bytes <= max_block {
        return (total_bytes, 1);
    }
    let mut block_count = total_bytes.div_ceil(max_block);
    let mut block_size = total_bytes.div_ceil(block_count);
    loop {
        if block_count * block_size == total_bytes {
            return (block_size, block_count);
        }
        block_count += 1;
        block_size = total_bytes.div_ceil(block_count);
        assert!(
            block_count <= max_blocks,
            "MDMA: cannot factorize transfer size {}",
            total_bytes
        );
    }
}
