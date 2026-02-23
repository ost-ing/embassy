//! Raw MDMA register access for STM32H7.
//!
//! The stm32-metapac for some H7 variants doesn't generate typed MDMA register
//! definitions. This module provides direct register access via the MDMA base
//! address (0x5200_0000).

use core::ptr::{read_volatile, write_volatile};

/// MDMA peripheral base address (STM32H7).
const MDMA_BASE: u32 = 0x5200_0000;

/// Channel register block offset = 0x40 + ch * 0x40.
const fn ch_base(ch: usize) -> u32 {
    MDMA_BASE + 0x40 + (ch as u32) * 0x40
}

// ─── Global registers ────────────────────────────────────────────────

/// Read GISR0 — global interrupt status. Bit `i` = channel `i` has pending IRQ.
#[inline(always)]
pub fn gisr0() -> u32 {
    unsafe { read_volatile(MDMA_BASE as *const u32) }
}

// ─── Per-channel register offsets ────────────────────────────────────

const CISR: u32 = 0x00;
const CIFCR: u32 = 0x04;
#[allow(dead_code)]
const CESR: u32 = 0x08;
const CCR: u32 = 0x10;
const CTCR: u32 = 0x14;
const CBNDTR: u32 = 0x18;
const CSAR: u32 = 0x1C;
const CDAR: u32 = 0x20;
const CBRUR: u32 = 0x24;
const CLAR: u32 = 0x28;
const CTBR: u32 = 0x2C;
const CMAR: u32 = 0x34;
const CMDR: u32 = 0x38;

#[inline(always)]
fn ch_read(ch: usize, offset: u32) -> u32 {
    unsafe { read_volatile((ch_base(ch) + offset) as *const u32) }
}

#[inline(always)]
fn ch_write(ch: usize, offset: u32, val: u32) {
    unsafe { write_volatile((ch_base(ch) + offset) as *mut u32, val) }
}

#[inline(always)]
fn ch_modify(ch: usize, offset: u32, f: impl FnOnce(u32) -> u32) {
    let v = ch_read(ch, offset);
    ch_write(ch, offset, f(v));
}

// ─── CISR — Channel interrupt/status register ────────────────────────

pub mod isr {
    pub const TEIF: u32 = 1 << 0;
    pub const CTCIF: u32 = 1 << 1;
    pub const BRTIF: u32 = 1 << 2;
    pub const BTIF: u32 = 1 << 3;
    pub const TCIF: u32 = 1 << 4;
    #[allow(dead_code)]
    pub const CRQA: u32 = 1 << 16;
}

#[inline(always)]
pub fn read_isr(ch: usize) -> u32 {
    ch_read(ch, CISR)
}

// ─── CIFCR — Channel interrupt flag clear register ───────────────────

pub mod ifcr {
    pub const CTEIF: u32 = 1 << 0;
    pub const CCTCIF: u32 = 1 << 1;
    pub const CBRTIF: u32 = 1 << 2;
    pub const CBTIF: u32 = 1 << 3;
    pub const CLTCIF: u32 = 1 << 4;
    pub const ALL: u32 = CTEIF | CCTCIF | CBRTIF | CBTIF | CLTCIF;
}

#[inline(always)]
pub fn write_ifcr(ch: usize, val: u32) {
    ch_write(ch, CIFCR, val);
}

// ─── CCR — Channel control register ─────────────────────────────────

pub mod cr {
    pub const EN: u32 = 1 << 0;
    pub const TEIE: u32 = 1 << 1;
    pub const CTCIE: u32 = 1 << 2;
    pub const BRTIE: u32 = 1 << 3;
    pub const BTIE: u32 = 1 << 4;
    pub const TCIE: u32 = 1 << 5;
    pub const PL_SHIFT: u32 = 6;
    #[allow(dead_code)]
    pub const PL_MASK: u32 = 0b11 << PL_SHIFT;
    pub const BEX: u32 = 1 << 12;
    pub const HEX: u32 = 1 << 13;
    pub const WEX: u32 = 1 << 14;
    pub const SWRQ: u32 = 1 << 16;
}

#[inline(always)]
pub fn read_cr(ch: usize) -> u32 {
    ch_read(ch, CCR)
}

#[inline(always)]
pub fn write_cr(ch: usize, val: u32) {
    ch_write(ch, CCR, val);
}

#[inline(always)]
pub fn modify_cr(ch: usize, f: impl FnOnce(u32) -> u32) {
    ch_modify(ch, CCR, f);
}

// ─── CTCR — Channel transfer configuration register ─────────────────

pub mod tcr {
    pub const SINCOS_SHIFT: u32 = 0;
    pub const SSIZE_SHIFT: u32 = 2;
    pub const SINC_SHIFT: u32 = 4;
    pub const DINCOS_SHIFT: u32 = 6;
    pub const DSIZE_SHIFT: u32 = 8;
    pub const DINC_SHIFT: u32 = 10;
    pub const SBURST_SHIFT: u32 = 12;
    pub const DBURST_SHIFT: u32 = 15;
    pub const TLEN_SHIFT: u32 = 18;
    #[allow(dead_code)]
    pub const TLEN_MASK: u32 = 0x7F << TLEN_SHIFT;
    pub const PKE: u32 = 1 << 25;
    pub const PAM_SHIFT: u32 = 26;
    pub const TRGM_SHIFT: u32 = 28;
    pub const SWRM: u32 = 1 << 30;
    #[allow(dead_code)]
    pub const BWM: u32 = 1 << 31;
}

#[inline(always)]
pub fn write_tcr(ch: usize, val: u32) {
    ch_write(ch, CTCR, val);
}

#[inline(always)]
pub fn read_tcr(ch: usize) -> u32 {
    ch_read(ch, CTCR)
}

// ─── CBNDTR — Block number of data register ─────────────────────────

pub mod bndtr {
    pub const BNDT_MASK: u32 = 0x1_FFFF;
    pub const BRSUM: u32 = 1 << 18;
    pub const BRDUM: u32 = 1 << 19;
    pub const BRC_SHIFT: u32 = 20;
    #[allow(dead_code)]
    pub const BRC_MASK: u32 = 0xFFF << BRC_SHIFT;
}

#[inline(always)]
pub fn write_bndtr(ch: usize, val: u32) {
    ch_write(ch, CBNDTR, val);
}

// ─── Address registers ──────────────────────────────────────────────

#[inline(always)]
pub fn write_sar(ch: usize, addr: u32) {
    ch_write(ch, CSAR, addr);
}

#[inline(always)]
pub fn write_dar(ch: usize, addr: u32) {
    ch_write(ch, CDAR, addr);
}

// ─── CBRUR — Block repeat address update ────────────────────────────

#[inline(always)]
pub fn write_brur(ch: usize, val: u32) {
    ch_write(ch, CBRUR, val);
}

// ─── CLAR — Link address register ───────────────────────────────────

#[inline(always)]
pub fn write_lar(ch: usize, val: u32) {
    ch_write(ch, CLAR, val);
}

// ─── CTBR — Trigger and bus selection ───────────────────────────────

pub mod tbr {
    pub const TSEL_MASK: u32 = 0x3F;
    pub const SBUS: u32 = 1 << 16;
    pub const DBUS: u32 = 1 << 17;
}

#[inline(always)]
pub fn write_tbr(ch: usize, val: u32) {
    ch_write(ch, CTBR, val);
}

// ─── CMAR / CMDR — Mask address / data ──────────────────────────────

#[inline(always)]
pub fn write_mar(ch: usize, val: u32) {
    ch_write(ch, CMAR, val);
}

#[inline(always)]
pub fn write_mdr(ch: usize, val: u32) {
    ch_write(ch, CMDR, val);
}

// ─── RCC and NVIC helpers ───────────────────────────────────────────

/// RCC AHB3ENR address (STM32H7).
const RCC_AHB3ENR: u32 = 0x5802_4400 + 0xD4;

/// MDMA clock enable bit in AHB3ENR.
const MDMAEN_BIT: u32 = 1 << 0;

/// Enable the MDMA peripheral clock via RCC.
pub fn enable_clock() {
    unsafe {
        let val = read_volatile(RCC_AHB3ENR as *const u32);
        write_volatile(RCC_AHB3ENR as *mut u32, val | MDMAEN_BIT);
    }
    cortex_m::asm::dsb();
}

// ─── Register builders ──────────────────────────────────────────────

/// Build a CTCR register value from individual field values.
pub fn build_tcr(
    sincos: u32,
    ssize: u32,
    sinc: u32,
    dincos: u32,
    dsize: u32,
    dinc: u32,
    sburst: u32,
    dburst: u32,
    tlen: u32,
    pke: bool,
    pam: u32,
    trgm: u32,
    swrm: bool,
) -> u32 {
    (sincos & 0x3) << tcr::SINCOS_SHIFT
        | (ssize & 0x3) << tcr::SSIZE_SHIFT
        | (sinc & 0x3) << tcr::SINC_SHIFT
        | (dincos & 0x3) << tcr::DINCOS_SHIFT
        | (dsize & 0x3) << tcr::DSIZE_SHIFT
        | (dinc & 0x3) << tcr::DINC_SHIFT
        | (sburst & 0x7) << tcr::SBURST_SHIFT
        | (dburst & 0x7) << tcr::DBURST_SHIFT
        | (tlen & 0x7F) << tcr::TLEN_SHIFT
        | if pke { tcr::PKE } else { 0 }
        | (pam & 0x3) << tcr::PAM_SHIFT
        | (trgm & 0x3) << tcr::TRGM_SHIFT
        | if swrm { tcr::SWRM } else { 0 }
}

/// Build a CBNDTR register value.
pub fn build_bndtr(bndt: u32, brc: u32, brsum: bool, brdum: bool) -> u32 {
    (bndt & bndtr::BNDT_MASK)
        | ((brc & 0xFFF) << bndtr::BRC_SHIFT)
        | if brsum { bndtr::BRSUM } else { 0 }
        | if brdum { bndtr::BRDUM } else { 0 }
}

/// Build a CTBR register value.
pub fn build_tbr(tsel: u32, sbus_ahb: bool, dbus_ahb: bool) -> u32 {
    (tsel & tbr::TSEL_MASK) | if sbus_ahb { tbr::SBUS } else { 0 } | if dbus_ahb { tbr::DBUS } else { 0 }
}

/// Build a CCR register value.
#[inline(always)]
pub fn build_cr(
    pl: u32,
    teie: bool,
    ctcie: bool,
    brtie: bool,
    btie: bool,
    tcie: bool,
    bex: bool,
    hex: bool,
    wex: bool,
    en: bool,
) -> u32 {
    let mut cr_val = 0u32;

    if en {
        cr_val |= cr::EN;
    }
    if teie {
        cr_val |= cr::TEIE;
    }
    if ctcie {
        cr_val |= cr::CTCIE;
    }
    if brtie {
        cr_val |= cr::BRTIE;
    }
    if btie {
        cr_val |= cr::BTIE;
    }
    if tcie {
        cr_val |= cr::TCIE;
    }

    cr_val |= (pl & 0x3) << cr::PL_SHIFT;

    if bex {
        cr_val |= cr::BEX;
    }
    if hex {
        cr_val |= cr::HEX;
    }
    if wex {
        cr_val |= cr::WEX;
    }

    cr_val
}
