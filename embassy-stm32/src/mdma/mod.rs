//! MDMA (Master DMA) driver for STM32H7 on Embassy.
//!
//! Uses raw register access at 0x5200_0000 — does not depend on
//! stm32-metapac having typed MDMA register definitions.

pub mod mdma;
pub mod mdma_fmc_display;
pub mod mdma_linked_list;
pub mod regs;

pub use mdma::*;
pub use mdma_fmc_display::MdmaFmcDisplay;
pub use mdma_linked_list::{LinkedListNodeConfig, MdmaLinkedListNode};
