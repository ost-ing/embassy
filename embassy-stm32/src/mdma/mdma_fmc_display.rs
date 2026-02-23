//! MDMA-accelerated FMC display driver helper.
//!
//! Provides zero-copy, async framebuffer transfers from any memory region
//! (including DTCM) to an FMC-connected display.
//!
//! # Typical use with parallel RGB displays (ILI9341, ST7789, etc.)
//!
//! ```rust,ignore
//! let mut display = MdmaFmcDisplay::new(
//!     0, // MDMA channel
//!     0x6008_0000 as *mut u16, // FMC data address (RS=1)
//!     0x6000_0000 as *mut u16, // FMC command address (RS=0)
//! );
//!
//! // Set window, then blast pixels
//! display.set_address_window(0, 0, 239, 319).await;
//! display.write_pixels(&framebuffer).await;
//! ```

use core::sync::atomic::{Ordering, fence};

use super::mdma::{IncrementMode, MdmaChannel, MdmaError, MdmaTransferOptions, Priority, TriggerMode};

/// MDMA-accelerated FMC display interface.
///
/// Wraps an MDMA channel and provides high-level methods for writing
/// framebuffer data to an FMC-connected parallel display.
pub struct MdmaFmcDisplay<'d> {
    channel: MdmaChannel<'d>,
    data_addr: *mut u16,
    cmd_addr: *mut u16,
    options: MdmaTransferOptions,
}

impl<'d> MdmaFmcDisplay<'d> {
    /// Create a new MDMA FMC display driver.
    ///
    /// - `mdma_channel`: MDMA channel number (0..15)
    /// - `data_addr`: FMC address for data writes (RS/DC pin high)
    /// - `cmd_addr`: FMC address for command writes (RS/DC pin low)
    ///
    /// # Safety
    ///
    /// The MDMA must be initialized ([`mdma::init()`]) and the FMC must be
    /// configured for the display before calling this.
    pub unsafe fn new(mdma_channel: u8, data_addr: *mut u16, cmd_addr: *mut u16) -> Self {
        Self {
            channel: MdmaChannel::new(mdma_channel),
            data_addr,
            cmd_addr,
            options: MdmaTransferOptions {
                priority: Priority::VeryHigh,
                trigger: None,
                trigger_mode: TriggerMode::Block,
                buffer_length: 128,
                source_increment: IncrementMode::Increment,
                destination_increment: IncrementMode::Fixed,
                ..Default::default()
            },
        }
    }

    /// Write a command byte to the display (blocking, since commands are short).
    pub fn write_cmd(&self, cmd: u16) {
        unsafe {
            core::ptr::write_volatile(self.cmd_addr, cmd);
        }
    }

    /// Write a data byte/word to the display (blocking, single value).
    pub fn write_data(&self, data: u16) {
        unsafe {
            core::ptr::write_volatile(self.data_addr, data);
        }
    }

    /// Write a slice of pixel data (RGB565 u16) to the display using MDMA.
    ///
    /// This is the main workhorse — it DMAs the entire buffer to the display
    /// at maximum bus speed without CPU involvement.
    ///
    /// The source buffer can be in ANY memory region (DTCM, AXI SRAM, etc.).
    /// The MDMA will automatically select the correct bus.
    pub async fn write_pixels(&mut self, pixels: &[u16]) -> Result<(), MdmaError> {
        if pixels.is_empty() {
            return Ok(());
        }

        fence(Ordering::SeqCst);

        unsafe {
            self.channel
                .write_to_peripheral(pixels, self.data_addr, &self.options)
                .await
        }
    }

    /// Write pixel data as u32 words (two RGB565 pixels per word).
    ///
    /// Useful when your framebuffer is packed as u32 for alignment/speed.
    /// The MDMA source is u32 (incrementing), destination is u16 (fixed FMC register).
    /// Packing mode handles the u32→u16 conversion automatically.
    pub async fn write_pixels_packed_u32(&mut self, pixels: &[u32]) -> Result<(), MdmaError> {
        if pixels.is_empty() {
            return Ok(());
        }

        fence(Ordering::SeqCst);

        unsafe {
            self.channel
                .write_to_peripheral_packed::<u32, u16>(pixels, self.data_addr, &self.options)
                .await
        }
    }

    /// Write raw bytes to the data register.
    pub async fn write_data_bytes(&mut self, data: &[u8]) -> Result<(), MdmaError> {
        if data.is_empty() {
            return Ok(());
        }

        fence(Ordering::SeqCst);

        unsafe {
            self.channel
                .write_to_peripheral_packed::<u8, u16>(data, self.data_addr, &self.options)
                .await
        }
    }

    /// Set address window on a typical ILI9341/ST7789 display, then prepare for pixel writes.
    ///
    /// After calling this, call `write_pixels()` with exactly
    /// `(x1-x0+1) * (y1-y0+1)` pixels.
    pub fn set_address_window(&self, x0: u16, y0: u16, x1: u16, y1: u16) {
        // Column Address Set (0x2A)
        self.write_cmd(0x2A);
        self.write_data(x0 >> 8);
        self.write_data(x0 & 0xFF);
        self.write_data(x1 >> 8);
        self.write_data(x1 & 0xFF);

        // Page Address Set (0x2B)
        self.write_cmd(0x2B);
        self.write_data(y0 >> 8);
        self.write_data(y0 & 0xFF);
        self.write_data(y1 >> 8);
        self.write_data(y1 & 0xFF);

        // Memory Write (0x2C)
        self.write_cmd(0x2C);
    }

    /// Full-screen write: set window to full display, then DMA the framebuffer.
    pub async fn write_framebuffer(&mut self, width: u16, height: u16, pixels: &[u16]) -> Result<(), MdmaError> {
        self.set_address_window(0, 0, width - 1, height - 1);
        self.write_pixels(pixels).await
    }

    /// Partial update: write a rectangular region of the display.
    pub async fn write_region(&mut self, x: u16, y: u16, w: u16, h: u16, pixels: &[u16]) -> Result<(), MdmaError> {
        assert!(
            pixels.len() >= (w as usize) * (h as usize),
            "pixel buffer too small for region"
        );
        self.set_address_window(x, y, x + w - 1, y + h - 1);
        self.write_pixels(&pixels[..(w as usize) * (h as usize)]).await
    }

    /// Get mutable access to the underlying MDMA channel for custom transfers.
    pub fn channel_mut(&mut self) -> &mut MdmaChannel<'d> {
        &mut self.channel
    }

    /// Set the transfer priority.
    pub fn set_priority(&mut self, priority: Priority) {
        self.options.priority = priority;
    }

    /// Set the buffer length (controls how often MDMA checks for other requests).
    /// Lower = better interleaving with other MDMA channels.
    /// Higher = better throughput.
    pub fn set_buffer_length(&mut self, len: u8) {
        self.options.buffer_length = len.min(128).max(1);
    }
}
