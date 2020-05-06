//! I²C [Drone OS] driver for STM32F4 micro-controllers.
//!
//! # Restrictions
//!
//! * Transmission and reception works only through DMA channels and
//! interrupts. Interrupt only and polling methods are not in the scope of this
//! crate.
//!
//! * Errors from peripherals are handled via panicking.
//!
//! # Usage
//!
//! Add the crate to your `Cargo.toml` dependencies:
//!
//! ```toml
//! [dependencies]
//! smartoris-i2c = { version = "0.1.0" }
//! ```
//!
//! Add or extend `std` feature as follows:
//!
//! ```toml
//! [features]
//! std = ["smartoris-i2c/std"]
//! ```
//!
//! Example of initializing the driver for I2C1, DMA1 CH5/CH6, and B6/B7 pins:
//!
//! ```no_run
//! # #![feature(const_fn)]
//! # use drone_stm32_map::stm32_reg_tokens;
//! # use drone_core::token::Token;
//! # stm32_reg_tokens! {
//! #     struct Regs;
//! #     !scb_ccr;
//! #     !mpu_type; !mpu_ctrl; !mpu_rnr; !mpu_rbar; !mpu_rasr;
//! # }
//!
//! mod thr {
//!     pub use drone_cortexm::thr::init;
//!     pub use drone_stm32_map::thr::*;
//!
//!     use drone_cortexm::thr;
//!
//!     thr::vtable! {
//!         use Thr;
//!         pub struct Vtable;
//!         pub struct Handlers;
//!         pub struct Thrs;
//!         pub struct ThrsInit;
//!         static THREADS;
//!
//!         /// DMA1 Stream5 global interrupt.
//!         pub 16: DMA1_CH5;
//!         /// DMA1 Stream6 global interrupt.
//!         pub 17: DMA1_CH6;
//!         /// I²C1 event interrupt.
//!         pub 31: I2C1_EV;
//!         /// I²C1 error interrupt.
//!         pub 32: I2C1_ER;
//!     }
//!
//!     thr! {
//!         use THREADS;
//!         pub struct Thr {}
//!         pub struct ThrLocal {}
//!     }
//! }
//!
//! use crate::thr::ThrsInit;
//! use drone_cortexm::{reg::prelude::*, thr::prelude::*};
//! use drone_stm32_map::periph::{
//!     dma::{periph_dma1, periph_dma1_ch5, periph_dma1_ch6},
//!     gpio::periph_gpio_b,
//!     i2c::periph_i2c1,
//! };
//! use smartoris_i2c::{I2CDrv, I2CSetup};
//!
//! fn handler(reg: Regs, thr_init: ThrsInit) {
//!     let thr = thr::init(thr_init);
//!
//!     // Enable interrupts.
//!     thr.dma1_ch5.enable_int();
//!     thr.dma1_ch6.enable_int();
//!     thr.i2c1_ev.enable_int();
//!     thr.i2c1_er.enable_int();
//!
//!     // Configure GPIO pins.
//!     let gpio_b = periph_gpio_b!(reg);
//!     gpio_b.rcc_busenr_gpioen.set_bit(); // IO port clock enable
//!     gpio_b.gpio_otyper.store(|r| {
//!         r.set_ot6() // output open-drain
//!             .set_ot7() // output open-drain
//!     });
//!     gpio_b.gpio_ospeedr.store(|r| {
//!         r.write_ospeedr6(0b11) // very high speed
//!             .write_ospeedr7(0b11) // very high speed
//!     });
//!     gpio_b.gpio_pupdr.store(|r| {
//!         r.write_pupdr6(0b00) // no pull-up, pull-down
//!             .write_pupdr7(0b00) // no pull-up, pull-down
//!     });
//!     gpio_b.gpio_afrl.store(|r| {
//!         r.write_afrl6(0b0100) // I2C1/I2C2/I2C3
//!             .write_afrl7(0b0100) // I2C1/I2C2/I2C3
//!     });
//!     gpio_b.gpio_moder.store(|r| {
//!         r.write_moder6(0b10) // alternate function
//!             .write_moder7(0b10) // alternate function
//!     });
//!     gpio_b.rcc_busenr_gpioen.clear_bit(); // IO port clock disable
//!
//!     periph_dma1!(reg).rcc_busenr_dmaen.set_bit(); // DMA clock enable
//!
//!     // Set up the driver.
//!     let i2c1 = I2CDrv::init(I2CSetup {
//!         i2c: periph_i2c1!(reg),
//!         i2c_ev: thr.i2c1_ev,
//!         i2c_er: thr.i2c1_er,
//!         dma_tx: periph_dma1_ch6!(reg),
//!         dma_tx_int: thr.dma1_ch6,
//!         dma_tx_ch: 1,    // I2C1_TX
//!         dma_tx_pl: 0b11, // very high
//!         dma_rx: periph_dma1_ch5!(reg),
//!         dma_rx_int: thr.dma1_ch5,
//!         dma_rx_ch: 1,    // I2C1_RX
//!         dma_rx_pl: 0b11, // very high
//!     });
//! }
//! # fn main() {
//! #     unsafe { handler(Regs::take(), ThrsInit::take()) };
//! # }
//! ```
//!
//! Example of usage:
//!
//! ```no_run
//! # #![feature(const_fn)]
//! # use drone_stm32_map::periph::{
//! #     dma::ch::{Dma1Ch5, Dma1Ch6},
//! #     i2c::I2C1,
//! # };
//! # mod thr {
//! #     use drone_stm32_map::thr::*;
//! #     drone_cortexm::thr::vtable! {
//! #         use Thr;
//! #         pub struct Vtable;
//! #         pub struct Handlers;
//! #         pub struct Thrs;
//! #         pub struct ThrsInit;
//! #         static THREADS;
//! #         pub 16: DMA1_CH5;
//! #         pub 17: DMA1_CH6;
//! #         pub 31: I2C1_EV;
//! #         pub 32: I2C1_ER;
//! #     }
//! #     drone_cortexm::thr! {
//! #         use THREADS;
//! #         pub struct Thr {}
//! #         pub struct ThrLocal {}
//! #     }
//! # }
//! # async fn handler() {
//! # let mut i2c1: smartoris_i2c::I2CDrv<
//! #     I2C1,
//! #     thr::I2C1Ev,
//! #     thr::I2C1Er,
//! #     Dma1Ch6,
//! #     thr::Dma1Ch6,
//! #     Dma1Ch5,
//! #     thr::Dma1Ch5,
//! # > = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
//! let mut buf = [0; 1];
//!
//! // Generate a Start signal, initiate write transaction for `0x39` slave
//! // address, and transmit `0x92` byte.
//! i2c1.master_write(0x39, &[0x92])
//!     .await
//!     // Generate Repeated Start signal without releasing the bus, initiate
//!     // read transaction for `0x39` slave address, and receive 1 byte.
//!     .read(0x39, &mut buf)
//!     .await;
//! // The master session is dropped here, automatically sending a Stop signal.
//!
//! println!("{:02X}", buf[0]);
//! # }
//! # fn main() {}
//! ```
//!
//! [Drone OS]: https://www.drone-os.com/

#![feature(never_type)]
#![feature(prelude_import)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::module_name_repetitions,
    clippy::wildcard_imports
)]
#![cfg_attr(not(feature = "std"), no_std)]

mod diverged;
mod drv;
mod master;

pub use self::{
    drv::{I2CDrv, I2CSetup},
    master::I2CMaster,
};

#[prelude_import]
#[allow(unused_imports)]
use drone_core::prelude::*;
