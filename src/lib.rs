//! I²C [Drone OS] driver for STM32F4 micro-controllers.
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
