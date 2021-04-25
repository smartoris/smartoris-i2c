[![crates.io](https://img.shields.io/crates/v/smartoris-i2c.svg)](https://crates.io/crates/smartoris-i2c)
![maintenance](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg)

# smartoris-i2c

I²C [Drone OS] driver for STM32F4 micro-controllers.

## Limitations

* Transmission and reception works only through DMA channels with
interrupts. Polling and interrupt only methods are not supported.

* Errors from peripherals are handled via panicking.

* Only the master role is implemented.

## Usage

Add the crate to your `Cargo.toml` dependencies:

```toml
[dependencies]
smartoris-i2c = { version = "0.1.0" }
```

Add or extend `std` feature as follows:

```toml
[features]
std = ["smartoris-i2c/std"]
```

Example of initializing the driver for I2C1, DMA1 CH5/CH6, and B6/B7 pins:

```rust
mod thr {
    pub use drone_cortexm::thr::init;
    pub use drone_stm32_map::thr::*;

    use drone_cortexm::thr;

    thr::nvic! {
        thread => pub Thr {};
        local => pub ThrLocal {};
        vtable => pub Vtable;
        index => pub Thrs;
        init => pub ThrsInit;

        threads => {
            interrupts => {
                /// DMA1 Stream5 global interrupt.
                16: pub dma1_ch5;
                /// DMA1 Stream6 global interrupt.
                17: pub dma1_ch6;
                /// I²C1 event interrupt.
                31: pub i2c1_ev;
                /// I²C1 error interrupt.
                32: pub i2c1_er;
            };
        };
    }
}

use crate::thr::ThrsInit;
use drone_cortexm::{reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
    dma::{periph_dma1, periph_dma1_ch5, periph_dma1_ch6},
    gpio::periph_gpio_b,
    i2c::periph_i2c1,
};
use smartoris_i2c::{I2CDrv, I2CMode, I2CSetup};

fn handler(reg: Regs, thr_init: ThrsInit) {
    let thr = thr::init(thr_init);

    // Enable interrupts.
    thr.dma1_ch5.enable_int();
    thr.dma1_ch6.enable_int();
    thr.i2c1_ev.enable_int();
    thr.i2c1_er.enable_int();

    // Configure GPIO pins.
    let gpio_b = periph_gpio_b!(reg);
    gpio_b.rcc_busenr_gpioen.set_bit(); // IO port clock enable
    gpio_b.gpio_otyper.store(|r| {
        r.set_ot6() // output open-drain
            .set_ot7() // output open-drain
    });
    gpio_b.gpio_ospeedr.store(|r| {
        r.write_ospeedr6(0b11) // very high speed
            .write_ospeedr7(0b11) // very high speed
    });
    gpio_b.gpio_pupdr.store(|r| {
        r.write_pupdr6(0b00) // no pull-up, pull-down
            .write_pupdr7(0b00) // no pull-up, pull-down
    });
    gpio_b.gpio_afrl.store(|r| {
        r.write_afrl6(0b0100) // I2C1/I2C2/I2C3
            .write_afrl7(0b0100) // I2C1/I2C2/I2C3
    });
    gpio_b.gpio_moder.store(|r| {
        r.write_moder6(0b10) // alternate function
            .write_moder7(0b10) // alternate function
    });
    gpio_b.rcc_busenr_gpioen.clear_bit(); // IO port clock disable

    periph_dma1!(reg).rcc_busenr_dmaen.set_bit(); // DMA clock enable

    // Set up the driver.
    let i2c1 = I2CDrv::init(I2CSetup {
        i2c: periph_i2c1!(reg),
        i2c_ev: thr.i2c1_ev,
        i2c_er: thr.i2c1_er,
        i2c_freq: 42,           // APB1 clock = 42 MHz
        i2c_presc: 35,          // SCL clock = 400 kHz
        i2c_trise: 13,          // 285.7 ns
        i2c_mode: I2CMode::Fm2, // Fm mode t_low/t_high = 2
        dma_tx: periph_dma1_ch6!(reg),
        dma_tx_int: thr.dma1_ch6,
        dma_tx_ch: 1,    // I2C1_TX
        dma_tx_pl: 0b11, // very high
        dma_rx: periph_dma1_ch5!(reg),
        dma_rx_int: thr.dma1_ch5,
        dma_rx_ch: 1,    // I2C1_RX
        dma_rx_pl: 0b11, // very high
    });
}
```

Example of usage:

```rust
let buf = vec![0x92, 0, 0, 0].into_boxed_slice();
let buf = i2c1
    .master(buf) // create a master session backed by the given buffer
    .write(0x39, ..1) // write the first byte from the buffer to address `0x39`
    .await
    .read(0x39, ..) // read 4 bytes into the buffer from address `0x39`
    .await
    .stop(); // release the bus and get the buffer back
println!("{:?}", buf);
```

## References

* [I²C-bus Specification, Version 6.0, 4th of April
2014](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

[Drone OS]: https://www.drone-os.com/

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
