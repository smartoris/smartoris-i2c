use crate::{
    diverged::{DmaChDiverged, I2CDiverged},
    I2CMaster,
};
use drone_cortexm::{fib, reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
    dma::ch::{traits::*, DmaChMap, DmaChPeriph},
    i2c::{traits::*, I2CMap, I2CPeriph},
};
use futures::prelude::*;

/// I²C setup.
pub struct I2CSetup<
    I2C: I2CMap,
    I2CEv: IntToken,
    I2CEr: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> {
    /// I²C peripheral.
    pub i2c: I2CPeriph<I2C>,
    /// I²C event interrupt.
    pub i2c_ev: I2CEv,
    /// I²C error interrupt.
    pub i2c_er: I2CEr,
    /// I²C peripheral clock frequency.
    ///
    /// The value should be set with the APB bus frequency this I2C peripheral
    /// connected to in MHz.
    ///
    /// This will be written to I2C_CR2.FREQ field. See the reference manual for
    /// details.
    pub i2c_freq: u32,
    /// I²C clock prescaler.
    ///
    /// SCL clock = [`i2c_freq`](I2CSetup::i2c_freq) × 1_000_000 ÷
    /// [`i2c_presc`](I2CSetup::i2c_presc) ÷ `mode`
    ///
    /// Where `mode` is the sum of divisor and denominator of
    /// t<sub>low</sub>/t<sub>high</sub> ratio selected by
    /// [`i2c_mode`](I2CSetup::i2c_mode).
    ///
    /// This will be written to I2C_CCR.CCR field. See the reference manual for
    /// details.
    pub i2c_presc: u32,
    /// I²C maximum rise time.
    ///
    /// Calculated as follows: floor[[`i2c_freq`](I2CSetup::i2c_freq) ×
    /// 1_000_000 × t<sub>r</sub>] + 1
    ///
    /// Where t<sub>r</sub> is the maximum rise time of both SDA and SCL signals
    /// in seconds. Reference values: 1000 ns for Standard-mode, 300 ns for
    /// Fast-mode.
    ///
    /// This will be written to I2C_TRISE.TRISE field. See the reference manual
    /// for details.
    pub i2c_trise: u32,
    /// I²C bus mode.
    ///
    /// This will be written to I2C_CCR.F/S and I2C_CCR.DUTY fields. See the
    /// reference manual for details.
    pub i2c_mode: I2CMode,
    /// DMA Tx channel peripheral.
    pub dma_tx: DmaChPeriph<DmaTx>,
    /// DMA Tx channel interrupt.
    pub dma_tx_int: DmaTxInt,
    /// DMA Tx channel number.
    ///
    /// This will be written to DMA_SxCR.CHSEL field. See the reference manual
    /// for details.
    pub dma_tx_ch: u32,
    /// DMA Tx channel priority level.
    ///
    /// This will be written to DMA_SxCR.PL field. See the reference manual for
    /// details.
    pub dma_tx_pl: u32,
    /// DMA Rx channel peripheral.
    pub dma_rx: DmaChPeriph<DmaRx>,
    /// DMA Rx channel interrupt.
    pub dma_rx_int: DmaRxInt,
    /// DMA Rx channel number.
    ///
    /// This will be written to DMA_SxCR.CHSEL field. See the reference manual
    /// for details.
    pub dma_rx_ch: u32,
    /// DMA Rx channel priority level.
    ///
    /// This will be written to DMA_SxCR.PL field. See the reference manual for
    /// details.
    pub dma_rx_pl: u32,
}

/// I²C bus mode.
#[derive(Clone, Copy)]
pub enum I2CMode {
    /// Standard-mode with t<sub>low</sub>/t<sub>high</sub> = 1 duty cycle.
    Sm1,
    /// Fast-mode with t<sub>low</sub>/t<sub>high</sub> = 2 duty cycle.
    Fm2,
    /// Fast-mode with t<sub>low</sub>/t<sub>high</sub> = 16/9 duty cycle.
    Fm169,
}

/// I²C driver.
pub struct I2CDrv<
    I2C: I2CMap,
    I2CEv: IntToken,
    I2CEr: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> {
    i2c: I2CDiverged<I2C>,
    i2c_ev: I2CEv,
    i2c_er: I2CEr,
    dma_tx: DmaChDiverged<DmaTx>,
    dma_tx_int: DmaTxInt,
    dma_rx: DmaChDiverged<DmaRx>,
    dma_rx_int: DmaRxInt,
}

impl<
    I2C: I2CMap,
    I2CEv: IntToken,
    I2CEr: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> I2CDrv<I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>
{
    /// Sets up a new [`I2CDrv`] from `setup` values.
    #[must_use]
    pub fn init(setup: I2CSetup<I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>) -> Self {
        let I2CSetup {
            i2c,
            i2c_ev,
            i2c_er,
            i2c_freq,
            i2c_presc,
            i2c_trise,
            i2c_mode,
            dma_tx,
            dma_tx_int,
            dma_tx_ch,
            dma_tx_pl,
            dma_rx,
            dma_rx_int,
            dma_rx_ch,
            dma_rx_pl,
        } = setup;
        let mut drv = Self {
            i2c: i2c.into(),
            i2c_ev,
            i2c_er,
            dma_tx: dma_tx.into(),
            dma_tx_int,
            dma_rx: dma_rx.into(),
            dma_rx_int,
        };
        drv.init_i2c(i2c_freq, i2c_presc, i2c_trise, i2c_mode);
        drv.init_dma_tx(dma_tx_ch, dma_tx_pl);
        drv.init_dma_rx(dma_rx_ch, dma_rx_pl);
        drv
    }

    /// Creates a new master session.
    ///
    /// This method can block if previous Stop signal generation is not
    /// finished.
    ///
    /// The returned session object takes ownership of `buf`, which is returned
    /// by [`I2CMaster::stop`] method. If the `stop` method is not called, `buf`
    /// will be leaked.
    pub fn master(
        &mut self,
        buf: Box<[u8]>,
    ) -> I2CMaster<'_, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt> {
        while self.i2c.i2c_cr1.stop().read_bit() {} // stop generation
        I2CMaster::new(self, buf)
    }

    pub(crate) unsafe fn write(&mut self, addr: u8, buf_tx: &[u8]) -> impl Future<Output = ()> {
        self.dma_tx(buf_tx);
        self.start(addr << 1, false)
    }

    pub(crate) unsafe fn read(&mut self, addr: u8, buf_rx: &mut [u8]) -> impl Future<Output = ()> {
        let dma_rx = self.dma_rx(buf_rx);
        self.start(addr << 1 | 1, buf_rx.len() > 1).then(|()| dma_rx)
    }

    pub(crate) fn stop(&mut self) {
        self.i2c.i2c_cr1.stop().set_bit(); // stop generation
    }

    unsafe fn dma_tx(&mut self, buf_tx: &[u8]) {
        self.dma_tx.dma_cm0ar.store_reg(|r, v| {
            r.m0a().write(v, buf_tx.as_ptr() as u32); // memory address
        });
        self.dma_tx.dma_cndtr.store_reg(|r, v| {
            r.ndt().write(v, buf_tx.len() as u32); // number of data items to transfer
        });
        self.dma_tx.dma_ifcr_ctcif.set_bit(); // clear transfer complete interrupt flag
        self.dma_tx.dma_ccr.modify_reg(|r, v| r.en().set(v)); // stream enable
    }

    unsafe fn dma_rx(&mut self, buf_rx: &mut [u8]) -> impl Future<Output = ()> {
        let dma_ifcr_ctcif = self.dma_rx.dma_ifcr_ctcif;
        let dma_isr_dmeif = self.dma_rx.dma_isr_dmeif;
        let dma_isr_feif = self.dma_rx.dma_isr_feif;
        let dma_isr_tcif = self.dma_rx.dma_isr_tcif;
        let dma_isr_teif = self.dma_rx.dma_isr_teif;
        let future = self.dma_rx_int.add_future(fib::new_fn(move || {
            let val = dma_isr_tcif.load_val();
            handle_dma_err::<DmaRx>(&val, dma_isr_dmeif, dma_isr_feif, dma_isr_teif);
            if dma_isr_tcif.read(&val) {
                // transfer complete interrupt flag
                dma_ifcr_ctcif.set_bit(); // clear transfer complete interrupt flag
                fib::Complete(())
            } else {
                fib::Yielded(())
            }
        }));
        self.dma_rx.dma_cm0ar.store_reg(|r, v| {
            r.m0a().write(v, buf_rx.as_mut_ptr() as u32); // memory address
        });
        self.dma_rx.dma_cndtr.store_reg(|r, v| {
            r.ndt().write(v, buf_rx.len() as u32); // number of data items to transfer
        });
        self.dma_rx.dma_ccr.modify_reg(|r, v| r.en().set(v)); // stream enable
        future
    }

    fn start(&mut self, addr: u8, ack: bool) -> impl Future<Output = ()> {
        let i2c_cr1 = self.i2c.i2c_cr1;
        let i2c_cr2 = self.i2c.i2c_cr2;
        let i2c_sr1 = self.i2c.i2c_sr1;
        let i2c_sr2 = self.i2c.i2c_sr2;
        let i2c_dr = self.i2c.i2c_dr;
        let set_start = move || {
            i2c_cr1.modify_reg(|r, v| {
                if ack {
                    r.ack().set(v); // acknowledge enable
                } else {
                    r.ack().clear(v); // acknowledge disable
                }
                r.start().set(v); // start generation
            });
        };
        let repeated = self.i2c.i2c_sr2.msl().read_bit();
        let future = self.i2c_ev.add_future(fib::new_fn(move || {
            let sr1_val = i2c_sr1.load_val();
            if i2c_sr1.sb().read(&sr1_val) {
                // start condition generated
                i2c_dr.store_reg(|r, v| r.dr().write(v, u32::from(addr))); // 8-bit data register
                fib::Yielded(())
            } else if i2c_sr1.addr().read(&sr1_val) {
                let sr2_val = i2c_sr2.load_val();
                // end of address transmission
                if i2c_sr2.tra().read(&sr2_val) {
                    // transmitter
                    fib::Yielded(())
                } else {
                    fib::Complete(())
                }
            } else if i2c_sr1.btf().read(&sr1_val) {
                // data byte transfer succeeded
                if repeated {
                    set_start();
                    fib::Yielded(())
                } else {
                    i2c_cr2.itevten().clear_bit(); // event interrupt disable
                    fib::Complete(())
                }
            } else {
                fib::Yielded(())
            }
        }));
        self.i2c.i2c_cr2.itevten().set_bit(); // event interrupt enable
        if !repeated {
            set_start();
        }
        future
    }

    fn init_i2c(&mut self, i2c_freq: u32, i2c_presc: u32, i2c_trise: u32, i2c_mode: I2CMode) {
        self.i2c.rcc_busenr_i2cen.set_bit(); // I2C clock enable
        self.i2c.i2c_cr2.store_reg(|r, v| {
            r.last().set(v); // next DMA EOT is the last transfer
            r.dmaen().set(v); // DMA requests enable
            r.iterren().set(v); // error interrupt enable
            r.freq().write(v, i2c_freq); // peripheral clock frequency
        });
        self.i2c.i2c_ccr.store_reg(|r, v| {
            match i2c_mode {
                I2CMode::Sm1 => {
                    r.f_s().clear(v); // Sm mode I2C
                }
                I2CMode::Fm2 => {
                    r.f_s().set(v); // Fm mode I2C
                    r.duty().clear(v); // Fm mode t_low/t_high = 2
                }
                I2CMode::Fm169 => {
                    r.f_s().set(v); // Fm mode I2C
                    r.duty().set(v); // Fm mode t_low/t_high = 16/9
                }
            }
            r.ccr().write(v, i2c_presc); // SCL clock in master mode
        });
        self.i2c.i2c_trise.store_reg(|r, v| {
            r.trise().write(v, i2c_trise); // maximum rise time in Fm/Sm mode
        });
        self.i2c.i2c_cr1.store_reg(|r, v| r.pe().set(v)); // peripheral enable
        let i2c_sr1 = self.i2c.i2c_sr1;
        self.i2c_er.add_fn(move || {
            let val = i2c_sr1.load_val();
            handle_i2c_err::<I2C>(&val, i2c_sr1);
            fib::Yielded::<(), !>(())
        });
    }

    fn init_dma_tx(&mut self, channel: u32, priority: u32) {
        let address = self.i2c.i2c_dr.as_mut_ptr(); // 8-bit data register
        self.dma_tx.dma_cpar.store_reg(|r, v| {
            r.pa().write(v, address as u32); // peripheral address
        });
        self.dma_tx.dma_ccr.store_reg(|r, v| {
            r.chsel().write(v, channel); // channel selection
            r.pl().write(v, priority); // priority level
            r.msize().write(v, 0b00); // byte (8-bit)
            r.psize().write(v, 0b00); // byte (8-bit)
            r.minc().set(v); // memory address pointer is incremented after each data transfer
            r.pinc().clear(v); // peripheral address pointer is fixed
            r.dir().write(v, 0b01); // memory-to-peripheral
            r.tcie().clear(v); // transfer complete interrupt disable
            r.teie().set(v); // transfer error interrupt enable
        });
        let dma_isr_dmeif = self.dma_tx.dma_isr_dmeif;
        let dma_isr_feif = self.dma_tx.dma_isr_feif;
        let dma_isr_teif = self.dma_tx.dma_isr_teif;
        self.dma_tx_int.add_fn(move || {
            let val = dma_isr_teif.load_val();
            handle_dma_err::<DmaTx>(&val, dma_isr_dmeif, dma_isr_feif, dma_isr_teif);
            fib::Yielded::<(), !>(())
        });
    }

    fn init_dma_rx(&mut self, channel: u32, priority: u32) {
        let address = self.i2c.i2c_dr.as_ptr(); // 8-bit data register
        self.dma_rx.dma_cpar.store_reg(|r, v| {
            r.pa().write(v, address as u32); // peripheral address
        });
        self.dma_rx.dma_ccr.store_reg(|r, v| {
            r.chsel().write(v, channel); // channel selection
            r.pl().write(v, priority); // priority level
            r.msize().write(v, 0b00); // byte (8-bit)
            r.psize().write(v, 0b00); // byte (8-bit)
            r.minc().set(v); // memory address pointer is incremented after each data transfer
            r.pinc().clear(v); // peripheral address pointer is fixed
            r.dir().write(v, 0b00); // peripheral-to-memory
            r.tcie().set(v); // transfer complete interrupt enable
            r.teie().set(v); // transfer error interrupt enable
        });
    }
}

fn handle_dma_err<T: DmaChMap>(
    val: &T::DmaIsrVal,
    dma_isr_dmeif: T::CDmaIsrDmeif,
    dma_isr_feif: T::CDmaIsrFeif,
    dma_isr_teif: T::CDmaIsrTeif,
) {
    if dma_isr_teif.read(&val) {
        panic!("Transfer error");
    }
    if dma_isr_dmeif.read(&val) {
        panic!("Direct mode error");
    }
    if dma_isr_feif.read(&val) {
        panic!("FIFO error");
    }
}

fn handle_i2c_err<T: I2CMap>(val: &T::I2CSr1Val, i2c_sr1: T::CI2CSr1) {
    if i2c_sr1.berr().read(&val) {
        panic!("Misplaced Start or Stop condition");
    }
    if i2c_sr1.arlo().read(&val) {
        panic!("Arbitration Lost detected");
    }
    if i2c_sr1.af().read(&val) {
        panic!("Acknowledge failure");
    }
    if i2c_sr1.ovr().read(&val) {
        panic!("Overrun or underrun");
    }
    if i2c_sr1.timeout().read(&val) {
        panic!("SCL remained LOW for 25 ms");
    }
}
