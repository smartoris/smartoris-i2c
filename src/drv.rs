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
    /// DMA Tx channel peripheral.
    pub dma_tx: DmaChPeriph<DmaTx>,
    /// DMA Tx channel interrupt.
    pub dma_tx_int: DmaTxInt,
    /// DMA Tx channel number.
    pub dma_tx_ch: u32,
    /// DMA Tx channel priority level.
    pub dma_tx_pl: u32,
    /// DMA Rx channel peripheral.
    pub dma_rx: DmaChPeriph<DmaRx>,
    /// DMA Rx channel interrupt.
    pub dma_rx_int: DmaRxInt,
    /// DMA Rx channel number.
    pub dma_rx_ch: u32,
    /// DMA Rx channel priority level.
    pub dma_rx_pl: u32,
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
        drv.init_i2c();
        drv.init_dma_tx(dma_tx_ch, dma_tx_pl);
        drv.init_dma_rx(dma_rx_ch, dma_rx_pl);
        drv
    }

    /// Starts a new master session in transmitter mode.
    pub fn master_write<'a: 'b, 'b>(
        &'a mut self,
        addr: u8,
        buf_tx: &'b [u8],
    ) -> impl Future<Output = I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>> + 'b
    {
        I2CMaster::new(self).send_write(addr, buf_tx, false)
    }

    /// Starts a new master session in receiver mode.
    pub fn master_read<'a: 'b, 'b>(
        &'a mut self,
        addr: u8,
        buf_rx: &'b mut [u8],
    ) -> impl Future<Output = I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>> + 'b
    {
        I2CMaster::new(self).send_read(addr, buf_rx, false)
    }

    pub(crate) fn dma_tx(&mut self, buf_tx: &[u8]) {
        self.dma_tx.dma_cm0ar.store_reg(|r, v| {
            r.m0a().write(v, buf_tx.as_ptr() as u32); // memory address
        });
        self.dma_tx.dma_cndtr.store_reg(|r, v| {
            r.ndt().write(v, buf_tx.len() as u32); // number of data items to transfer
        });
        self.dma_tx.dma_ifcr_ctcif.set_bit(); // clear transfer complete interrupt flag
        self.dma_tx.dma_ccr.modify_reg(|r, v| r.en().set(v)); // stream enable
    }

    pub(crate) fn dma_rx(&mut self, buf_rx: &mut [u8]) -> impl Future<Output = ()> {
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

    pub(crate) fn start(
        &mut self,
        addr: u8,
        repeated: bool,
        ack: bool,
    ) -> impl Future<Output = ()> {
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

    pub(crate) fn stop(&mut self) {
        self.i2c.i2c_cr1.stop().set_bit(); // stop generation
    }

    pub(crate) fn wait_unfinished_stop(&mut self) {
        while self.i2c.i2c_cr1.stop().read_bit() {} // stop generation
    }

    fn init_i2c(&mut self) {
        self.i2c.rcc_busenr_i2cen.set_bit(); // I2C clock enable
        self.i2c.i2c_cr2.store_reg(|r, v| {
            r.last().set(v); // next DMA EOT is the last transfer
            r.dmaen().set(v); // DMA requests enable
            r.iterren().set(v); // error interrupt enable
            r.freq().write(v, 0b10_1010); // peripheral clock frequency 42 MHz
        });
        self.i2c.i2c_ccr.store_reg(|r, v| {
            r.f_s().set(v); // Fm mode I2C
            r.duty().clear(v); // Fm mode t_low/t_high = 2
            r.ccr().write(v, 35); // SCL clock in master mode - 400 kHz
        });
        self.i2c.i2c_trise.store_reg(|r, v| {
            r.trise().write(v, 13); // maximum rise time in Fm/Sm mode - 285.7 ns
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
