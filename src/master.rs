use crate::I2CDrv;
use drone_cortexm::thr::prelude::*;
use drone_stm32_map::periph::{dma::ch::DmaChMap, i2c::I2CMap};
use futures::prelude::*;

/// I²C master session.
///
/// Dropping a master session automatically generates STOP signal.
pub struct I2CMaster<
    'a,
    I2C: I2CMap,
    I2CEv: IntToken,
    I2CEr: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> {
    drv: &'a mut I2CDrv<I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>,
}

impl<
    'a,
    I2C: I2CMap,
    I2CEv: IntToken,
    I2CEr: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>
{
    /// Continues this master session in transmitter mode.
    pub fn write<'b>(
        self,
        addr: u8,
        buf_tx: &'b [u8],
    ) -> impl Future<Output = I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>> + 'b
    where
        'a: 'b,
    {
        self.send_write(addr, buf_tx, true)
    }

    /// Continues this master session in receiver mode.
    pub fn read<'b>(
        self,
        addr: u8,
        buf_rx: &'b mut [u8],
    ) -> impl Future<Output = I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>> + 'b
    where
        'a: 'b,
    {
        self.send_read(addr, buf_rx, true)
    }

    pub(crate) fn new(
        drv: &'a mut I2CDrv<I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>,
    ) -> Self {
        drv.wait_unfinished_stop();
        Self { drv }
    }

    pub(crate) async fn send_write(
        self,
        addr: u8,
        buf_tx: &[u8],
        repeated: bool,
    ) -> I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt> {
        self.drv.dma_tx(buf_tx);
        self.drv.start(addr << 1, repeated, false).await;
        self
    }

    pub(crate) async fn send_read(
        self,
        addr: u8,
        buf_rx: &mut [u8],
        repeated: bool,
    ) -> I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt> {
        let dma_rx = self.drv.dma_rx(buf_rx);
        self.drv.start(addr << 1 | 1, repeated, buf_rx.len() > 1).await;
        dma_rx.await;
        self
    }
}

impl<
    I2C: I2CMap,
    I2CEv: IntToken,
    I2CEr: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> Drop for I2CMaster<'_, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>
{
    fn drop(&mut self) {
        self.drv.stop();
    }
}
