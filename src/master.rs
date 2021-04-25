use crate::I2CDrv;
use core::{mem::ManuallyDrop, slice::SliceIndex};
use drone_cortexm::thr::prelude::*;
use drone_stm32_map::periph::{dma::ch::DmaChMap, i2c::I2CMap};

/// I²C master session.
///
/// The session object takes ownership of the provided buffer, which is returned
/// by [`I2CMaster::stop`] method. If the `stop` method is not called, the
/// buffer will be leaked.
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
    buf: ManuallyDrop<Box<[u8]>>,
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
    pub(crate) fn new(
        drv: &'a mut I2CDrv<I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt>,
        buf: Box<[u8]>,
    ) -> Self {
        Self { drv, buf: ManuallyDrop::new(buf) }
    }

    /// Sends the Start signal for the address `addr`, and writes the data from
    /// the session buffer slice of the range `index` to the slave.
    pub async fn write<I: SliceIndex<[u8], Output = [u8]>>(
        self,
        addr: u8,
        index: I,
    ) -> I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt> {
        unsafe { self.drv.write(addr, &self.buf[index]).await };
        self
    }

    /// Sends the Start signal for the address `addr`, and reads the data from
    /// the slave into the session buffer slice of the range `index`.
    pub async fn read<I: SliceIndex<[u8], Output = [u8]>>(
        mut self,
        addr: u8,
        index: I,
    ) -> I2CMaster<'a, I2C, I2CEv, I2CEr, DmaTx, DmaTxInt, DmaRx, DmaRxInt> {
        unsafe { self.drv.read(addr, &mut self.buf[index]).await };
        self
    }

    /// Returns a reference to the session buffer.
    #[inline]
    #[must_use]
    pub fn buf(&self) -> &[u8] {
        &self.buf
    }

    /// Returns a mutable reference to the session buffer.
    #[inline]
    #[must_use]
    pub fn buf_mut(&mut self) -> &mut Box<[u8]> {
        &mut self.buf
    }

    /// Sends the Stop signal and returns the session buffer.
    #[must_use]
    pub fn stop(self) -> Box<[u8]> {
        let Self { drv, buf } = self;
        drv.stop();
        ManuallyDrop::into_inner(buf)
    }
}
