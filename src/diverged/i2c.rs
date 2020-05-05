use drone_cortexm::reg::prelude::*;
use drone_stm32_map::periph::i2c::{I2CMap, I2CPeriph};

#[allow(dead_code)]
pub(crate) struct I2CDiverged<T: I2CMap> {
    pub(crate) rcc_busenr_i2cen: T::SRccBusenrI2Cen,
    pub(crate) rcc_busrstr_i2crst: T::SRccBusrstrI2Crst,
    pub(crate) rcc_bussmenr_i2csmen: T::SRccBussmenrI2Csmen,
    pub(crate) i2c_cr1: T::CI2CCr1,
    pub(crate) i2c_cr2: T::CI2CCr2,
    pub(crate) i2c_oar1: T::UI2COar1,
    pub(crate) i2c_oar2: T::UI2COar2,
    pub(crate) i2c_dr: T::CI2CDr,
    pub(crate) i2c_sr1: T::CI2CSr1,
    pub(crate) i2c_sr2: T::CI2CSr2,
    pub(crate) i2c_ccr: T::UI2CCcr,
    pub(crate) i2c_trise: T::UI2CTrise,
    pub(crate) i2c_fltr: T::UI2CFltr,
}

impl<T: I2CMap> From<I2CPeriph<T>> for I2CDiverged<T> {
    fn from(periph: I2CPeriph<T>) -> Self {
        let I2CPeriph {
            rcc_busenr_i2cen,
            rcc_busrstr_i2crst,
            rcc_bussmenr_i2csmen,
            i2c_cr1,
            i2c_cr2,
            i2c_oar1,
            i2c_oar2,
            i2c_dr,
            i2c_sr1,
            i2c_sr2,
            i2c_ccr,
            i2c_trise,
            i2c_fltr,
        } = periph;
        Self {
            rcc_busenr_i2cen,
            rcc_busrstr_i2crst,
            rcc_bussmenr_i2csmen,
            i2c_cr1: i2c_cr1.into_copy(),
            i2c_cr2: i2c_cr2.into_copy(),
            i2c_oar1: i2c_oar1.into_unsync(),
            i2c_oar2: i2c_oar2.into_unsync(),
            i2c_dr: i2c_dr.into_copy(),
            i2c_sr1: i2c_sr1.into_copy(),
            i2c_sr2: i2c_sr2.into_copy(),
            i2c_ccr: i2c_ccr.into_unsync(),
            i2c_trise: i2c_trise.into_unsync(),
            i2c_fltr: i2c_fltr.into_unsync(),
        }
    }
}
