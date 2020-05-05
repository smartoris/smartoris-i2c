use drone_cortexm::reg::prelude::*;
use drone_stm32_map::periph::dma::ch::{DmaChMap, DmaChPeriph};

#[allow(dead_code)]
pub(crate) struct DmaChDiverged<T: DmaChMap> {
    pub(crate) dma_ccr: T::UDmaCcr,
    pub(crate) dma_cfcr: T::UDmaCfcr,
    pub(crate) dma_cm0ar: T::UDmaCm0Ar,
    pub(crate) dma_cm1ar: T::UDmaCm1Ar,
    pub(crate) dma_cndtr: T::UDmaCndtr,
    pub(crate) dma_cpar: T::UDmaCpar,
    pub(crate) dma_ifcr_cdmeif: T::CDmaIfcrCdmeif,
    pub(crate) dma_ifcr_cfeif: T::CDmaIfcrCfeif,
    pub(crate) dma_ifcr_chtif: T::CDmaIfcrChtif,
    pub(crate) dma_ifcr_ctcif: T::CDmaIfcrCtcif,
    pub(crate) dma_ifcr_cteif: T::CDmaIfcrCteif,
    pub(crate) dma_isr_dmeif: T::CDmaIsrDmeif,
    pub(crate) dma_isr_feif: T::CDmaIsrFeif,
    pub(crate) dma_isr_htif: T::CDmaIsrHtif,
    pub(crate) dma_isr_tcif: T::CDmaIsrTcif,
    pub(crate) dma_isr_teif: T::CDmaIsrTeif,
}

impl<T: DmaChMap> From<DmaChPeriph<T>> for DmaChDiverged<T> {
    fn from(periph: DmaChPeriph<T>) -> Self {
        let DmaChPeriph {
            dma_ccr,
            dma_cfcr,
            dma_cm0ar,
            dma_cm1ar,
            dma_cndtr,
            dma_cpar,
            dma_ifcr_cdmeif,
            dma_ifcr_cfeif,
            dma_ifcr_chtif,
            dma_ifcr_ctcif,
            dma_ifcr_cteif,
            dma_isr_dmeif,
            dma_isr_feif,
            dma_isr_htif,
            dma_isr_tcif,
            dma_isr_teif,
        } = periph;
        Self {
            dma_ccr: dma_ccr.into_unsync(),
            dma_cfcr: dma_cfcr.into_unsync(),
            dma_cm0ar: dma_cm0ar.into_unsync(),
            dma_cm1ar: dma_cm1ar.into_unsync(),
            dma_cndtr: dma_cndtr.into_unsync(),
            dma_cpar: dma_cpar.into_unsync(),
            dma_ifcr_cdmeif: dma_ifcr_cdmeif.into_copy(),
            dma_ifcr_cfeif: dma_ifcr_cfeif.into_copy(),
            dma_ifcr_chtif: dma_ifcr_chtif.into_copy(),
            dma_ifcr_ctcif: dma_ifcr_ctcif.into_copy(),
            dma_ifcr_cteif: dma_ifcr_cteif.into_copy(),
            dma_isr_dmeif: dma_isr_dmeif.into_copy(),
            dma_isr_feif: dma_isr_feif.into_copy(),
            dma_isr_htif: dma_isr_htif.into_copy(),
            dma_isr_tcif: dma_isr_tcif.into_copy(),
            dma_isr_teif: dma_isr_teif.into_copy(),
        }
    }
}
