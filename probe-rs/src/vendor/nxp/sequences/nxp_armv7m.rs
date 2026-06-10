//! Sequences for NXP chips that use ARMv7-M cores.

use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

use crate::{
    architecture::arm::{
        ArmDebugInterface, ArmError, FullyQualifiedApAddress,
        armv7m::{Demcr, FpCtrl, FpRev2CompX},
        core::{
            armv7m::{Aircr, Dhcsr},
            registers::cortex_m::PC,
        },
        dp::DpAddress,
        memory::ArmMemoryInterface,
        sequences::{self, ArmDebugSequence, ArmDebugSequenceError},
    },
    core::MemoryMappedRegister,
};

/// Debug sequences for MIMXRT10xx MCUs.
///
/// In its current form, it uses no custom debug sequences. Instead, it ensures a reliable
/// reset sequence.
///
/// # On custom reset catch
///
/// Some tools use a custom reset catch that looks at the program image, finds the
/// reset vector, then places a breakpoint on that reset vector. This implementation
/// isn't doing that. That would be necessary if we don't control the kind of reset
/// that's happening. Since we're definitely using a SYSRESETREQ, we can rely on the
/// normal reset catch.
///
/// If the design changes such that the kind of reset isn't in our control, we'll
/// need to handle those cases.
#[derive(Debug)]
pub struct MIMXRT10xx {
    /// We're always catching the MCU at a watchpoint
    /// in the boot ROM. "Not catching" means that we'll
    /// release it after it hits the watchpoint.
    simulate_reset_catch: AtomicBool,
}

impl MIMXRT10xx {
    /// Create a sequence handle for the MIMXRT10xx.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self {
            simulate_reset_catch: AtomicBool::new(false),
        })
    }

    /// Halt or unhalt the core.
    fn halt(&self, probe: &mut dyn ArmMemoryInterface, halt: bool) -> Result<(), ArmError> {
        let mut dhcsr = Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?);
        dhcsr.set_c_halt(halt);
        dhcsr.set_c_debugen(true);
        dhcsr.enable_write();

        probe.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
        probe.flush()?;

        self.wait_for_halt(probe, halt)?;

        Ok(())
    }

    /// Use the boot fuse configuration for FlexRAM.
    ///
    /// If the user changed the FlexRAM configuration in software,
    /// this will undo that configuration, preferring the system's POR
    /// FlexRAM state.
    ///
    /// This function may change the processor's memory map, which may
    /// cause problems for any running firmware.  Halt the processor
    /// before calling this function.
    fn use_boot_fuses_for_flexram(
        &self,
        probe: &mut dyn ArmMemoryInterface,
    ) -> Result<(), ArmError> {
        const IOMUXC_GPR_GPR16: u64 = 0x400A_C040;
        const FLEXRAM_BANK_CFG_SEL_MASK: u32 = 1 << 2;
        let mut gpr16 = probe.read_word_32(IOMUXC_GPR_GPR16)?;
        gpr16 &= !FLEXRAM_BANK_CFG_SEL_MASK;
        probe.write_word_32(IOMUXC_GPR_GPR16, gpr16)?;
        probe.flush()?;
        Ok(())
    }

    /// Wait for the MCU to signal it's halted.
    fn wait_for_halt(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        halt: bool,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        let action = if halt { "halt" } else { "unhalt" };
        while Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?).s_halt() != halt {
            if start.elapsed() > Duration::from_millis(100) {
                tracing::debug!("Exceeded timeout while waiting for core to {action}");
                return Err(ArmError::Timeout);
            }
            thread::sleep(Duration::from_millis(1));
        }

        Ok(())
    }
}

impl ArmDebugSequence for MIMXRT10xx {
    fn reset_catch_set(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(true, Ordering::Relaxed);
        Ok(())
    }
    fn reset_catch_clear(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(false, Ordering::Relaxed);
        Ok(())
    }

    fn reset_system(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        _: crate::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        tracing::debug!("Halting MCU before changing FlexRAM layout");
        self.halt(interface, true)?;

        // OK to perform before the reset, since the configuration
        // persists beyond the reset.
        tracing::debug!("Setting FlexRAM layout");
        self.use_boot_fuses_for_flexram(interface)?;

        tracing::debug!("Enabling DWT to set a watchpoint");
        let mut demcr = Demcr(interface.read_word_32(Demcr::get_mmio_address())?);
        let trcena = demcr.trcena();
        demcr.set_trcena(true);
        interface.write_word_32(Demcr::get_mmio_address(), demcr.0)?;

        // Catching the MCU here helps RAM loading reliability.
        // The boot ROM sets up just enough of the MCU for us,
        // and we catch it as it tries to figure out the boot
        // configuration. If we're not changing execution context
        // after the fact, this is a no-op.
        tracing::debug!("Installing watchpoint to catch boot ROM SRC_SBMR1 read");
        const DWT_COMP0: u64 = 0xE000_1020;
        const DWT_MASK0: u64 = 0xE000_1024;
        const DWT_FUNCTION0: u64 = 0xE000_1028;
        const DWT_FUNCTION_DATAVSIZE_WORD: u32 = 0b10 << 10;
        const DWT_FUNCTION_DEBUG_DATA_RW: u32 = 0b0111;
        const SRC_SBMR1: u32 = 0x400F_8004;
        interface.write_word_32(DWT_COMP0, SRC_SBMR1)?;
        interface.write_word_32(DWT_MASK0, 0)?;
        interface.write_word_32(
            DWT_FUNCTION0,
            DWT_FUNCTION_DATAVSIZE_WORD | DWT_FUNCTION_DEBUG_DATA_RW,
        )?;

        interface.flush()?;

        // Do the usual reset. The watchpoint persists across the
        // reset.
        tracing::debug!("Performing the standard Cortex-M system reset");
        sequences::cortex_m_reset_system(interface)?;

        // Wait for that watchpoint to hit.
        tracing::debug!("Waiting for watchpoint to hit");
        self.wait_for_halt(interface, true)?;

        // Clean up after ourselves.
        tracing::debug!("Cleaning up watchpoints");
        interface.write_word_32(DWT_COMP0, 0)?;
        interface.write_word_32(DWT_FUNCTION0, 0)?;

        // Keep whatever tracing selection the system
        // previously had.
        let mut demcr = Demcr(interface.read_word_32(Demcr::get_mmio_address())?);
        demcr.set_trcena(trcena);
        interface.write_word_32(Demcr::get_mmio_address(), demcr.0)?;

        interface.flush()?;

        // Unhalt if we're not catching the reset.
        if !self.simulate_reset_catch.load(Ordering::Relaxed) {
            self.halt(interface, false)?;
        }

        Ok(())
    }
}

/// Backwards-compatible debug sequence for MIMXRT1170 MCUs.
#[deprecated(note = "Prefer MIMXRT11xx, which supports 1170 and 1160 targets")]
pub type MIMXRT117x = MIMXRT11xx;

/// Debug sequences for MIMXRT1170 / MIMXRT1160 MCUs.
///
/// Currently only supports the Cortex M7. In fact, if you try to interact with the Cortex M4,
/// you'll have a bad time: its access port doesn't appear until it's released from reset!
/// For the time being, you can only do things through the CM7.
#[derive(Debug)]
pub struct MIMXRT11xx {
    /// Given the reset we're performing, we won't be able to perform
    /// a normal vector catch. (The boot ROM doesn't care about us.)
    /// We'll simulate that behavior for the user.
    simulate_reset_catch: AtomicBool,
}

impl MIMXRT11xx {
    /// System reset controller base address.
    const SRC: u64 = 0x40C0_4000;
    /// SRC reset mode register.
    const SRC_SRMR: u64 = Self::SRC + 4;

    fn new() -> Self {
        Self {
            simulate_reset_catch: AtomicBool::new(false),
        }
    }

    /// Create a sequence handle for the MIMXRT1170 / MIMXRT1160.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self::new())
    }

    /// To ensure we affect a system reset, clear the mask that would prevent
    /// a response to the CM7's SYSRESETREQ.
    fn clear_src_srmr_mask(&self, probe: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
        let mut srmr = probe.read_word_32(Self::SRC_SRMR)?;
        tracing::debug!("SRC_SRMR: {srmr:#010X}. Clearing the M7REQ_RESET_MODE mask...");
        srmr &= !(0b11 << 12);
        probe.write_word_32(Self::SRC_SRMR, srmr)?;
        probe.flush()?;
        Ok(())
    }

    /// Halt or unhalt the core.
    fn halt(&self, probe: &mut dyn ArmMemoryInterface, halt: bool) -> Result<(), ArmError> {
        let mut dhcsr = Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?);
        dhcsr.set_c_halt(halt);
        dhcsr.set_c_debugen(true);
        dhcsr.enable_write();

        probe.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
        probe.flush()?;

        let start = Instant::now();
        let action = if halt { "halt" } else { "unhalt" };

        while Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?).s_halt() != halt {
            if start.elapsed() > Duration::from_millis(100) {
                tracing::debug!("Exceeded timeout while waiting for the core to {action}");
                return Err(ArmError::Timeout);
            }
            thread::sleep(Duration::from_millis(1));
        }

        Ok(())
    }

    /// Poll the AP's status until it can accept transfers.
    fn wait_for_enable(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        let mut errors = 0usize;
        let mut disables = 0usize;

        loop {
            match probe.generic_status() {
                Ok(csw) if csw.DeviceEn => {
                    tracing::debug!(
                        "Device enabled after {}ms with {errors} errors and {disables} invalid statuses",
                        start.elapsed().as_millis()
                    );
                    return Ok(());
                }
                Ok(_) => disables += 1,
                Err(_) => errors += 1,
            }

            if start.elapsed() > timeout {
                tracing::debug!(
                    "Exceeded {}ms timeout while waiting for enable with {errors} errors and {disables} invalid statuses",
                    timeout.as_millis()
                );
                return Err(ArmError::Timeout);
            }

            thread::sleep(Duration::from_millis(1));
        }
    }

    /// Assumes that the core is halted.
    fn read_core_reg(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        reg: crate::core::registers::CoreRegister,
    ) -> Result<u32, ArmError> {
        crate::architecture::arm::core::cortex_m::read_core_reg(probe, reg.into())
    }

    /// Assumes that the core is halted.
    fn write_core_reg(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        reg: crate::core::registers::CoreRegister,
        value: u32,
    ) -> Result<(), ArmError> {
        crate::architecture::arm::core::cortex_m::write_core_reg(probe, reg.into(), value)?;
        probe.flush()?;
        Ok(())
    }

    /// Ensure that the program counter's contents match `expected`.
    ///
    /// Assumes that the core is halted.
    fn check_pc(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        expected: u32,
    ) -> Result<(), ArmDebugSequenceError> {
        let pc = self
            .read_core_reg(probe, PC)
            .map_err(|err| ArmDebugSequenceError::SequenceSpecific(err.into()))?;
        if pc != expected {
            let err = format!(
                "The Cortex M7 should be at address {expected:#010X} but it's at {pc:#010X}"
            );
            return Err(ArmDebugSequenceError::SequenceSpecific(err.into()));
        }
        Ok(())
    }

    /// When the boot ROM detects a reset due to SYSRESETREQ, it spins
    /// at this location. It appears that this spinning location is after
    /// the boot ROM has done its useful work (like turn on clocks, prepare
    /// FlexSPI configuration blocks), but before it jumps into the program.
    const BOOT_ROM_SPIN_ADDRESS: u32 = 0x00223104;

    /// Returns the reset handler address contained in the NVM program image.
    ///
    /// We might not find that reset handler. In that case, return `None`.
    fn find_flexspi_image_reset_handler(
        &self,
        probe: &mut dyn ArmMemoryInterface,
    ) -> Result<Option<u32>, ArmError> {
        /// Assumed by today's in-tree target definition.
        const FLEXSPI1: u64 = 0x30000000;
        /// A well-formed FlexSPI program has its image vector table at this offset in flash.
        const IVT: u64 = FLEXSPI1 + 0x1000;
        tracing::debug!("Assuming that your CM7's program is in FlexSPI1 at {FLEXSPI1:#010X}");

        // Make sure the IVT header looks reasonable.
        //
        // See 10.7.1.1 Image vector table structure in the 1170 RM (Rev 2).
        // If it doesn't look reasonable, we assume that FlexSPI is inaccessible.
        let ivt_header = probe.read_word_32(IVT)?;
        tracing::debug!("IVT Header: {ivt_header:#010X}");

        if ivt_header & 0xFF != 0xD1 {
            tracing::debug!("IVT tag is incorrect! Expected 0xD1 in {ivt_header:#010X}");
            return Ok(None);
        }

        if (ivt_header >> 8) & 0xFFFF != 0x2000 {
            tracing::debug!("IVT length is incorrect! {ivt_header:#010X}");
            return Ok(None);
        }

        let ivt_version = ivt_header >> 24;
        if !(0x40..=0x45).contains(&ivt_version) {
            tracing::debug!("IVT version is invalid! {ivt_header:#010X}");
            return Ok(None);
        }

        // IVT versions 4.0 (0x40) are documented as containing the "entry point."
        // But in practice, this seems to be the pointer to the vector table. IVT
        // versions 4.1 and 4.3 (0x41, 0x43) appear to truly use the reset handler, not
        // the vector table. I can't find any documentation on this, so this comes from
        // some local testing. We assume that 4.0 is the outlier, and that all versions
        // above it use the same approach.
        let reset_handler = if ivt_version == 0x40 {
            // The address of the vector table is immediately behind the IVT header.
            let vector_table = probe.read_word_32(IVT + 4)?;
            tracing::debug!("Vector table address: {vector_table:#010X}");

            // The vector table starts with the stack pointer. Then the
            // reset handle is immediately behind that.
            probe.read_word_32(u64::from(vector_table) + 4u64)?
        } else {
            // The reset handler immediately follows the IVT header.
            probe.read_word_32(IVT + 4)?
        };

        tracing::debug!("Reset handler: {reset_handler:#010X}");
        if reset_handler & 1 == 0 {
            tracing::debug!(
                "Is your reset handler actually a function address? Where's its thumb bit?"
            );
            return Ok(None);
        }

        Ok(Some(reset_handler))
    }

    /// See documentation for [`MIMXRT10xx::use_boot_fuses_for_flexram`].
    fn use_boot_fuses_for_flexram(
        &self,
        probe: &mut dyn ArmMemoryInterface,
    ) -> Result<(), ArmError> {
        const IOMUXC_GPR_GPR16: u64 = 0x400E_4040;
        const FLEXRAM_BANK_CFG_SEL_MASK: u32 = 1 << 2;
        let mut gpr16 = probe.read_word_32(IOMUXC_GPR_GPR16)?;
        gpr16 &= !FLEXRAM_BANK_CFG_SEL_MASK;
        probe.write_word_32(IOMUXC_GPR_GPR16, gpr16)?;
        probe.flush()?;
        Ok(())
    }
}

impl ArmDebugSequence for MIMXRT11xx {
    fn reset_catch_set(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(true, Ordering::Relaxed);
        Ok(())
    }
    fn reset_catch_clear(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(false, Ordering::Relaxed);
        Ok(())
    }

    fn reset_system(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        core_type: probe_rs_target::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        // OK to perform before the reset, since the configuration
        // persists beyond the reset.
        self.halt(probe, true)?;
        self.use_boot_fuses_for_flexram(probe)?;

        // Cache debug system state that may be lost across the reset.
        let debug_cache = DebugCache::from_target(probe)?;

        // Make sure that the CM7's SYSRESETREQ isn't ignored by the system
        // reset controller.
        self.clear_src_srmr_mask(probe)?;

        // Affect a SYSRESETREQ through the CM7 to reset the entire system.
        //
        // For more information on the SYSRESETREQ response, consult the system
        // reset controller (SRC) section of the reference manual. This is a
        // convenient way to perform a whole-system reset.
        //
        // Another approach to perform this reset: iterate through all SRC slice controls,
        // and manually reset them. That should be close to SYSRESETREQ. However, it seems
        // that there are no slice controls for CM4MEM (LMEM) and CM7MEM (FlexRAM)
        // so you might not be able to affect a reset on those two domains.
        //
        // If you scan through the slices, you'll notice that the M7CORE and M7DEBUG are
        // different slices. You'll think "I can perform a reset through the SRC that hits
        // all slices except the M*DEBUG slices. This would preserve debugging and I won't
        // have to re-initialize the debug port!" I could not get that to work; if I did a
        // reset through SRC_CTRL_M7CORE, I found that I still needed to re-initialize the
        // debug port after the reset. Maybe I did something wrong.
        //
        // We're about to lose the debug port! We're ignoring missed or incorrect responses.
        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);
        probe
            .write_word_32(Aircr::get_mmio_address(), aircr.into())
            .ok();
        probe.flush().ok();

        // If all goes well, we lost the debug port. Thanks, boot ROM. Let's bring it back.
        //
        // The ARM communication interface knows how to re-initialize the debug port.
        // Re-initializing the core(s) is on us.
        let ap = probe.fully_qualified_address();
        let interface = probe.get_arm_debug_interface()?;
        interface.reinitialize()?;

        assert!(debug_base.is_none());
        self.debug_core_start(interface, &ap, core_type, None, None)?;

        // Are we back?
        self.wait_for_enable(probe, Duration::from_millis(300))?;

        // We're back. Halt the core so we can establish the reset context.
        self.halt(probe, true)?;

        // When we reset into the boot ROM, it checks why we reset. If the boot ROM observes that
        // we reset due to SYSRESETREQ, it spins at a known address. Are we spinning there?
        self.check_pc(probe, Self::BOOT_ROM_SPIN_ADDRESS)?;

        // Why does the boot ROM spin? It wants us to set up the reset context! (And it wanted
        // to give us a chance to re-establish debugging after it took it away from us.)
        //
        // We assume that the user wants reset into the program they store within FlexSPI. We
        // emulate the behaviors of the boot ROM here: find the reset handler, and prepare the
        // CM7 to run that reset handler. It's convenient that the boot ROM prepares the FlexSPI
        // controller...
        //
        // But that's not always true: if you change your boot fuses, your board's boot pins, etc.
        // then the boot ROM respects that configuration. It might not initialize the FlexSPI
        // controller, and we won't be able to find the reset handler. We're not sure what to do
        // here, so we'll keep the CM7 in the boot ROM.
        //
        // (A generous tool might inspect the boot fuses to figure out what the next step would
        // be. Maybe it could invoke more boot ROM APIs to put us into the next stage. Sorry,
        // we're not yet a generous tool.)
        if let Some(pc) = self.find_flexspi_image_reset_handler(probe)? {
            self.write_core_reg(probe, PC, pc)?
        } else {
            tracing::warn!(
                "Could not find a valid reset handler in FlexSPI! Keeping the CM7 in the boot ROM."
            );
        }

        debug_cache.restore(probe)?;

        // We're halted in order to establish the reset context. Did the user want us to stay halted?
        if !self.simulate_reset_catch.load(Ordering::Relaxed) {
            self.halt(probe, false)?;
        }

        Ok(())
    }
}

/// Cache the debug state of the MCU.
///
/// Some targets will lose this state once they execute a system reset. For
/// targets that know this will happen, we can restore the context after
/// the reset occurs.
///
/// There's probably more we could cache, but this is a good enough starting
/// point for 1170 testing.
///
/// The FPB assumes the v2 architecture revision, and it only cares about
/// control and comparator registers. (No caching of any CoreSight IDs.)
/// A portable implementation may need to specialize this for the FPB revision
/// of the chip.
struct DebugCache {
    fp_ctrl: FpCtrl,
    fp_comps: Vec<FpRev2CompX>,
}

impl DebugCache {
    /// Produce a debug cache from the target.
    fn from_target(probe: &mut dyn ArmMemoryInterface) -> Result<Self, ArmError> {
        let fp_ctrl = FpCtrl(probe.read_word_32(FpCtrl::get_mmio_address())?);

        Ok(Self {
            fp_ctrl,
            fp_comps: (0..fp_ctrl.num_code())
                .map(|base_address| -> Result<FpRev2CompX, ArmError> {
                    let address = FpRev2CompX::get_mmio_address_from_base(base_address as u64 * 4)?;
                    let fp_comp = probe.read_word_32(address)?;
                    Ok(FpRev2CompX(fp_comp))
                })
                .collect::<Result<Vec<_>, _>>()?,
        })
    }

    /// Put this cached debug state back into the target.
    fn restore(mut self, probe: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
        self.fp_ctrl.set_key(true);
        probe.write_word_32(FpCtrl::get_mmio_address(), self.fp_ctrl.into())?;

        for (base, fp_comp) in self.fp_comps.into_iter().enumerate() {
            probe.write_word_32(
                FpRev2CompX::get_mmio_address_from_base(base as u64 * 4)?,
                fp_comp.into(),
            )?;
        }

        Ok(())
    }
}


/// Debug sequences for NXP S32K3xx MCUs.
///
/// S32K3 parts expose a "Self-hosted Debug Access Port" (SDA-AP) that gates
/// debug visibility. On power-up the SDA-AP keeps debug disabled until we
/// write to its `DBGENCTRL` register — pyOCD does this as part of its
/// connect sequence:
///
/// ```python
/// self.dp.aps[SDA_AP_ID].write_reg(SDA_AP_DBGENCTRL_ADDR, SDA_AP_EN_ALL)
/// ```
///
/// We do the same in [`debug_device_unlock`], which the default
/// [`ArmDebugSequence`] dispatch runs after `debug_port_start` has powered
/// the DP up.
#[derive(Debug)]
pub struct S32K3xx {
    /// Matches the MIMXRT10xx pattern: we may be catching the core at a
    /// watchpoint placed by the reset sequence. "Not catching" means we
    /// release it after the watchpoint hits.
    simulate_reset_catch: AtomicBool,
}

impl S32K3xx {
    // AP IDs:
    // [1]   APB_AP
    // [4]   CM7_0_AHB_AP
    // [6]   MDM_AP
    // [7]   SDA_AP
    const _APD_AP_INDEX: u8 = 1;
    const _CM7_0_AHB_AP_INDEX: u8 = 4;
    const MDM_AP_INDEX: u8 = 6;
    const SDA_AP_INDEX: u8 = 7;

    /// Offset of the `MDMAPCTL` register inside the MDM-AP.
    const MDM_AP_CTL_ADDR: u64 = 0x04;

    // MDMAPCTL value sequence for `FunctionalReset`:
    //   step 1: assert RSTRELCM7/RSTRELTLn + CMnDBGREQ
    //   step 2: above + SYSFUNCRST  (this is the actual reset trigger)
    //   step 3: drop SYSFUNCRST, keep RSTRELCM7/RSTRELTLn + CMnDBGREQ
    //   step 4: drop CMnDBGREQ so the core leaves debug mode and runs
    const MDMAPCTL_RSTRELCM7_DBGREQ: u32 = 0x0040_0B00;
    const MDMAPCTL_RSTRELCM7_DBGREQ_SYSFUNCRST: u32 = 0x0040_0B20;
    const MDMAPCTL_RSTRELCM7: u32 = 0x0040_0000;

    /// Offset of the `DBGENCTRL` register inside the SDA-AP.
    const SDA_AP_DBGENCTRL_ADDR: u64 = 0x80;


    // SDA_AP DBGENCTRL bit fields:
    // [31:30]   reserved
    // [29]      Core Non-Invasive Debug Enable (CNIDEN)
    // [28]      Core Debug Enable (CDBGEN)
    // [27:8]    reserved
    // [7]       Global Secure Privileged Non-Invasive Debug Enable (GSPNIDEN)
    // [6]       Global Secure Privileged Debug Enable (GSPIDEN)
    // [5]       Global Non-Invasive Debug Enable (GNIDEN)
    // [4]       Global Debug Enable (GDBGEN)
    // [3:0]     reserved
    const SDA_AP_CNIDEN: u32 = 1 << 29;
    const SDA_AP_CDBGEN: u32 = 1 << 28;
    const SDA_AP_GSPNIDEN: u32 = 1 << 7;
    const SDA_AP_GSPIDEN: u32 = 1 << 6;
    const SDA_AP_GNIDEN: u32 = 1 << 5;
    const SDA_AP_GDBGEN: u32 = 1 << 4;

    /// Value written to `DBGENCTRL` to enable all debug functions.
    const SDA_AP_EN_ALL: u32 = Self::SDA_AP_CNIDEN
        | Self::SDA_AP_CDBGEN
        | Self::SDA_AP_GSPNIDEN
        | Self::SDA_AP_GSPIDEN
        | Self::SDA_AP_GNIDEN
        | Self::SDA_AP_GDBGEN;

    /// Create a sequence handle for the S32K3xx.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self {
            simulate_reset_catch: AtomicBool::new(false),
        })
    }

    // ---- MC_ME (Mode Entry) — used to enable peripheral clocks ----
    // Reference: Keil S32K3xx_DFP pdsc, sequence `EnablePeripheralClocks` (S32K344).
    const MC_ME_CTL_KEY: u64 = 0x402D_C000;
    const MC_ME_KEY: u32 = 0x0000_5AF0;
    const MC_ME_INVKEY: u32 = 0x0000_A50F;

    const MC_ME_PRTN0_PCONF: u64 = 0x402D_C100;
    const MC_ME_PRTN0_PUPD: u64 = 0x402D_C104;
    const MC_ME_PRTN0_COFB1_CLKEN: u64 = 0x402D_C134;

    const MC_ME_PRTN1_PCONF: u64 = 0x402D_C300;
    const MC_ME_PRTN1_PUPD: u64 = 0x402D_C304;
    const MC_ME_PRTN1_COFB0_CLKEN: u64 = 0x402D_C330;
    const MC_ME_PRTN1_COFB1_CLKEN: u64 = 0x402D_C334;
    const MC_ME_PRTN1_COFB2_CLKEN: u64 = 0x402D_C338;
    const MC_ME_PRTN1_COFB3_CLKEN: u64 = 0x402D_C33C;

    const MC_ME_PRTN2_PCONF: u64 = 0x402D_C500;
    const MC_ME_PRTN2_PUPD: u64 = 0x402D_C504;
    const MC_ME_PRTN2_COFB0_CLKEN: u64 = 0x402D_C530;
    const MC_ME_PRTN2_COFB1_CLKEN: u64 = 0x402D_C534;

    // ---- eDMA / DMAMUX — used to bulk-init ECC RAM ----
    // The pdsc programs DMA channel 0 via DMAMUX_0 and TCD0.
    const DMAMUX0_CHCFG0: u64 = 0x4028_0003;
    const TCD0_BASE: u64 = 0x4021_0000;
    const TCD0_CH_CSR: u64 = Self::TCD0_BASE + 0x000;
    const TCD0_SADDR: u64 = Self::TCD0_BASE + 0x020;
    const TCD0_ATTR: u64 = Self::TCD0_BASE + 0x024;
    const TCD0_NBYTES: u64 = Self::TCD0_BASE + 0x028;
    const TCD0_SLAST_SDA: u64 = Self::TCD0_BASE + 0x02C;
    const TCD0_DADDR: u64 = Self::TCD0_BASE + 0x030;
    const TCD0_DOFF_CITER: u64 = Self::TCD0_BASE + 0x034;
    const TCD0_DLAST_SGA: u64 = Self::TCD0_BASE + 0x038;
    const TCD0_CSR: u64 = Self::TCD0_BASE + 0x03C;

    /// Bit in TCD_CH_CSR signalling the channel is still transferring.
    const TCD_ACTIVE: u32 = 0x8000_0000;

    fn enable_sda_debug(
        &self,
        interface: &mut dyn ArmDebugInterface,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        let sda_ap = FullyQualifiedApAddress::v1_with_dp(dp, Self::SDA_AP_INDEX);
        tracing::debug!(
            "S32K3xx: enabling debug via SDA-AP (ap={}, DBGENCTRL={:#x} <- {:#010x})",
            Self::SDA_AP_INDEX,
            Self::SDA_AP_DBGENCTRL_ADDR,
            Self::SDA_AP_EN_ALL,
        );
        interface.write_raw_ap_register(&sda_ap, Self::SDA_AP_DBGENCTRL_ADDR, Self::SDA_AP_EN_ALL)?;
        interface.flush()?;
        Ok(())
    }

    /// Commit a pending MC_ME partition update
    fn mc_me_commit(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
        memory.write_word_32(Self::MC_ME_CTL_KEY, Self::MC_ME_KEY)?;
        memory.write_word_32(Self::MC_ME_CTL_KEY, Self::MC_ME_INVKEY)?;
        Ok(())
    }

    fn enable_peripheral_clocks(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
        tracing::debug!("S32K3xx: enabling peripheral clocks (S32K344 partitions 0/1/2)");

        // Partition 0
        memory.write_word_32(Self::MC_ME_PRTN0_COFB1_CLKEN, 0x0000_F7DF)?;
        memory.write_word_32(Self::MC_ME_PRTN0_PCONF, 0x0000_0001)?;
        memory.write_word_32(Self::MC_ME_PRTN0_PUPD, 0x0000_0001)?;
        Self::mc_me_commit(memory)?;

        // Partition 1
        memory.write_word_32(Self::MC_ME_PRTN1_COFB0_CLKEN, 0xB1E0_FFF8)?;
        memory.write_word_32(Self::MC_ME_PRTN1_COFB1_CLKEN, 0x812A_A407)?;
        memory.write_word_32(Self::MC_ME_PRTN1_COFB2_CLKEN, 0xBBF3_FE7E)?;
        memory.write_word_32(Self::MC_ME_PRTN1_COFB3_CLKEN, 0x0000_0141)?;
        memory.write_word_32(Self::MC_ME_PRTN1_PCONF, 0x0000_0001)?;
        memory.write_word_32(Self::MC_ME_PRTN1_PUPD, 0x0000_0001)?;
        Self::mc_me_commit(memory)?;

        // Partition 2
        memory.write_word_32(Self::MC_ME_PRTN2_COFB0_CLKEN, 0x29FF_FFF0)?;
        memory.write_word_32(Self::MC_ME_PRTN2_COFB1_CLKEN, 0xC489_87F9)?;
        memory.write_word_32(Self::MC_ME_PRTN2_PCONF, 0x0000_0001)?;
        memory.write_word_32(Self::MC_ME_PRTN2_PUPD, 0x0000_0001)?;
        Self::mc_me_commit(memory)?;

        memory.flush()?;
        Ok(())
    }

    /// Program DMA channel 0 (TCD0) for a 64-bit-wide copy of `nbytes` from
    /// `src` to `dst`, then start the transfer and wait for completion.
    ///
    /// The TCD field encodings come straight from the pdsc `RAMInitialize`:
    /// `ATTR = 0x03030000` (SSIZE/DSIZE = 64-bit), `DOFF_CITER = 0x00010008`
    /// (8-byte destination stride, one major iteration), single major loop.
    fn dma_copy(
        memory: &mut dyn ArmMemoryInterface,
        src: u32,
        dst: u32,
        nbytes: u32,
    ) -> Result<(), ArmError> {
        // Re-arm the DMAMUX channel so the trigger comes from the explicit
        // start in TCD0_CSR rather than a peripheral request.
        memory.write_word_8(Self::DMAMUX0_CHCFG0, 0x80)?;
        memory.write_word_32(Self::TCD0_SADDR, src)?;
        memory.write_word_32(Self::TCD0_ATTR, 0x0303_0000)?;
        memory.write_word_32(Self::TCD0_NBYTES, nbytes)?;
        memory.write_word_32(Self::TCD0_SLAST_SDA, 0)?;
        memory.write_word_32(Self::TCD0_DADDR, dst)?;
        memory.write_word_32(Self::TCD0_DOFF_CITER, 0x0001_0008)?;
        // Last-destination-address adjustment is -nbytes so the channel
        // rewinds for the next reuse.
        memory.write_word_32(Self::TCD0_DLAST_SGA, nbytes.wrapping_neg())?;
        memory.write_word_32(Self::TCD0_CSR, 0x0000_0001)?;
        memory.flush()?;

        let start = Instant::now();
        loop {
            let csr = memory.read_word_32(Self::TCD0_CH_CSR)?;
            if csr & Self::TCD_ACTIVE == 0 {
                return Ok(());
            }
            if start.elapsed() > Duration::from_secs(1) {
                tracing::warn!("S32K3xx: DMA RAM-init timeout, CH_CSR={csr:#010x}");
                return Err(ArmError::Timeout);
            }
        }
    }

    /// Initialize SRAM and DTCM ECC via DMA.
    fn ram_initialize(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
        tracing::debug!("S32K3xx: RAMInitialize (S32K344: 320 KiB SRAM + 128 KiB DTCM)");

        // SRAM: 2 x 160 KiB @ 0x20400000
        Self::dma_copy(memory, 0x0040_0000, 0x2040_0000, 0x0005_0000)?;

        // DTCM: 1 x 128 KiB @ 0x20000000, accessed via the 0x21000000 backdoor.
        Self::dma_copy(memory, 0x0040_0000, 0x2100_0000, 0x0002_0000)?;

        Ok(())
    }

    /// Reset the chip via the MDM-AP
    fn functional_reset(
        probe: &mut dyn ArmMemoryInterface,
        release_core: bool,
    ) -> Result<(), ArmError> {
        let dp = probe.fully_qualified_address().dp();
        let interface = probe.get_arm_debug_interface()?;
        let mdm_ap = FullyQualifiedApAddress::v1_with_dp(dp, Self::MDM_AP_INDEX);

        tracing::debug!(
            "S32K3xx: FunctionalReset via MDM-AP (release_core={release_core})",
        );

        interface.write_raw_ap_register(&mdm_ap, Self::MDM_AP_CTL_ADDR, Self::MDMAPCTL_RSTRELCM7_DBGREQ)?;
        interface.write_raw_ap_register(
            &mdm_ap,
            Self::MDM_AP_CTL_ADDR,
            Self::MDMAPCTL_RSTRELCM7_DBGREQ_SYSFUNCRST,
        )?;
        interface.write_raw_ap_register(&mdm_ap, Self::MDM_AP_CTL_ADDR, Self::MDMAPCTL_RSTRELCM7_DBGREQ)?;
        if release_core {
            interface.write_raw_ap_register(&mdm_ap, Self::MDM_AP_CTL_ADDR, Self::MDMAPCTL_RSTRELCM7)?;
        }
        interface.flush()?;
        Ok(())
    }

    /// Halt or unhalt the core. Mirrors the MIMXRT10xx helper.
    fn halt(&self, probe: &mut dyn ArmMemoryInterface, halt: bool) -> Result<(), ArmError> {
        let mut dhcsr = Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?);
        dhcsr.set_c_halt(halt);
        dhcsr.set_c_debugen(true);
        dhcsr.enable_write();

        probe.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
        probe.flush()?;

        let start = Instant::now();
        let action = if halt { "halt" } else { "unhalt" };
        while Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?).s_halt() != halt {
            if start.elapsed() > Duration::from_millis(100) {
                tracing::debug!("Exceeded timeout while waiting for core to {action}");
                return Err(ArmError::Timeout);
            }
            thread::sleep(Duration::from_millis(1));
        }
        Ok(())
    }
}

impl ArmDebugSequence for S32K3xx {
    // Called after `debug_port_start` succeeds
    fn debug_device_unlock(
        &self,
        interface: &mut dyn ArmDebugInterface,
        default_ap: &FullyQualifiedApAddress,
        _permissions: &crate::Permissions,
    ) -> Result<(), ArmError> {
        //   1. EnableM7Debug          — write SDA-AP DBGENCTRL
        //   2. EnablePeripheralClocks — turn on the MC_ME partitions that
        //                               gate eDMA/DMAMUX
        //   3. RAMInitialize          — use DMA to initialize ECC across SRAM and DTCM
        self.enable_sda_debug(interface, default_ap.dp())?;

        let mut memory = interface.memory_interface(default_ap)?;
        Self::enable_peripheral_clocks(&mut *memory)?;
        Self::ram_initialize(&mut *memory)?;
        Ok(())
    }

    fn reset_catch_set(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(true, Ordering::Relaxed);
        Ok(())
    }

    fn reset_catch_clear(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(false, Ordering::Relaxed);
        Ok(())
    }

    fn reset_system(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        _core_type: crate::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        self.halt(interface, true)?;
        sequences::cortex_m_reset_system(interface)?;
        self.halt(interface, true)?;

        Ok(())
    }

    fn debug_core_stop(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        _core_type: crate::CoreType,
    ) -> Result<(), ArmError> {
        // The default `debug_core_stop` just clears DHCSR, which on S32K3
        // would resume the core from wherever the flash algorithm left it
        // (PC somewhere in the SRAM scratch area). The core then executes
        // garbage, takes a fault, and the FCCU loops the chip on reset.
        //
        // Instead, do a real reset on the way out: `FunctionalReset` with
        // `release_core=true` resets through MDM-AP and ends by dropping
        // CMnDBGREQ, so the CM7 comes out of reset and runs the freshly-
        // flashed application from its reset vector.
        Self::functional_reset(interface, true)
    }
}
