//! An implementation of `embedded_hal::can for the STM32F4xx.

extern crate can_utils;
extern crate embedded_hal;
extern crate nb;
extern crate stm32f439;

// pub type Can1Wrapper = stm32f439::CAN1;
pub struct Can1Wrapper {
  can1: stm32f439::CAN1,
}

// rustc --explain E0117
impl embedded_hal::can::CanInterface for Can1Wrapper { }

impl embedded_hal::can::BaseCanInterface<can_utils::CanFrame> for Can1Wrapper {
  /// Maybe populates buf with a frame from the bus, and returns if it did so.
  fn receive(&mut self, buf: &mut can_utils::CanFrame) -> nb::Result<bool, u16> {
    nb::Error::WouldBlock
  }

  /// Sends a frame to all listeners on the bus.
  fn transmit(&mut self, frame: &can_utils::CanFrame) -> nb::Result<(), u16> {
    let id = if frame.ide {
      (frame.id << 3) | 0b100 | if frame.rtr {0b010} else {0}
    } else {
      (frame.id << 20) | if frame.rtr {0b010} else {0}
    }
    let data: [u32; 2] = unsafe { transmute_copy(frame.data) };

    if self.can1.tsr.read().tme0().bit() {
      self.can1.t0r.write(|w| unsafe { w.bits(id) });
      self.can1.tdt0r.modify(|_, w| unsafe { w.dlc().bits(frame.dlc) });
      self.can1.tdl0r.write(|w| unsafe { w.bits(data[0]) });
      self.can1.tdh0r.write(|w| unsafe { w.bits(data[1]) });
      self.can1.t0r.modify(|_, w| w.txrq().set_bit() );
    } else if self.can1.tsr.read().tme1().bit() {
      // TODO: ditto the above
    } else if self.can1.tsr.read().tme2().bit() {
      // TODO: ditto the above
    } else {
      nb::Error::WouldBlock
    }
  }

  /// Sets the bit timing parameters for the bus (for CAN-FD these are the parameters for the arbitration phase).
  ///
  /// Implementation Note: If this is a CAN-FD interface and the data phase speed registers are at
  /// their reset values, implementations should set the data phase parameters to match the values
  /// provided here.
  fn set_speed(&mut self, timing_parameters: can_utils::timing_calculator::CanBitTimingParameters) {
    // TODO: probably need to deactivate the interface before writing to these registers?
    // TODO: enforce a reasonable minimum on jump_width elsewhere
    self.can1.btr.modify(|_, w| unsafe {
      w.brp().bits(timing_parameters.baud_rate_prescaler - 1)
        .ts1().bits(timing_parameters.seg1.0 - 1)
        .ts2().bits(timing_parameters.seg2.0 - 1)
        .sjw().bits(timing_parameters.jump_width.0 - 1)
    });
    // TODO: turn the interface back on?
  }

  /// This returns the largest timing values the hardware can accept.
  fn maximum_timing_values(&self) -> can_utils::timing_calculator::CanBitTimingParameters {
    can_utils::timing_calculator::CanBitTimingParameters {
      baud_rate_prescaler: 1 << 10,  // it's a 10 bit field in the register
      seg1: can_utils::timing_calculator::SegmentLength(1 << 4),
      seg2: can_utils::timing_calculator::SegmentLength(1 << 3),
      jump_width: can_utils::timing_calculator::SegmentLength(1 << 2),
    }
  }

  /// Gets the curernt operation mode (as defined by CAN-FD but present in regular CAN).
  fn current_operation_mode(&self) -> can_utils::interface::InterfaceOperationMode {
    if self.is_asleep() || self.can1.msr.read().inak().bit() {
      can_utils::interface::InterfaceOperationMode::Integrating
    } else if self.can1.msr.read().txm().bit() {
      can_utils::interface::InterfaceOperationMode::Transmitter
    } else if self.can1.msr.read().rxm().bit() {
      can_utils::interface::InterfaceOperationMode::Receiver
    } else {
      can_utils::interface::InterfaceOperationMode::Idle
    }
  }

  /// Returns true iff this interface configured not to send dominant bits.
  ///
  /// Bus Monitoring Mode is defined in CAN-FD, but some MCUs (notably STM-32s) support this
  /// feature even on a regular CAN bus, check your local documentation closely to see if you
  /// have such a mode (though it may have a different name), otherwise return false.
  fn in_bus_monitoring_mode(&self) -> bool {
    self.can1.btr.read().silm().bit()
  }

  /// Gets the number of unused slots in the hardware message filter bank.
  ///
  /// Zero here may mean either that there is no hardware filtering support on this platform, or
  /// that it exists but is full.  If you absolutely must determine if hardware filtering exists
  /// clear the filters and then call this method... but realistically you probably don't need
  /// to know that.
  fn unused_filter_bank_count(&self)-> u32 {
    self.can1.fa1r.read().bits().count_zeros() - 4
  }

  /// Adds an incomming message filter.
  fn add_filter(&mut self, filter: &can_utils::interface::MessageFilter) {
    if !self.can1.fa1r.read().fact0().bit() {
      self.can1.fs1r.modify(|_, w| w.fsc0().set_bit() );
      self.can1.fm1r.modify(|_, w| w.fbm0().clear_bit() );
      self.can1.f0r1.write(|w| unsafe { w.bits(filter.id << 3) });
      if let Some(m) = filter.mask {
        self.can1.f0r2.write(|w| unsafe { w.bits(m << 3) });
      } else {
        self.can1.f0r2.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
      }
      self.can1.fa1r.modify(|_, w| w.fact0().set_bit());
    }
    // else if the other 27 times
  }

  /// Removes a single incomming message filter, if it exists.
  fn remove_filter(&mut self, filter: &can_utils::interface::MessageFilter) {
    if self.can1.f0r1.read().bits() == filter.id {
      self.can1.fa1r.modify(|_, w| w.fact0().clear_bit());
    }
    // ditto the above 27 times... perhaps a macro?
    // ...or perhaps abandoning svd2rust...
  }

  /// Remove all incomming message filters.  After this call all valid traffic on the BUS is available
  /// via `receive`.
  fn clear_filters(&mut self) {
    // 28 banks to clear
    self.can1.fa1r.reset()
  }

  /// Returns true iff the CAN hardware is in the sleep state.
  fn is_asleep(&self) -> bool {
    self.can1.msr.read().slak().bit()
  }

  /// Tell the hardware to enter the sleep state.
  fn request_sleep_mode(&mut self) {
    self.can1.mcr.modify(|_, w| w.sleep().set_bit());
  }

  /// Tell the hardware to leave the sleep state.
  fn request_wakeup(&mut self) {
    self.can1.mcr.modify(|_, w| w.sleep().clear_bit());
  }

  /// Fault confinement states are used by CAN to control how errors are reported.
  ///
  /// Implementation hint: if your hardware doesn't give you this information you can use
  /// can_utils::interface::FaultConfinementState::from_error_counts to infer it.
  fn fault_confinement_state(&self) -> can_utils::interface::FaultConfinementState {
    let rx = self.receive_error_count();
    let tx = self.transmit_error_count();
    can_utils::interface::confinement_state_from_error_counts(rx, tx)
  }

  /// Gets the receive error count.
  ///
  /// The exact rules for the meaning of the receive error count are too hairy to go into here,
  /// if you care what this means I'd encourage you to begin reading at page 24 of the CAN 2.0 spec.
  fn receive_error_count(&self) -> u32 { self.can1.esr.read().rec().bits() as u32 }

  /// Gets the transmit error count.
  ///
  /// The exact rules for the meaning of the transmit error count are too hairy to go into here,
  /// if you care what this means I'd encourage you to begin reading at page 24 of the CAN 2.0 spec.
  fn transmit_error_count(&self) -> u32 { self.can1.esr.read().tec().bits() as u32 }
}
