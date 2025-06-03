/*!
  # Synchronous implementation of embedded-hal 1.0.0 I2C traits based on GPIO bitbang

  This implementation consumes the following hardware resources:
  - A periodic timer to mark clock cycles
  - Two GPIO pins for SDA and SCL lines.

  Note: This implementation does not support I2C clock stretching.

  ## Hardware requirements

  1. Configure GPIO pins as Open-Drain outputs.
  2. Configure timer frequency to be twice the desired I2C clock frequency.

  ## Example

  ```rust,ignore
    let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);
    let mut sensor = Lm75::new(i2c, SlaveAddr::default());
    let temp = sensor.read_temperature().unwrap();
  ```
*/

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{ErrorType, I2c, Operation};

/// I2C error
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    /// GPIO error
    Bus(E),
    /// No ack received
    NoAck,
    /// Invalid input
    InvalidData,
}

impl<E: core::fmt::Debug> embedded_hal::i2c::Error for Error<E> {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::Bus(_) => embedded_hal::i2c::ErrorKind::Other,
            Error::NoAck => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            ),
            Error::InvalidData => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

/// Bit banging I2C device
pub struct I2cBB<SCL, SDA, CLK, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: DelayNs,
    E: core::fmt::Debug,
{
    scl: SCL,
    sda: SDA,
    clk: CLK,
}

impl<SCL, SDA, CLK, E> I2cBB<SCL, SDA, CLK, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: DelayNs,
    E: core::fmt::Debug,
{
    /// Create instance
    pub fn new(scl: SCL, sda: SDA, clk: CLK) -> Self {
        I2cBB { scl, sda, clk }
    }

    fn set_scl_high(&mut self) -> Result<(), Error<E>> {
        self.scl.set_high().map_err(Error::Bus)
    }
    fn set_scl_low(&mut self) -> Result<(), Error<E>> {
        self.scl.set_low().map_err(Error::Bus)
    }
    fn set_sda_high(&mut self) -> Result<(), Error<E>> {
        self.sda.set_high().map_err(Error::Bus)
    }
    fn set_sda_low(&mut self) -> Result<(), Error<E>> {
        self.sda.set_low().map_err(Error::Bus)
    }
    fn wait_for_clk(&mut self) {
        self.clk.delay_ns(1);
    }

    fn raw_i2c_start(&mut self) -> Result<(), Error<E>> {
        self.set_scl_high()?;
        self.set_sda_high()?;
        self.wait_for_clk();
        self.set_sda_low()?;
        self.wait_for_clk();
        self.set_scl_low()?;
        self.wait_for_clk();
        Ok(())
    }

    fn raw_i2c_stop(&mut self) -> Result<(), Error<E>> {
        self.set_scl_high()?;
        self.wait_for_clk();
        self.set_sda_high()?;
        self.wait_for_clk();
        Ok(())
    }

    fn i2c_is_ack(&mut self) -> Result<bool, Error<E>> {
        self.set_sda_high()?;
        self.set_scl_high()?;
        self.wait_for_clk();
        let ack = self.sda.is_low().map_err(Error::Bus)?;
        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk();
        Ok(ack)
    }

    fn i2c_write_byte(&mut self, byte: u8) -> Result<(), Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = (byte >> (7 - bit_offset)) & 0b1;
            if out_bit == 1 {
                self.set_sda_high()?;
            } else {
                self.set_sda_low()?;
            }
            self.set_scl_high()?;
            self.wait_for_clk();
            self.set_scl_low()?;
            self.set_sda_low()?;
            self.wait_for_clk();
        }
        Ok(())
    }

    fn i2c_read_byte(&mut self, should_send_ack: bool) -> Result<u8, Error<E>> {
        let mut byte: u8 = 0;
        self.set_sda_high()?;
        for bit_offset in 0..8 {
            self.set_scl_high()?;
            self.wait_for_clk();
            if self.sda.is_high().map_err(Error::Bus)? {
                byte |= 1 << (7 - bit_offset);
            }
            self.set_scl_low()?;
            self.wait_for_clk();
        }
        if should_send_ack {
            self.set_sda_low()?;
        } else {
            self.set_sda_high()?;
        }
        self.set_scl_high()?;
        self.wait_for_clk();
        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk();
        Ok(byte)
    }

    fn check_ack(&mut self) -> Result<(), Error<E>> {
        if !self.i2c_is_ack()? {
            Err(Error::NoAck)
        } else {
            Ok(())
        }
    }

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<E>> {
        for byte in bytes {
            self.i2c_write_byte(*byte)?;
            self.check_ack()?;
        }
        Ok(())
    }

    fn read_bytes(&mut self, buffer: &mut [u8]) -> Result<(), Error<E>> {
        for i in 0..buffer.len() {
            let should_send_ack = i != (buffer.len() - 1);
            buffer[i] = self.i2c_read_byte(should_send_ack)?;
        }
        Ok(())
    }
}

impl<SCL, SDA, CLK, E> ErrorType for &mut I2cBB<SCL, SDA, CLK, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: DelayNs,
    E: core::fmt::Debug,
{
    type Error = Error<E>;
}

impl<SCL, SDA, CLK, E> I2c<u8> for &mut I2cBB<SCL, SDA, CLK, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: DelayNs,
    E: core::fmt::Debug,
{
    fn transaction(
        &mut self,
        addr: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        // Helper to get R/W bit for Operation
        fn is_read(op: &Operation<'_>) -> bool {
            matches!(op, Operation::Read(_))
        }

        let mut first = true;
        let mut current_type = None;

        for op in operations.iter_mut() {
            let op_is_read = is_read(op);

            // Start or repeated start if switching type
            if first || current_type != Some(op_is_read) {
                if first {
                    self.raw_i2c_start()?;
                    first = false;
                } else {
                    self.raw_i2c_start()?; // repeated start
                }
                let rw_bit = if op_is_read { 0x1 } else { 0x0 };
                self.i2c_write_byte((addr << 1) | rw_bit)?;
                self.check_ack()?;
                current_type = Some(op_is_read);
            }

            // Process the current op
            match op {
                Operation::Read(buf) => self.read_bytes(buf)?,
                Operation::Write(buf) => self.write_bytes(buf)?,
            }
        }

        self.raw_i2c_stop()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{I2c, Operation};
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };
    use std::vec::Vec;

    // No need to implement Debug for MockError; use PinMock without generics.

    fn pin_transactions(states: &[PinState]) -> Vec<PinTransaction> {
        states.iter().map(|&s| PinTransaction::set(s)).collect()
    }

    #[test]
    fn test_raw_i2c_start_and_stop() {
        let scl = PinMock::new(&pin_transactions(&[
            PinState::High, // set_scl_high
            PinState::Low,  // set_scl_low
            PinState::High, // set_scl_high (stop)
        ]));
        let sda = PinMock::new(&pin_transactions(&[
            PinState::High, // set_sda_high
            PinState::Low,  // set_sda_low
            PinState::High, // set_sda_high (stop)
        ]));
        let clk = MockDelay::new();

        let mut i2c = I2cBB::new(scl, sda, clk);
        i2c.raw_i2c_start().expect("start failed");
        i2c.raw_i2c_stop().expect("stop failed");

        i2c.scl.done();
        i2c.sda.done();
    }

    #[test]
    fn test_write_and_ack() {
        // 0b10101010
        let scl = PinMock::new(&vec![
            // Each bit: set_scl_high, set_scl_low, set_sda_low after clock
            PinTransaction::set(PinState::High), // bit 7 clock high
            PinTransaction::set(PinState::Low),  // bit 7 clock low
            PinTransaction::set(PinState::High), // bit 6 clock high
            PinTransaction::set(PinState::Low),  // bit 6 clock low
            PinTransaction::set(PinState::High), // bit 5 clock high
            PinTransaction::set(PinState::Low),  // bit 5 clock low
            PinTransaction::set(PinState::High), // bit 4 clock high
            PinTransaction::set(PinState::Low),  // bit 4 clock low
            PinTransaction::set(PinState::High), // bit 3 clock high
            PinTransaction::set(PinState::Low),  // bit 3 clock low
            PinTransaction::set(PinState::High), // bit 2 clock high
            PinTransaction::set(PinState::Low),  // bit 2 clock low
            PinTransaction::set(PinState::High), // bit 1 clock high
            PinTransaction::set(PinState::Low),  // bit 1 clock low
            PinTransaction::set(PinState::High), // bit 0 clock high
            PinTransaction::set(PinState::Low),  // bit 0 clock low
        ]);
        let sda = PinMock::new(&vec![
            // Each bit: set_sda_high/low, set_sda_low after clock
            PinTransaction::set(PinState::High), // 1
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::Low), // 0
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High), // 1
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::Low), // 0
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High), // 1
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::Low), // 0
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High), // 1
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::Low), // 0
            PinTransaction::set(PinState::Low),
        ]);
        let clk = MockDelay::new();

        let mut i2c = I2cBB::new(scl, sda, clk);
        i2c.i2c_write_byte(0b10101010).expect("write failed");
        i2c.scl.done();
        i2c.sda.done();
    }

    #[test]
    fn test_i2c_trait_write() {
        // Use dummy pins that do nothing and always succeed.
        use core::convert::Infallible;
        struct DummyPin;
        impl embedded_hal::digital::ErrorType for DummyPin {
            type Error = Infallible;
        }
        impl embedded_hal::digital::OutputPin for DummyPin {
            fn set_high(
                &mut self,
            ) -> Result<(), <Self as embedded_hal::digital::ErrorType>::Error> {
                Ok(())
            }
            fn set_low(&mut self) -> Result<(), <Self as embedded_hal::digital::ErrorType>::Error> {
                Ok(())
            }
        }
        impl embedded_hal::digital::InputPin for DummyPin {
            fn is_high(
                &mut self,
            ) -> Result<bool, <Self as embedded_hal::digital::ErrorType>::Error> {
                Ok(false)
            }
            fn is_low(
                &mut self,
            ) -> Result<bool, <Self as embedded_hal::digital::ErrorType>::Error> {
                Ok(true)
            }
        }
        struct DummyDelay;
        impl embedded_hal::delay::DelayNs for DummyDelay {
            fn delay_ns(&mut self, _ns: u32) {}
        }

        let scl = DummyPin;
        let sda = DummyPin;
        let clk = DummyDelay;

        let mut i2c = I2cBB::new(scl, sda, clk);
        let mut ops = [Operation::Write(&[0xAB])];
        I2c::transaction(&mut &mut i2c, 0x50, &mut ops).expect("i2c write failed");
    }
}
