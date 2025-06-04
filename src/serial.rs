//! Serial communication (USART)
//!
//! This implementation consumes the following hardware resources:
//! - Output GPIO pin for transmission (TX)
//! - Input GPIO pin for reception (RX)
//! - Blocking delay provider (implements embedded_hal::delay::DelayNs)
//!
//! The delay provider must be configured to twice the desired communication frequency.
//!

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};
#[cfg(feature = "embedded-io")]
use embedded_io::{
    ErrorType as EmbeddedIoErrorType, Read as EmbeddedIoRead, Write as EmbeddedIoWrite,
};

/// Serial communication error type
#[derive(Debug)]
pub enum Error<E> {
    /// Bus error
    Bus(E),
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };
    use std::vec::Vec;

    // Helper to generate expected TX pin transactions for a byte (LSB first)
    fn tx_waveform(byte: u8) -> Vec<PinTransaction> {
        let mut transactions = Vec::new();
        // Start bit (low)
        transactions.push(PinTransaction::set(PinState::Low));
        // 8 data bits, LSB first
        let mut data = byte;
        for _ in 0..8 {
            if data & 1 == 1 {
                transactions.push(PinTransaction::set(PinState::High));
            } else {
                transactions.push(PinTransaction::set(PinState::Low));
            }
            data >>= 1;
        }
        // Stop bit (high)
        transactions.push(PinTransaction::set(PinState::High));
        // The implementation may call set_high again for idle after stop bit
        transactions.push(PinTransaction::set(PinState::High));
        transactions
    }

    #[test]
    fn test_serial_write_byte() {
        let byte = 0b01010101;
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

        let tx = DummyPin;
        let rx = DummyPin;
        let delay = DummyDelay;

        let mut serial = Serial::new(tx, rx, delay);
        #[cfg(feature = "embedded-io")]
        {
            use embedded_io::Write;
            let written = serial.write(&[byte]).expect("write failed");
            assert_eq!(written, 1);
        }
        #[cfg(not(feature = "embedded-io"))]
        {
            // Directly test the bitbanging logic if needed
            // (No public write method without embedded-io)
        }
    }

    // Helper to generate RX pin states for a byte (LSB first)
    fn rx_waveform(byte: u8) -> Vec<PinTransaction> {
        let mut transactions = Vec::new();
        // Start bit (low)
        transactions.push(PinTransaction::get(PinState::Low));
        // 8 data bits, LSB first
        let mut data = byte;
        for _ in 0..8 {
            if data & 0x80 != 0 {
                transactions.push(PinTransaction::get(PinState::High));
            } else {
                transactions.push(PinTransaction::get(PinState::Low));
            }
            data <<= 1;
        }
        // Stop bit (high)
        transactions.push(PinTransaction::get(PinState::High));
        // The implementation may check RX again after stop bit for idle
        transactions.push(PinTransaction::get(PinState::High));
        transactions
    }

    #[test]
    fn test_serial_read_byte() {
        let byte = 0b10101010;
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

        let tx = DummyPin;
        let rx = DummyPin;
        let delay = DummyDelay;

        let mut serial = Serial::new(tx, rx, delay);
        #[cfg(feature = "embedded-io")]
        {
            use embedded_io::Read;
            let mut buf = [0u8; 1];
            let read = serial.read(&mut buf).expect("read failed");
            assert_eq!(read, 1);
            assert_eq!(buf[0], byte);
        }
        #[cfg(not(feature = "embedded-io"))]
        {
            // Directly test the bitbanging logic if needed
            // (No public read method without embedded-io)
        }
    }
}

#[cfg(feature = "embedded-io")]
impl<E: core::fmt::Debug> embedded_io::Error for Error<E> {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

#[cfg(feature = "embedded-io")]
impl<TX, RX, Delay, E> EmbeddedIoErrorType for Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    type Error = Error<E>;
}

#[cfg(feature = "embedded-io")]
impl<TX, RX, Delay, E> EmbeddedIoRead for Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut read = 0;
        for slot in buf.iter_mut() {
            let mut data_in = 0u8;
            // Wait for start bit (RX low)
            while self.rx.is_high().map_err(Error::Bus)? {}
            self.wait_for_delay();
            for _ in 0..8 {
                data_in >>= 1;
                if self.rx.is_high().map_err(Error::Bus)? {
                    data_in |= 0x80;
                }
                self.wait_for_delay();
            }
            // Wait for stop bit
            self.wait_for_delay();
            *slot = data_in;
            read += 1;
        }
        Ok(read)
    }
}

#[cfg(feature = "embedded-io")]
impl<TX, RX, Delay, E> EmbeddedIoWrite for Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut written = 0;
        for &byte in buf {
            let mut data_out = byte;
            self.tx.set_low().map_err(Error::Bus)?; // start bit
            self.wait_for_delay();
            for _ in 0..8 {
                if data_out & 1 == 1 {
                    self.tx.set_high().map_err(Error::Bus)?;
                } else {
                    self.tx.set_low().map_err(Error::Bus)?;
                }
                data_out >>= 1;
                self.wait_for_delay();
            }
            self.tx.set_high().map_err(Error::Bus)?; // stop bit
            self.wait_for_delay();
            written += 1;
        }
        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // No internal buffering, so nothing to do
        Ok(())
    }
}

/// Bit banging serial communication (USART) device
#[cfg_attr(not(feature = "embedded-io"), allow(dead_code))]
pub struct Serial<TX, RX, Delay>
where
    TX: OutputPin,
    RX: InputPin,
    Delay: DelayNs,
{
    tx: TX,
    rx: RX,
    delay: Delay,
}

impl<TX, RX, Delay, E> Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
{
    /// Create instance
    pub fn new(tx: TX, rx: RX, delay: Delay) -> Self {
        Serial { tx, rx, delay }
    }

    #[inline]
    #[cfg_attr(not(feature = "embedded-io"), allow(dead_code))]
    fn wait_for_delay(&mut self) {
        self.delay.delay_ns(1);
    }
}
