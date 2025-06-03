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
use embedded_io::{
    ErrorType as EmbeddedIoErrorType,
    Read as EmbeddedIoRead,
    Write as EmbeddedIoWrite,
};

/// Serial communication error type
#[derive(Debug)]
pub enum Error<E> {
    /// Bus error
    Bus(E),
}

impl<E: core::fmt::Debug> embedded_io::Error for Error<E> {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

// --- embedded-io trait implementations ---


impl<TX, RX, Delay, E> EmbeddedIoErrorType for Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    type Error = Error<E>;
}

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
    fn wait_for_delay(&mut self) {
        self.delay.delay_ns(1);
    }
}
