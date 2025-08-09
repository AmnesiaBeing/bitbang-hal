//! Serial Peripheral Interface
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for clock signal (SCLK)
//! - Output GPIO pin for data transmission (Master Output Slave Input - MOSI)
//! - Input GPIO pin for data reception (Master Input Slave Output - MISO)
//!
//! The timer must be configured to twice the desired communication frequency.
//!
//! SS/CS (slave select) must be handled independently.
//!
//! MSB-first and LSB-first bit orders are supported.
//!

use core::cmp::max;

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    spi::{ErrorType, Mode, Polarity, SpiBus, SpiDevice, MODE_0, MODE_1, MODE_2, MODE_3},
};

/// Error type
#[derive(Debug)]
pub enum Error<E> {
    /// Communication error
    Bus(E),
    /// Attempted read without input data
    NoData,
}

impl<E: core::fmt::Debug> embedded_hal::spi::Error for Error<E> {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            Error::Bus(_) => embedded_hal::spi::ErrorKind::Other,
            Error::NoData => embedded_hal::spi::ErrorKind::Other,
        }
    }
}

/// Transmission bit order
#[derive(Debug)]
pub enum BitOrder {
    /// Most significant bit first
    MSBFirst,
    /// Least significant bit first
    LSBFirst,
}

impl Default for BitOrder {
    /// Default bit order: MSB first
    fn default() -> Self {
        BitOrder::MSBFirst
    }
}

/// Configuration of the SPI Interface
pub struct SpiConfig {
    mode: Mode,
    bit_order: BitOrder,
    /// Value used when still receiving bits from MISO, but not value is available
    /// in buffer to be written to MOSI. This value is used instead. Usually `0x00``
    empty_write_value: u8,
    /// controls the clock speed. `f = 2 / half_period_duration_ns`
    half_period_duration_ns: u32,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: Default::default(),
            empty_write_value: 0x00,
            half_period_duration_ns: 10, // 100 kHz.
        }
    }
}

/// A Full-Duplex SPI implementation, takes 3 pins, and a timer running at 2x
/// the desired SPI frequency.
pub struct SPIBus<Mosi, Sck, Delay>
where
    Mosi: OutputPin,
    Sck: OutputPin,
    Delay: DelayNs,
{
    mosi: Mosi,
    sck: Sck,
    delay: Delay,
    config: SpiConfig,
}

impl<Mosi, Sck, Delay, E> SPIBus<Mosi, Sck, Delay>
where
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Delay: DelayNs,
{
    /// Create instance
    pub fn new(mode: Mode, mosi: Mosi, sck: Sck, delay: Delay, spi_config: SpiConfig) -> Self {
        let mut spi = SPIBus {
            mosi,
            sck,
            delay,
            config: spi_config,
        };

        match mode.polarity {
            Polarity::IdleLow => spi.sck.set_low(),
            Polarity::IdleHigh => spi.sck.set_high(),
        }
        .unwrap_or(());

        spi
    }

    /// Set transmission bit order
    pub fn with_bit_order(&mut self, order: BitOrder) {
        self.config.bit_order = order;
    }

    fn read_bit(&mut self, read_val: &mut u8) -> Result<(), crate::spi_halfduplex::Error<E>> {
        *read_val = 1;
        Ok(())
    }

    #[inline]
    fn set_clk_high(&mut self) -> Result<(), crate::spi_halfduplex::Error<E>> {
        self.sck.set_high().map_err(Error::Bus)
    }

    #[inline]
    fn set_clk_low(&mut self) -> Result<(), crate::spi_halfduplex::Error<E>> {
        self.sck.set_low().map_err(Error::Bus)
    }

    #[inline]
    fn wait_for_delay(&mut self) {
        self.delay.delay_ns(self.config.half_period_duration_ns);
    }

    #[inline]
    fn rw_byte(
        &mut self,
        clock_out: u8,
        read_in: &mut u8,
    ) -> Result<(), crate::spi_halfduplex::Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = match self.config.bit_order {
                BitOrder::MSBFirst => (clock_out >> (7 - bit_offset)) & 0b1,
                BitOrder::LSBFirst => (clock_out >> bit_offset) & 0b1,
            };

            if out_bit == 1 {
                self.mosi.set_high().map_err(Error::Bus)?;
            } else {
                self.mosi.set_low().map_err(Error::Bus)?;
            }

            match self.config.mode {
                MODE_0 => {
                    self.wait_for_delay();
                    self.set_clk_high()?;
                    self.read_bit(read_in)?;
                    self.wait_for_delay();
                    self.set_clk_low()?;
                }
                MODE_1 => {
                    self.set_clk_high()?;
                    self.wait_for_delay();
                    self.read_bit(read_in)?;
                    self.set_clk_low()?;
                    self.wait_for_delay();
                }
                MODE_2 => {
                    self.wait_for_delay();
                    self.set_clk_low()?;
                    self.read_bit(read_in)?;
                    self.wait_for_delay();
                    self.set_clk_high()?;
                }
                MODE_3 => {
                    self.set_clk_low()?;
                    self.wait_for_delay();
                    self.read_bit(read_in)?;
                    self.set_clk_high()?;
                    self.wait_for_delay();
                }
            };
        }
        Ok(())
    }
}

impl<Mosi, Sck, Delay, E> ErrorType for SPIBus<Mosi, Sck, Delay>
where
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    type Error = crate::spi_halfduplex::Error<E>;
}

impl<Mosi, Sck, Delay, E> SpiBus<u8> for SPIBus<Mosi, Sck, Delay>
where
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    #[inline]
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            self.rw_byte(self.config.empty_write_value, word)?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_write = 0u8;
        for byte in words {
            self.rw_byte(*byte, &mut ignored_write)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_write = 0u8;
        for i in 0..max(read.len(), write.len()) {
            let read_in_byte = read.get_mut(i).unwrap_or(&mut ignored_write);
            let clock_out_byte = write
                .get(i)
                .copied()
                .unwrap_or(self.config.empty_write_value);
            self.rw_byte(clock_out_byte, read_in_byte)?;
        }

        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut current_read_byte = 0u8;
        for clock_out_byte in words {
            self.rw_byte(*clock_out_byte, &mut current_read_byte)?;
            *clock_out_byte = current_read_byte;
        }

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // we always flush. Nothing to do here.
        Ok(())
    }
}

/// A Full-Duplex SPI implementation, takes 3 pins, and a timer running at 2x
/// the desired SPI frequency.
pub struct SPIDevice<Mosi, Sck, Cs, Delay>
where
    Mosi: OutputPin,
    Sck: OutputPin,
    Cs: OutputPin,
    Delay: DelayNs,
{
    mosi: Mosi,
    sck: Sck,
    cs: Cs,
    delay: Delay,
    config: SpiConfig,
}

impl<Mosi, Sck, Cs, Delay, E> ErrorType for SPIDevice<Mosi, Sck, Cs, Delay>
where
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Cs: OutputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    type Error = crate::spi_halfduplex::Error<E>;
}

impl<Mosi, Sck, Cs, Delay, E> SpiDevice for SPIDevice<Mosi, Sck, Cs, Delay>
where
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Cs: OutputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.transaction(&mut [embedded_hal::spi::Operation::Read(buf)])
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.transaction(&mut [embedded_hal::spi::Operation::Write(buf)])
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }
    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        unimplemented!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::spi::MODE_0;
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };
    use std::vec::Vec;

    fn waveform(string: &str) -> Vec<PinState> {
        let mut transactions = Vec::new();
        let mut last_state = None;
        let mut last_action = string.chars().next().unwrap();
        for step in string.chars() {
            let step = if step == '.' { last_action } else { step };
            match step {
                '0' => {
                    transactions.push(PinState::Low);
                    last_state = Some(PinState::Low);
                }
                '1' => {
                    transactions.push(PinState::High);
                    last_state = Some(PinState::High);
                }
                'p' | 'P' => {
                    let next_state = if last_state == Some(PinState::Low) {
                        PinState::High
                    } else {
                        PinState::Low
                    };
                    transactions.push(next_state);
                    last_state = Some(next_state);
                }
                'n' | 'N' => {
                    let next_state = if last_state == Some(PinState::High) {
                        PinState::Low
                    } else {
                        PinState::High
                    };
                    transactions.push(next_state);
                    last_state = Some(next_state);
                }
                _ => panic!("Invalid binary literal"),
            };
            last_action = step;
        }
        transactions
    }

    #[test]
    fn test_states() {
        let res = waveform("p..");
        assert_eq!(res, vec![PinState::Low, PinState::High, PinState::Low]);

        let res = waveform("P..");
        assert_eq!(res, vec![PinState::Low, PinState::High, PinState::Low]);

        let res = waveform("n..");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::High]);

        let res = waveform("N..");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::High]);

        let res = waveform("n.0");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::Low]);
    }

    fn _input_waveform(string: &str) -> Vec<PinTransaction> {
        waveform(string)
            .into_iter()
            .map(PinTransaction::get)
            .collect()
    }

    fn output_waveform(string: &str) -> Vec<PinTransaction> {
        waveform(string)
            .into_iter()
            .map(PinTransaction::set)
            .collect()
    }

    #[test]
    fn test_spi_write_single_byte() {
        let mosi = PinMock::new(&output_waveform("01010101"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let mut spi = SPIBus::new(MODE_0, mosi, sck, delay, SpiConfig::default());
        let data = [0b01010101];
        spi.write(&data).expect("SPI write failed");

        // Verify that all transactions were completed
        spi.mosi.done();
        spi.sck.done();
    }
}
