//! A generic driver for the MAX31865 RTD to Digital converter
//!
//! # References
//! - Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf

#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::spi;
use hal::digital::v2::OutputPin;

#[cfg(feature = "doc")]
pub use examples;

pub mod temp_conversion;

pub enum FilterMode {
    Filter60Hz = 0,
    Filter50Hz = 1,
}

pub enum SensorWires {
    TwoOrFourWire = 0,
    ThreeWire = 1,
}

#[derive(Debug)]
pub struct Max31865<SPI: spi::Write<u8>, NCS: OutputPin> {
    spi: SPI,
    ncs: NCS,
    ref_resistor: u32,
}

impl<SPI, NCS, SPIErr, CSErr> Max31865<SPI, NCS>
where
    SPI: spi::Write<u8, Error = SPIErr> + spi::Transfer<u8, Error = SPIErr>,
    NCS: OutputPin<Error=CSErr>,
{
    /// Create a new MAX31865 module.
    ///
    /// # Arguments
    ///
    /// * `spi` - The SPI module to communicate on.
    /// * `ncs` - The chip select pin which should be set to a push pull output pin.
    /// * `wires` - The number of wires: 2, 3, 4 to enable the correct mode
    ///

    pub fn new(spi: SPI, mut ncs: NCS) -> Result<Max31865<SPI, NCS>, MxErr<SPIErr, CSErr>> {
        let default_calib = 43000;

        ncs.set_high().map_err(MxErr::CS)?;
        let max31865 = Max31865 {
            spi,
            ncs,
            ref_resistor: default_calib, /* value in ohms multiplied by 100 */
        };

        Ok(max31865)
    }

    /// Updates the devices configuration.
    ///
    /// # Arguments
    /// * `vbias` - Set to `true` to enable V_BIAS voltage, which is required to correctly perform conversion.Clone
    /// * `conversion_mode` - `true` to automatically perform conversion, otherwise normally off.
    /// * `one_shot` - Only perform detection once if set to `true`, otherwise repeats conversion.
    /// * `sensor_type` - Define whether a two, three or four wire sensor is used.Clone
    /// * `filter_mode` - Specify the mains frequency that should be used to filter out noise, e.g. 50Hz in Europe.
    ///
    /// # Remarks
    ///
    /// This will update the configuration register of the MAX31865 register. If the device doesn't properly react
    /// to this, add a delay after calling `new` to increase the time that the chip select line is set high.
    ///
    /// *Note*: The correct sensor configuration also requires changes to the PCB! Make sure to read the datasheet
    /// concerning this.
    pub fn configure(
        &mut self,
        vbias: bool,
        conversion_mode: bool,
        one_shot: bool,
        sensor_wires: SensorWires,
        filter_mode: FilterMode,
    ) -> Result<(), MxErr<SPIErr, CSErr>> {
        let conf: u8 = ((vbias as u8) << 7)
            | ((conversion_mode as u8) << 6)
            | ((one_shot as u8) << 5)
            | ((sensor_wires as u8) << 4)
            | (filter_mode as u8);

        self.write(Register::CONFIG, conf)?;

        Ok(())
    }

    /// Set the calibration reference resistance.
    /// This can be used to calibrate inaccuracies of both the reference resistor
    /// and the PT100 element.
    ///
    /// # Arguments
    ///
    /// * `calib` - A 32 bit integer specifying the reference resistance in ohms
    ///             multiplied by 100, e.g. `40000` for 400 Ohms
    ///
    /// # Remarks
    ///
    /// You can perform calibration by putting the sensor in boiling (100 degrees
    /// Celcius) water and then measuring the raw value using `read_raw`. Calculate
    /// `calib` as `(13851 << 15) / raw >> 1`.
    pub fn set_calibration(&mut self, calib: u32) -> Result<(), MxErr<SPIErr, CSErr>> {
        self.ref_resistor = calib;
        Ok(())
    }

    /// Read the raw resistance value and then perform conversion to degrees Celcius.
    ///
    /// # Remarks
    ///
    /// The output value is the value in degrees Celcius multiplied by 100.
    pub fn read_default_conversion(&mut self) -> Result<u32, MxErr<SPIErr, CSErr>> {
        let raw = self.read_16(Register::RTD_MSB)?;
        let ohms = (u32::from(raw >> 1) * self.ref_resistor) >> 15;
        let temp = temp_conversion::lookup_temperature(ohms as u16);

        Ok(temp)
    }

    /// Read the raw RTD value.
    ///
    /// # Remarks
    ///
    /// The raw value is the value of the combined MSB and LSB registers.
    /// The first 15 bits specify the ohmic value in relation to the reference
    /// resistor (i.e. 2^15 - 1 would be the exact same resistance as the reference
    /// resistor). See manual for further information.
    /// The last bit specifies if the conversion was successful.
    /// https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
    /// http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
    fn read_16(&mut self, reg: Register) -> Result<u16, MxErr<SPIErr, CSErr>> {
        let mut buffer = [0; 3];
        buffer[0] = reg.read_address();
        {
            self.ncs.set_low().map_err(MxErr::CS)?;
            self.spi.transfer(&mut buffer).map_err(MxErr::SPI)?;
            self.ncs.set_high().map_err(MxErr::CS)?;
        }
        let r: u16 = (u16::from(buffer[1]) << 8) | u16::from(buffer[2]);

        Ok(r)
    }

    fn read_8(&mut self, reg: Register) -> Result<u8, MxErr<SPIErr, CSErr>> {
        let mut buffer = [0xFF; 4];
        buffer[0] = reg.read_address();
        {
            self.ncs.set_low().map_err(MxErr::CS)?;
            self.spi.transfer(&mut buffer).map_err(MxErr::SPI)?;
            self.ncs.set_high().map_err(MxErr::CS)?;
        }
        let r: u8 = buffer[1];

        Ok(r)
    }

    /// Determine if a new conversion is available
    ///
    /// # Remarks
    ///
    /// When the module is finished converting the temperature it sets the
    /// ready pin to low. It is automatically returned to high upon reading the
    /// RTD registers.
    // pub fn is_ready(&self) -> Result<bool, E> {
    // Ok(self.rdy.is_low()?)
    // }
    fn write(&mut self, reg: Register, val: u8) -> Result<(), MxErr<SPIErr, CSErr>> {
        self.ncs.set_low().map_err(MxErr::CS)?;
        self.spi.write(&[reg.write_address(), val]).map_err(MxErr::SPI)?;
        self.ncs.set_high().map_err(MxErr::CS)?;
        Ok(())
    }
}

#[derive(Debug)]
pub enum MxErr<SPIErr, CSErr> {
    SPI(SPIErr),
    CS(CSErr),
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
enum Register {
    CONFIG = 0x00,
    RTD_MSB = 0x01,
    RTD_LSB = 0x02,
    HIGH_FAULT_THRESHOLD_MSB = 0x03,
    HIGH_FAULT_THRESHOLD_LSB = 0x04,
    LOW_FAULT_THRESHOLD_MSB = 0x05,
    LOW_FAULT_THRESHOLD_LSB = 0x06,
    FAULT_STATUS = 0x07,
}

const R: u8 = 0 << 7;
const W: u8 = 1 << 7;

impl Register {
    fn read_address(self) -> u8 {
        self as u8 | R
    }

    fn write_address(self) -> u8 {
        self as u8 | W
    }
}
