//! Raspberry Pi demo
//!
//! # Connections
//!
//! IMPORTANT: Do *not* use PIN24 / BCM8 / CE0 as the NCS pin
//!
//! - PIN1 = 3V3 = VCC
//! - PIN19 = BCM10 = MOSI (SDA)
//! - PIN21 = BCM9 = MISO (AD0)
//! - PIN23 = BCM11 = SCLK (SCL)
//! - PIN22 = BCM25 = NCS
//! - PIN15 = BCM22 = RDY
//! - PIN6 = GND = GND
//!
//! for further reference check https://pinout.xyz/#

extern crate linux_embedded_hal as hal;

use hal::spidev::{SpiModeFlags, SpidevOptions};
use hal::{Pin, Spidev};
use std::error::Error;
use std::thread;
use std::time::Duration;

use max31865::{FilterMode, Max31865, SensorWires};

fn main() -> Result<(), Box<dyn Error>> {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)
        .mode(SpiModeFlags::SPI_MODE_1)
        .build();
    spi.configure(&options).unwrap();
    let ncs = Pin::new(5); // Use GPIO 5
    ncs.export().unwrap();

    let mut max31865 = Max31865::new(spi, ncs).unwrap();
    max31865
        .configure(
            true,
            true,
            false,
            SensorWires::ThreeWire,
            FilterMode::Filter60Hz,
        )
        .unwrap();

    loop {
        let pt100 = max31865.read_default_conversion().unwrap();
        println!("PT100:  Temp: {:.2}ºC", f64::from(pt100) / 100.);
        thread::sleep(Duration::from_millis(1000));
    }
}
