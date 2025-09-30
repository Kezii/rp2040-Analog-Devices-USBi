#![no_std]
#![no_main]

use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{SPI1, USB};
use embassy_rp::spi::{Blocking, Phase, Polarity, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, spi};
use embassy_time::Timer;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request};
use embassy_usb::driver::Direction;
use embassy_usb::{Builder, Config, Handler};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

struct AdauHal {
    spi: Spi<'static, SPI1, Blocking>,
    ss: Output<'static>,
    led: Output<'static>,
}

impl AdauHal {
    const WRITE_REGISTER: u8 = 0;
    const READ_REGISTER: u8 = 1;

    pub fn read_register(&mut self, register: u16, buffer: &mut [u8]) {
        info!("Reading register {:X} for size {}", register, buffer.len());

        self.select();
        self.spi.blocking_write(&[Self::READ_REGISTER]).unwrap();
        self.spi.blocking_write(&register.to_be_bytes()).unwrap();
        self.spi.blocking_read(buffer).unwrap();
        self.deselect();

        if buffer.len() <= 8 {
            info!("Read register: {:x}", buffer);
        }
    }

    pub fn write_register(&mut self, register: u16, data: &[u8]) -> Result<(), ()> {
        info!("Writing register {:X} for size {}", register, data.len());

        self.select();
        self.spi.blocking_write(&[Self::WRITE_REGISTER]).unwrap();
        self.spi.blocking_write(&register.to_be_bytes()).unwrap();
        self.spi.blocking_write(data).unwrap();
        self.deselect();

        Ok(())
    }

    pub fn select(&mut self) {
        self.ss.set_low();
        self.led.set_high();
    }

    pub fn deselect(&mut self) {
        self.ss.set_high();
        self.led.set_low();
    }

    /// By default, the slave port is in I2 C mode, but it can be put into
    /// SPI control mode by pulling SS/ADDR0 low three times. This
    /// can be done either by toggling the SS/ADDR0 successively
    /// between logic high and logic low states, or by performing three
    /// dummy writes to the SPI port
    pub async fn enable_spi(&mut self) {
        for _ in 0..3 {
            self.select();
            Timer::after_millis(10).await;
            self.deselect();
            Timer::after_millis(10).await;
        }
    }
}

struct ControlLogger {
    read_register: Option<u16>,
    adau_hal: AdauHal,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum UsbiRequest {
    WriteRegister = 178,
    ReadRegister = 179, // when host sends two bytes, read the register and respond with two bytes
    ReadRegisterAck = 181, // after ReadRegister, we return a 0x01 to ack ??
    ReadRegisterResponse = 180, // return the two byte register value
    ReadRegisterResponseAck = 182, // I don't know, just send another 0x01 after ReadRegisterResponse
}

impl UsbiRequest {
    fn from_u8(value: u8) -> Option<Self> {
        match value {
            178 => Some(UsbiRequest::WriteRegister),
            179 => Some(UsbiRequest::ReadRegister),
            181 => Some(UsbiRequest::ReadRegisterAck),
            180 => Some(UsbiRequest::ReadRegisterResponse),
            182 => Some(UsbiRequest::ReadRegisterResponseAck),
            _ => None,
        }
    }
}

impl Handler for ControlLogger {
    fn configured(&mut self, configured: bool) {
        debug!("Configured: {}", configured);
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if req.direction == Direction::Out && req.recipient == Recipient::Device {
            match UsbiRequest::from_u8(req.request).unwrap() {
                UsbiRequest::WriteRegister => {
                    // i primi 2 byte sono il register
                    // i restanti byte sono i dati
                    let register = u16::from_be_bytes([data[0], data[1]]);
                    let data = &data[2..];

                    while self.adau_hal.write_register(register, data).is_err() {
                        error!("Write register failed, retrying...");
                    }
                    return Some(OutResponse::Accepted);
                }

                UsbiRequest::ReadRegister => {
                    //
                    let register = u16::from_be_bytes([data[0], data[1]]);

                    self.read_register = Some(register);

                    return Some(OutResponse::Accepted);
                }

                _ => {}
            }
        };

        Some(OutResponse::Accepted)
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.direction == Direction::In && req.recipient == Recipient::Device {
            match UsbiRequest::from_u8(req.request).unwrap() {
                UsbiRequest::ReadRegisterAck | UsbiRequest::ReadRegisterResponseAck => {
                    return Some(InResponse::Accepted(&[1]));
                }
                UsbiRequest::ReadRegisterResponse => {
                    if let Some(reg_to_read) = self.read_register {
                        self.adau_hal
                            .read_register(reg_to_read, &mut buf[..req.length as usize]);

                        let len = req.length as usize;
                        return Some(InResponse::Accepted(&buf[..len]));
                    } else {
                        return Some(InResponse::Rejected);
                    }
                }

                _ => {}
            }
        }

        Some(InResponse::Rejected)
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello there!");

    let p = embassy_rp::init(Default::default());

    let mut config = spi::Config::default();
    config.frequency = 100_000;

    config.phase = Phase::CaptureOnSecondTransition;
    config.polarity = Polarity::IdleHigh;

    let spi = Spi::new_blocking(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, config);

    let ss = Output::new(p.PIN_13, Level::High);
    let led = Output::new(p.PIN_25, Level::High);
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config to match Analog Devices USBi (FX2)
    let mut config = Config::new(0x0456, 0x7031);
    config.manufacturer = Some("Analog Devices");
    config.product = Some("USBi");
    config.serial_number = None;
    config.max_power = 500;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 4096];

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // No Microsoft OS descriptors; match vendor-specific, driver-agnostic interface.

    let mut adau_hal = AdauHal { spi, ss, led };

    adau_hal.enable_spi().await;

    // Log all control traffic (EP0)
    static CONTROL_LOGGER: StaticCell<ControlLogger> = StaticCell::new();
    let control_logger = CONTROL_LOGGER.init(ControlLogger {
        read_register: None,
        adau_hal,
    });
    builder.handler(control_logger);

    // Match FS behavior: device advertises as Misc/IAD in FS, but single interface.
    // Keep composite_with_iads = true (default), which adds an IAD. That's fine; real FS shows an IAD too.
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    interface.alt_setting(0xFF, 0, 0, None);

    drop(function);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    usb.run().await;
}
