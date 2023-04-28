//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::{
    debug,
    error,
    info,
};
use defmt_rtt as _;
//use fugit::RateExtU32;
use embedded_time::rate::*;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
//use rp_pico as bsp;

use rp2040_hal as hal;
//use bsp::hal;

use core::fmt::Write;
use embedded_hal::spi::MODE_0;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FloatingInput, PushPullOutput},
    i2c::I2C,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use heapless::String;

use bme280::i2c::BME280;

use esp32_wroom_rp::{
    gpio::EspControlPins, network::IpAddress, network::Port, network::TransportMode,
    tcp_client::Connect, tcp_client::TcpClient, wifi::ConnectionStatus, wifi::Wifi,
};

const MAX_HTTP_DOC_LENGTH: usize = 4096 as usize;

const SSID: &str = "Zion";
const PASSPHRASE: &str = "Diesel12103465";

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use embedded_hal_alpha::delay::blocking::DelayUs;

// Until cortex_m implements the DelayUs trait needed for embedded-hal-1.0.0,
// provide a wrapper around it
pub struct DelayWrap(cortex_m::delay::Delay);

impl embedded_hal_alpha::delay::blocking::DelayUs for DelayWrap {
    type Error = core::convert::Infallible;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.0.delay_us(us);

        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.0.delay_ms(ms);
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

   // Configure two pins as being I²C, not GPIO
   let sda_pin = pins.gpio26.into_mode::<hal::gpio::FunctionI2C>();
   let scl_pin = pins.gpio27.into_mode::<hal::gpio::FunctionI2C>();

   // Create the I²C drive, using the two pre-configured pins. This will fail
   // at compile time if the pins are in the wrong mode, or if this I²C
   // peripheral isn't available on these pins!
   let i2c = I2C::i2c1(
       pac.I2C1,
       sda_pin,
       scl_pin, // Try `not_an_scl_pin` here
       400.kHz(),
       &mut pac.RESETS,
       clocks.peripheral_clock.freq(),
   );

   debug!("BME280 example\r\n");

   let mut delay = DelayWrap(cortex_m::delay::Delay::new(
       core.SYST,
       clocks.system_clock.freq().integer(),
   ));

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_miso = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialized SPI driver for an initialized one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8000000.Hz(), // 8 MHz
        &MODE_0,
    );

    let esp_pins = EspControlPins {
        // CS on pin x (GPIO7)
        cs: pins.gpio7.into_mode::<PushPullOutput>(),
        // GPIO0 on pin x (GPIO2)
        gpio0: pins.gpio2.into_mode::<PushPullOutput>(),
        // RESETn on pin x (GPIO11)
        resetn: pins.gpio11.into_mode::<PushPullOutput>(),
        // ACK on pin x (GPIO10)
        ack: pins.gpio10.into_mode::<FloatingInput>(),
    };

    let mut wifi = Wifi::init(spi, esp_pins, &mut delay.0).unwrap();

    let result = wifi.join(SSID, PASSPHRASE);
    info!("Join Result: {:?}", result);

   // initialize the BME280 using the secondary I2C address 0x77
   let mut bme280 = BME280::new_secondary(i2c);

   // initialize the sensor
   match bme280.init(&mut delay) {
       Ok(_) => defmt::debug!("Successfully initialized BME280 device\r\n"),
       Err(_) => defmt::debug!("Failed to initialize BME280 device\r\n"),
   }

   let mut sleep: u32 = 1500;
   loop {
       match wifi.get_connection_status() {
            Ok(status) => {
                defmt::info!("Connection status: {:?}", status);
                delay.delay_ms(sleep).ok().unwrap();

                if status == ConnectionStatus::Connected {
                    defmt::info!("Connected to network: {:?}", SSID);

                    // The IPAddresses of two DNS servers to resolve hostnames with.
                    // Note that failover from ip1 to ip2 is fully functional.
                    let ip1: IpAddress = [9, 9, 9, 9];
                    let ip2: IpAddress = [8, 8, 8, 8];
                    let dns_result = wifi.set_dns(ip1, Some(ip2));

                    defmt::info!("set_dns result: {:?}", dns_result);

                    // let hostname = "github.com";
                    let ip_address: IpAddress = [10, 0, 1, 3]; // production Ambi server

                    let port: Port = 80;
                    let mode: TransportMode = TransportMode::Tcp;

                    // measure temperature, pressure, and humidity
                    let measurements = bme280.measure(&mut delay).unwrap();

                    debug!("Relative humidity: {:?}%\r", &measurements.humidity);
                    debug!("Temperature: {:?} deg C\r", &measurements.temperature);
                    debug!("Pressure: {:?} pascals\r\n", &measurements.pressure);

                    let mut http_document: String<MAX_HTTP_DOC_LENGTH> = String::from("POST /api/readings/add");
                    http_document
                        .push_str(" HTTP/1.1\r\nHost: ")
                        .ok()
                        .unwrap();
                    let mut host_address_str: String<MAX_HTTP_DOC_LENGTH> = String::new();
                    write!(
                        host_address_str,
                        "{}.{}.{}.{}:{:?}\r\n",
                        ip_address[0],
                        ip_address[1],
                        ip_address[2],
                        ip_address[3],
                        port
                    )
                    .unwrap();
                    http_document.push_str(&host_address_str).ok().unwrap();
                    http_document
                        .push_str("User-Agent: edge/0.0.1\r\n")
                        .ok()
                        .unwrap();
                    http_document.push_str("Accept: */*\r\n").ok().unwrap();
                    http_document
                        .push_str("Content-Type: application/json\r\n")
                        .ok()
                        .unwrap();
                    let mut json_str: String<MAX_HTTP_DOC_LENGTH> = String::new();
                    write!(json_str,
                        "{{\"temperature\":\"{:.1?}\",\"humidity\":\"{:.1?}\",\"pressure\":\"{:.0?}\",\"dust_concentration\":\"200\",\"air_purity\":\"Low Pollution\"}}\r\n",
                        measurements.temperature, measurements.humidity, measurements.pressure / 100.0
                    ).ok().unwrap();
                    let mut content_len_str: String<MAX_HTTP_DOC_LENGTH> = String::new();
                    write!(content_len_str, "{:?}\r\n", json_str.len())
                        .ok()
                        .unwrap();
                    http_document.push_str("Content-Length: ").ok().unwrap();
                    http_document.push_str(&content_len_str).ok().unwrap();
                    http_document.push_str("\r\n").ok().unwrap();
                    http_document.push_str(&json_str).ok().unwrap();
                    http_document.push_str("\r\n").ok().unwrap();
                    write!(http_document, "GET / HTTP/1.1\r\nHost: {}.{}.{}.{}:{}\r\nAccept: */*\r\n\r\n",
                        ip_address[0],
                        ip_address[1],
                        ip_address[2],
                        ip_address[3],
                        port
                    ).ok().unwrap();

                    if let Err(e) = TcpClient::build(&mut wifi, &mut delay.0).connect(
                        ip_address,
                        port,
                        mode,
                        &mut |tcp_client| {
                            //info!("TCP connection to {:?}:{:?} successful", hostname, port);
                            info!("Hostname: {:?}", tcp_client.server_hostname());
                            info!("Sending HTTP Document: {:?}", http_document.as_str());
                            match tcp_client.send_data(&http_document) {
                                Ok(result) => {
                                    defmt::info!("Data sent successfully: {:?}", result);
                                    defmt::info!("Receiving response...");

                                    match tcp_client.receive_data() {
                                        Ok(response) => {
                                            defmt::info!("{=[u8]:#X}", response);
                                            defmt::info!("{=[u8]:a}", response);
                                        }
                                        Err(e) => {
                                            defmt::info!("Error receiving data: {:?}", e);
                                        }
                                    }
                                }
                                Err(e) => {
                                    defmt::error!("Response error: {:?}", e)
                                }
                            }
                        },
                    ) {
                        error!(
                            "TCP connection to {:?}:{:?} failed: {:?}",
                            ip_address,
                            port,
                            e
                        );
                    }

                    delay.delay_ms(5000).ok().unwrap();

                } else if status == ConnectionStatus::Disconnected {
                    sleep = 20000; // No need to loop as often after disconnecting
                }
            }
            Err(e) => {
                defmt::error!("Failed to get connection result: {:?}", e);
            }
        }
   }
}

// End of file
