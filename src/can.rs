use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_rp::gpio::Output;
use embassy_rp::spi::{self, Spi};
use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
use embassy_time::{Delay, Timer};
use embedded_can::{ExtendedId, Frame, Id};
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515, Settings};


// SpiDevice<'static, NoopRawMutex, Spi<'static, embassy_rp::peripherals::SPI0, spi::Blocking>, Output<'static>>
#[embassy_executor::task]
pub async fn canbus(spi_bus: Mutex<NoopRawMutex, RefCell<Spi<'static, embassy_rp::peripherals::SPI0, spi::Blocking>>>, cs: Output<'static>, mut reset: Output<'static>) {
    //let spi_device = ExclusiveDevice::new_no_delay(spi, spi_cs);
    let can_spi = SpiDevice::new(&spi_bus, cs);
    let mut can = MCP2515::new(can_spi);

    let mut delay = Delay;

    log::info!("Starting canbus");

    // Reset The Device
    reset.set_low();
    Timer::after_millis(10).await;
    reset.set_high();

    // Initialize The Device
    match can.init(
        &mut delay,
        Settings {
            mode: OpMode::Loopback,         // Loopback for testing and example
            can_speed: CanSpeed::Kbps1000,  // Many options supported.
            mcp_speed: McpSpeed::MHz16,     // Currently 16MHz and 8MHz chips are supported.
            clkout_en: false,
        },
    ) {
        Ok(_) => log::info!("Initialized Successfully"),
        Err(_) => panic!("Failed to initialize CAN"),
    }

    let mut counter = 0;

    loop {
        // Send a message
        if counter % 4 == 0 {
            let frame = CanFrame::new(
                Id::Extended(ExtendedId::MAX),
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            )
            .unwrap();
            can.send_message(frame).unwrap();
            log::info!("Sent message!");
        }

        // Read the message back (we are in loopback mode)
        match can.read_message() {
            Ok(frame) => log::info!("Received frame {:?}", frame),
            Err(Error::NoMessage) => log::info!("No message to read!"),
            Err(_) => panic!("Oh no!"),
        }

        //delay.delay_ms(500);
        Timer::after_millis(500).await;
        counter += 1;

    }

    /*
    let mut counter = 0;
    loop {
        counter += 1;
        log::info!("SPI Tick {}", counter);

        Timer::after_millis(500).await;
    }
    */
}