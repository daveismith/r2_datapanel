// USB interface configuration & setup. This will create two endpoints. One is
// an application endpoint which contains the USB serial which we use for 
// application communications and the second is the log output endpoint.
//
// See https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_serial_with_logger.rs
// for the basis of this file.

use defmt::panic;
use embassy_futures::join::join3;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::InterruptHandler;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    driver::EndpointError
};
use embassy_usb::{Builder, Config};

use crate::noline_async::cli;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

//const USB_CDC_SUBCLASS_ACM: u8 = 0x02;
//const USB_CDC_PROTOCOL_AT: u8 = 0x01;

const BUF_SIZE_DESCRIPTOR: usize = 256;
const BUF_SIZE_CONTROL: usize = 64;
const MAX_PACKET_SIZE: usize = 64;

//static SEND: Forever<Mutex<NoopRawMutex, Sender<'static, Driver<'static, USB>>>> = Forever::new();
//static RECV: Forever<Mutex<NoopRawMutex, Receiver<'static, Driver<'static, USB>>>> = Forever::new();
//static RECV: Forever<Mutex<NoopRawMutex, Receiver<'static, Driver<'static, USB>>>> = Forever::new();


#[embassy_executor::task]
pub async fn usb_handler(usb: USB) {
    // Create the driver, from the HAL.
    let driver = embassy_rp::usb::Driver::new(usb, Irqs);

    // Create embassy-usb Config
    let config = {
        let mut config = Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB-serial example");
        config.serial_number = Some("TEST");
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        //config.device_class = USB_CLASS_CDC;
        //config.device_sub_class = USB_CDC_SUBCLASS_ACM;
        //config.device_protocol = USB_CDC_PROTOCOL_AT;
        config
    };

        // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; BUF_SIZE_DESCRIPTOR];
    let mut bos_descriptor = [0; BUF_SIZE_DESCRIPTOR];
    let mut msos_descriptor = [0; BUF_SIZE_DESCRIPTOR];
    let mut control_buf = [0; BUF_SIZE_CONTROL];
    
    let mut logger_state = State::new();
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let (mut send, mut recv, mut control, log_fut, mut usb_dev) = {
        // This setups up the various handlers and has to happen inside of this context to declare things with sufficient lifetime &
        // with the right orders.
        
        // Set Up Handling for Serial
        // Create The Serial Class for the CLI. 
        let serial = CdcAcmClass::new(&mut builder, &mut state, MAX_PACKET_SIZE as u16);
        let (send, recv, control) = serial.split_with_control();        

        // Create a class for the logger
        let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, MAX_PACKET_SIZE as u16);

        // Creates the logger and returns the logger future
        // Note: You'll need to use log::info! afterwards instead of info! for this to work (this also applies to all the other log::* macros)
        let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);

        let usb_dev = builder.build();

        (send, recv, control, log_fut, usb_dev)
    };

    let noline_fut = cli(&mut send, &mut recv, &mut control);

    // Run the USB device.
    let usb_fut = usb_dev.run();

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join3(usb_fut, log_fut, noline_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}
