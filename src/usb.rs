// USB interface configuration & setup. This will create two endpoints. One is
// an application endpoint which contains the USB serial which we use for 
// application communications and the second is the log output endpoint.
//
// See https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_serial_with_logger.rs
// for the basis of this file.

use defmt::panic;
use embassy_futures::join::{join, join3, join4, join5};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
//use embassy_time::Timer;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State, ControlChanged, Receiver, Sender},
    driver::EndpointError
};
use embassy_usb::{Builder, Config};
use embassy_usb_logger::UsbLogger;

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

    let mut state = State::new();
    let mut logger_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create The Serial Class for the CLI. 
    let mut serial = CdcAcmClass::new(&mut builder, &mut state, MAX_PACKET_SIZE as u16);

    // Create a class for the logger
    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, MAX_PACKET_SIZE as u16);

    // Creates the logger and returns the logger future
    // Note: You'll need to use log::info! afterwards instead of info! for this to work (this also applies to all the other log::* macros)
    let log_fut
     = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);
    
    // Set Up Handling for Serial
    let (mut send, mut recv, mut control) = serial.split_with_control();        
    //let (mut send, mut recv) = serial.split();

    let mut rx_pipe = Pipe::<CriticalSectionRawMutex, MAX_PACKET_SIZE>::new();
    let mut tx_pipe = Pipe::<CriticalSectionRawMutex, MAX_PACKET_SIZE>::new();

    let (rx_pipe_reader, rx_pipe_writer) = rx_pipe.split();
    let (tx_pipe_reader, tx_pipe_writer) = tx_pipe.split();

    
    let usb_reader_fut = async move {
        recv.wait_connection().await;
        loop {
            let mut buf: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];
            let len = recv.read_packet(&mut buf).await.unwrap();
            rx_pipe_writer.write(&buf[0 .. len]).await;
        }
    };

    let usb_writer_fut = async {
        send.wait_connection().await;
        loop {
            let mut buf: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];
            let len = tx_pipe_reader.read(&mut buf).await;
            send.write_packet(&mut buf[0..len]).await.unwrap();
        }
    };

    let echo_fut = async {
        loop {
            let mut buf: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            let len = rx_pipe_reader.read(&mut buf).await;
            for i in buf.iter().take(len) {
                log::info!("data: {:x}", i);
            }
            tx_pipe_writer.write(&buf).await;

        }
    };

 
    /*let noline_fut = async {
        serial.wait_connection().await;
        cli(&mut send, &mut recv, &mut control).await;
    };*/
    
    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join5(usb_fut, log_fut, echo_fut, usb_reader_fut, usb_writer_fut).await;
    //join(usb_fut, join(log_fut, noline_fut)).await;
    
    //join(usb_fut, noline_fut).await;
    
    //Figure out how our CLI, which should work with the pipe reader & writer
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

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        for i in data.iter().take(n) {
            log::info!("data: {:x}", i);
        }
        class.write_packet(data).await?;
    }
}
