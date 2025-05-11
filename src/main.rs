#![no_std]
#![no_main]

mod can;
mod led_grid;
mod noline_async;
mod usb;

use core::cell::RefCell;
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::flash::Async;
use embassy_rp::gpio::{Flex, Level, Output};
use embassy_rp::i2c::{self, Config, I2c};
use embassy_rp::spi::{self, Spi};
use embassy_rp::peripherals::I2C0;
use embassy_rp::uart;
//use embassy_rp::rom_data::reset_to_usb_boot;

//use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
use embassy_sync::blocking_mutex::Mutex;

//use embedded_can::{ExtendedId, Frame, Id};
//use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515, Settings};

use embassy_time::{Duration, Timer};
use rand::RngCore;

use {defmt_rtt as _, panic_probe as _}; // global logger

use pwm_pca9685::{Address, Channel, Pca9685, OutputDriver};

use crc16_umts_fast;

//use embassy_rp::peripherals::PIO0;
//use fixed::traits::ToFixed;
//use fixed_macro::types::U56F8;
//use embassy_rp::pio::{Common, InterruptHandler, Irq, Pio, PioPin, ShiftDirection, StateMachine};

const FLASH_SIZE: usize = 2 * 1024 * 1024;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    //PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

use core::fmt::{self, Write};

/// A little wrapper that will hex-format its inner byte slice.
pub struct Hex<'a>(pub &'a [u8]);

impl<'a> fmt::Display for Hex<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for &b in self.0 {
            // high nibble
            let hi = b >> 4;
            let lo = b & 0x0f;
            f.write_char(nibble_to_hex(hi))?;
            f.write_char(nibble_to_hex(lo))?;
        }
        Ok(())
    }
}

fn nibble_to_hex(n: u8) -> char {
    match n {
        0..=9  => (b'0' + n) as char,
        10..=15 => (b'A' + (n - 10)) as char,
        _       => '?',
    }
}

#[embassy_executor::task(pool_size=4)]
async fn blinker(mut led: Output<'static>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

#[embassy_executor::task]
async fn servo_driver(i2c: I2c<'static, I2C0, i2c::Async>) {
    let address = Address::default();
    let mut pwm = Pca9685::new(i2c, address).unwrap();

    pwm.set_prescale(121).unwrap();
    let _ = pwm.set_output_driver(OutputDriver::TotemPole);
    pwm.enable().unwrap();

    loop {
        //for i in (1300_u16..=2100).step_by(10) {
        for i in (1300_u16..=2100).step_by(100) {
        //for i in 266_u16..=430 {
            let pw = ((i / 1000) as f32) / 0.0048828125; 
            pwm.set_channel_on_off(Channel::C0, 0, pw as u16).unwrap();
            Timer::after_millis(10).await;
        }
        Timer::after_millis(100).await;
        pwm.set_channel_full_off(Channel::C0).unwrap();
        
        Timer::after_secs(1).await;

        for i in (500_u16..=2400).step_by(100).rev() {
            let pw = ((i / 1000) as f32) / 0.0048828125; 
            pwm.set_channel_on_off(Channel::C0, 0, pw as u16).unwrap();
            Timer::after_millis(10).await;
        }
        Timer::after_millis(100).await;
        pwm.set_channel_full_off(Channel::C0).unwrap();

        Timer::after_secs(1).await;
 
    }
}

#[embassy_executor::task]
async fn data_panel(mut cs: Output<'static>, mut spi: Spi<'static, embassy_rp::peripherals::SPI1, spi::Async> ) {
    // bring out of reset
    cs.set_low();
    let tx_buf = [0x0c, 0x01];
    let mut rx_buf = [0_u8, 0];
    spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
    cs.set_high();

    Timer::after_millis(100).await;

    // decode mode out of reset
    cs.set_low();
    let tx_buf = [0x09, 0x00];
    let mut rx_buf = [0_u8, 0];
    spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
    cs.set_high();

    Timer::after_millis(100).await;

    // set scan limit
    cs.set_low();
    let tx_buf = [0x0b, 0x07];
    let mut rx_buf = [0_u8, 0];
    spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
    cs.set_high();

    // set intensity
    cs.set_low();
    let tx_buf = [0x0a, 0xff];
    let mut rx_buf = [0_u8, 0];
    spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
    cs.set_high();

    for i in 1..=8 {
        cs.set_low();
        let tx_buf = [i, 0];
        let mut rx_buf = [0_u8, 0];
        spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
        Timer::after_millis(10).await;
        cs.set_high();
    }
    
    loop {

        


        for i in 1..=8 {
            cs.set_low();
            let rand = RoscRng.next_u32();
            let tx_buf = [i, rand as u8];
            let mut rx_buf = [0_u8, 0];
            spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
            cs.set_high();
        }
        

        /*
        for i in 1..=8 {
            // turn on the digit
            cs.set_low();
            Timer::after_millis(100).await;
            let tx_buf = [i, 0xff];
            let mut rx_buf = [0_u8, 0];
            spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
            Timer::after_millis(1).await;

            cs.set_high();

            Timer::after_millis(250).await;

            cs.set_low();
            Timer::after_millis(100).await;
            let tx_buf = [i, 0];
            let mut rx_buf = [0_u8, 0];
            spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
            Timer::after_millis(1).await;
            cs.set_high();

            Timer::after_millis(50).await;

        }
        */
        
        info!("{:?}", rx_buf);
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Confiugre USB early
    unwrap!(spawner.spawn(usb::usb_handler(p.USB)));

    // LED Control
    let led = Output::new(p.PIN_25, Level::Low);
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));

    let cp1 = Flex::new(p.PIN_3);
    let cp2 = Flex::new(p.PIN_6);
    let cp3 = Flex::new(p.PIN_7);
    let cp4 = Flex::new(p.PIN_8);
    let cp5 = Flex::new(p.PIN_9);

    unwrap!(spawner.spawn(led_grid::led_grid(cp1, cp2, cp3, cp4, cp5)));

    //cp1.set_high();
    //cp2.set_low();
    //cp3.set_high();
    //cp4.set_high();
    //cp5.set_high();

    //led.set_high();
    //Timer::after(interval).await;
    //led.set_low();

    //let l1 = Output::new(p.PIN_10, Level::Low);
    //unwrap!(spawner.spawn(blinker(l1, Duration::from_millis(300))));

    //let l2 = Output::new(p.PIN_11, Level::Low);
    //unwrap!(spawner.spawn(blinker(l2, Duration::from_millis(300))));

    //let l3 = Output::new(p.PIN_12, Level::Low);
    //unwrap!(spawner.spawn(blinker(l3, Duration::from_millis(300))));
    
    Timer::after_secs(2).await;

    let mut flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH0);

    // Get JEDEC id
    let jedec = flash.blocking_jedec_id().unwrap();
    
    // Get unique id
    let mut uid = [0; 8];
    flash.blocking_unique_id(&mut uid).unwrap();
    log::info!("JEDEC: {}, UID: {}", jedec, Hex(&uid));

    //log::info!("Hello USB");

    Timer::after_millis(100).await;

    //panic!("xit app");

    // CAN is SPI0.
    // 3MHz seems to be the fastest that this runs out of the box.
    let mut config = spi::Config::default();
    config.frequency = 3_0000_000;   // 1MHz

    // Setup SPI bus
    let spi = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, config);
    let spi_bus = Mutex::new(RefCell::new(spi));
    let can_cs = Output::new(p.PIN_17, Level::High);
    let can_reset = Output::new(p.PIN_21, Level::Low);
    unwrap!(spawner.spawn(can::canbus(spi_bus, can_cs, can_reset)));

    let mut tx_en = Output::new(p.PIN_14, Level::Low);


    let uart_config = uart::Config::default();
    //let uart = uart::Uart::new(p.UART1, p.PIN_6, p.PIN_7, irq, tx_dma, rx_dma, config)
    let mut uart = uart::Uart::new_blocking(p.UART1, p.PIN_4, p.PIN_5, uart_config);
    uart.set_baudrate(1_000_000);
    
    // Write A Ping
    // let d1 = [0xff, 0xff, 0xfd, 0x00, 0xfe, 0x03, 0x00, 0x01, 0x31, 0x42];
    
    //let mut instruction = [0xff, 0xff, 0xfd, 0x00, 0x01,  0x07, 0x00, 0x03, 0x1e, 0x00, 0xff, 0x03, 0x00, 0x00];
    let mut instruction = [0xff, 0xff, 0xfd, 0x00, 0x01,  0x07, 0x00, 0x03, 0x1e, 0x00, 0x0, 0x0, 0x00, 0x00];
    //let mut instruction = [0xff, 0xff, 0xfd, 0x00, 0xfe, 0x03, 0x00, 0x01, 0x31, 0x42];
    let crc = crc16_umts_fast::hash(&instruction[0..instruction.len()-2]);
    instruction[instruction.len() - 2] = (crc & 0xff) as u8;
    instruction[instruction.len() - 1] = ((crc >> 8) & 0xff) as u8;

    //log::info!("crc is {:x}", crc);
    //tx_en.set_low();
    tx_en.set_high();
    uart.blocking_write(&instruction).unwrap();
    Timer::after_micros(150).await;
    tx_en.set_low();

    Timer::after_millis(100).await;

    let mut read_buf = [0u8; 9];
    if uart.blocking_read(&mut read_buf).is_ok() {
        log::info!("contents of read are {}", read_buf[0]);
    } else {
        log::error!("failed to read from UART");
    }


    /*
    // Data Panel LEDs
    let miso = p.PIN_12;    // not actually

    let mosi = p.PIN_11;
    let clk = p.PIN_10;
    
    let mut config = spi::Config::default();
    //config.phase = spi::Phase::CaptureOnFirstTransition;
    //config.polarity = spi::Polarity::IdleLow;
    config.frequency = 500_000;   // 1KHz

    let spi: Spi<'_, embassy_rp::peripherals::SPI1, spi::Async> = Spi::new(p.SPI1, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, config);
    //let cs = Output::new(p.PIN_3, Level::Low);
    let cs = Output::new(p.PIN_9, Level::Low);
    unwrap!(spawner.spawn(data_panel(cs, spi)));

    let scl = p.PIN_1;
    let sda = p.PIN_0;


    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());
    //let i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());
    unwrap!(spawner.spawn(servo_driver(i2c)));
*/
    let mut counter = 0 as u32;
    loop {
        counter += 1;
        //log::info!("Tick {}", counter);
        //log::info!("jedec id: 0x{:x}", jedec);
        //log::info!("unique id: {:?}", uid);

        let rand = RoscRng.next_u32();
        
        let mut instruction = [0xff, 0xff, 0xfd, 0x00, 0x01,  0x07, 0x00, 0x03, 0x1e, 0x00, (counter << 2) as u8, ((counter >> 6) & 0x03) as u8, 0x00, 0x00];
        //let mut instruction = [0xff, 0xff, 0xfd, 0x00, 0xfe, 0x03, 0x00, 0x01, 0x31, 0x42];
        let crc = crc16_umts_fast::hash(&instruction[0..instruction.len()-2]);
        instruction[instruction.len() - 2] = (crc & 0xff) as u8;
        instruction[instruction.len() - 1] = ((crc >> 8) & 0xff) as u8;
    
        //log::info!("crc is {:x}", crc);
        //tx_en.set_low();
        tx_en.set_high();
        uart.blocking_write(&instruction).unwrap();
        Timer::after_micros(150).await;
        tx_en.set_low();

        Timer::after_millis(100).await;

        let mut read_buf = [0u8; 11];
        if uart.blocking_read(&mut read_buf).is_ok() {
            //log::info!("contents of read are {}", read_buf[0]);
        } else {
            log::error!("failed to read from UART");
        }
    
        /*
        instruction[instruction.len() - 4] = ((counter % 1024) & 0xff) as u8;
        instruction[instruction.len() - 3] = (((counter % 1024) >> 8) & 0xff) as u8;

        let crc = crc16_umts_fast::hash(&instruction[0..instruction.len()-2]);
        instruction[instruction.len() - 2] = (crc & 0xff) as u8;
        instruction[instruction.len() - 1] = ((crc >> 8) & 0xff) as u8;

        tx_en.set_high();
        uart.blocking_write(&instruction).unwrap();
        Timer::after_micros(150).await;
        tx_en.set_low();
         */

        //let rand = (1 << (counter % 20));
        //SHARED.store(rand % 1048576, Ordering::Relaxed);
        led_grid::set_leds(rand % 1048576);

        //Timer::after_secs(1).await;
        Timer::after_millis(250).await;
    }
}