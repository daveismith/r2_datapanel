use core::sync::atomic::{ AtomicU32, Ordering};
use embassy_rp::gpio::Flex;
use embassy_time::Timer;

static SHARED: AtomicU32 = AtomicU32::new(0);

pub fn set_leds(val: u32) {
    SHARED.store(val, Ordering::Relaxed);
}

#[embassy_executor::task(pool_size=4)]
pub async fn led_grid(mut pin1: Flex<'static>, mut pin2: Flex<'static>, mut pin3: Flex<'static>, mut pin4: Flex<'static>, mut pin5: Flex<'static>) {

    loop {
        let val = SHARED.load(Ordering::Relaxed);

        for i in 0..=4 {
            let (base, mut l1, mut l2, mut l3, mut l4) = match i {
                0 => (&mut pin1, &mut pin2, &mut pin3, &mut pin4, &mut pin5),
                1 => (&mut pin2, &mut pin1, &mut pin3, &mut pin4, &mut pin5),
                2 => (&mut pin3, &mut pin1, &mut pin2, &mut pin4, &mut pin5),
                3 => (&mut pin4, &mut pin1, &mut pin2, &mut pin3, &mut pin5),
                4 => (&mut pin5, &mut pin1, &mut pin2, &mut pin3, &mut pin4),
                _ => (&mut pin1, &mut pin2, &mut pin3, &mut pin4, &mut pin5) // to make the compiler happy until I learn this better
            };

            base.set_low();
            base.set_as_output();
            
            for j in 0..=3 {
                l1.set_as_input();
                l2.set_as_input();
                l3.set_as_input();
                l4.set_as_input();

                let out = match j {
                    0 => &mut l1,
                    1 => &mut l2,
                    2 => &mut l3,
                    3 => &mut l4,
                    _ => &mut l1,
                };

                if (1 << ((i * 4) + j)) & val != 0 {
                    out.set_as_output();
                    out.set_high();
                }
                Timer::after_nanos(25).await;
                out.set_as_input();
            }

            Timer::after_nanos(100).await;
            base.set_as_input();
            l1.set_as_input();
            l2.set_as_input();
            l3.set_as_input();
            l4.set_as_input();
        }
    }
}