#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::rmt::Rmt;
use esp_hal_smartled::Ws2812SmartLeds;
use panic_rtt_target as _;
use smart_leds::{brightness, hsv::hsv2rgb, hsv::Hsv, SmartLedsWrite};

extern crate alloc;

/// Number of WS2812 LEDs in the strip.
const NUM_LEDS: usize = 150;

/// RMT buffer size: 24 bits per LED (8 per channel * 3 channels) + 1 end marker.
const BUFFER_SIZE: usize = NUM_LEDS * 24 + 1;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    info!("Initializing RMT for WS2812...");

    // Set up RMT peripheral at 80 MHz (APB clock)
    let rmt = Rmt::new(peripherals.RMT, esp_hal::time::Rate::from_mhz(80))
        .expect("failed to initialize RMT");

    // Create WS2812 driver on GPIO2 using RMT channel 0
    let mut led_driver =
        Ws2812SmartLeds::<BUFFER_SIZE, _>::new(rmt.channel0, peripherals.GPIO2)
            .expect("failed to create WS2812 driver");

    info!("WS2812 driver ready, {} LEDs configured", NUM_LEDS);

    // Simple rainbow cycle demo
    let mut hue: u8 = 0;
    loop {
        let colors = (0..NUM_LEDS).map(|i| {
            hsv2rgb(Hsv {
                hue: hue.wrapping_add((i * 256 / NUM_LEDS) as u8),
                sat: 255,
                val: 255,
            })
        });

        led_driver
            .write(brightness(colors, 32))
            .expect("failed to write LED data");

        hue = hue.wrapping_add(1);
        Timer::after(Duration::from_millis(20)).await;
    }
}
