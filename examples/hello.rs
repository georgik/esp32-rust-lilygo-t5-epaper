#![no_std]
#![no_main]

extern crate alloc;
use core::mem::MaybeUninit;
use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    Delay, IO,
};
use log::info;

use ssd1680::color::{Black, White};
use ssd1680::prelude::*;

use embedded_graphics::{
    fonts::{Font6x8, Text},
    prelude::*,
    text_style,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

fn draw_text(display: &mut Display2in13, text: &str, x: i32, y: i32) {
    let _ = Text::new(text, Point::new(x, y))
        .into_styled(text_style!(
            font = Font6x8,
            text_color = Black,
            background_color = White
        ))
        .draw(display);
}

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut delay = Delay::new(&clocks);
    // setup logger
    // To change the log_level change the env section in .config/cargo.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    info!("Logger is setup");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let busy = io.pins.gpio4.into_floating_input();
    let rst = io.pins.gpio16.into_push_pull_output();
    let mosi = io.pins.gpio23.into_push_pull_output();
    let unused_miso = io.pins.gpio19.into_floating_input();
    let unused_cs = io.pins.gpio22.into_push_pull_output();
    let sclk = io.pins.gpio18.into_push_pull_output();
    let dc = io.pins.gpio17.into_push_pull_output();
    let cs = io.pins.gpio5.into_push_pull_output();
    delay.delay_ms(10u32);

    let mut spi = Spi::new(peripherals.SPI2, 40u32.MHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(sclk),
        Some(mosi),
        Some(unused_miso),
        Some(unused_cs),
    );

    let mut ssd1680 = Ssd1680::new(&mut spi, cs, busy, dc, rst, &mut delay).unwrap();

    ssd1680.clear_bw_frame(&mut spi).unwrap();
    let mut buf_array = alloc::vec![0u8; 4000];
    for i in 0..4000 {
        buf_array[i] = 255;
    }
    let mut display_bw = Display2in13::bw_with_buffer(buf_array).unwrap();

    draw_text(&mut display_bw, "Hello ESP-RS!", 25, 25);
    println!("Hello ESP-RS!");

    ssd1680
        .update_bw_frame(&mut spi, display_bw.buffer())
        .unwrap();
    ssd1680.display_frame(&mut spi, &mut delay).unwrap();

    loop {}
}
