#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    IO,
    spi,
    Delay
};
use log::info;

use ssd1680::prelude::*;
use ssd1680::color::{Black, White, Red};


use embedded_graphics::{
    fonts::{Font6x8, Text},
    prelude::*,
    // primitives::{Circle, Line, Rectangle},
    style::PrimitiveStyle,
    text_style,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
        static mut _heap_end: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        let heap_end = &_heap_end as *const _ as usize;
        assert!(
            heap_end - heap_start > HEAP_SIZE,
            "Not enough available heap memory."
        );
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
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
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);
    // setup logger
    // To change the log_level change the env section in .config/cargo.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    info!("Logger is setup");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let busy = io.pins.gpio4.into_floating_input();
    let mut rst = io.pins.gpio16.into_push_pull_output();
    let mosi = io.pins.gpio23.into_push_pull_output();
    let miso = io.pins.gpio19.into_floating_input();
    let mut sclk = io.pins.gpio18.into_push_pull_output();
    let dc = io.pins.gpio17.into_push_pull_output();
    let mut cs = io.pins.gpio5.into_push_pull_output();
    delay.delay_ms(10u32);

    let mut spi = spi::Spi::new_no_cs_no_miso(
        peripherals.SPI3,
        sclk,
        mosi,
        4u32.MHz(),
        spi::SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut ssd1680 = Ssd1680::new(&mut spi, cs, busy, dc, rst, &mut delay).unwrap();

    ssd1680.clear_bw_frame(&mut spi).unwrap();
    let mut display_bw = Display2in13::bw();

    draw_text(&mut display_bw, "Hello ESP-RS!", 25, 25);
    println!("Hello ESP-RS!");

    ssd1680.update_bw_frame(&mut spi, display_bw.buffer()).unwrap();
    ssd1680.display_frame(&mut spi, &mut delay).unwrap();

    loop {}
}
