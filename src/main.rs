#![no_std]
#![no_main]
use embedded_hal_1::spi::SpiDevice;
use esp32_hal::{clock::{ClockControl, CpuClock}, pac::Peripherals, prelude::*, spi::{ Spi, SpiBusController, SpiMode}, timer::TimerGroup, Delay, Rtc, IO};
use esp_backtrace as _;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle},
    text::{Baseline, Text, TextStyleBuilder}, pixelcolor::BinaryColor,
};
use esp_println::println;
use epd_waveshare::graphics::Display;
use epd_waveshare::{
    color::*,
    graphics::DisplayRotation,
};
use epd_waveshare::epd2in13bc::{Display2in13bc, Epd2in13bc};
use epd_waveshare::prelude::WaveshareDisplay;

fn draw_text(display: &mut Display2in13bc, text: &str, x: i32, y: i32) {
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_6X10)
        .text_color(Color::Black)
        .background_color(Color::White)
        .build();

    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style(text, Point::new(x, y), style, text_style).draw(display);
}

#[xtensa_lx_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.DPORT.split();
    // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);


    // Configuration based on https://github.com/Xinyuan-LilyGO/LilyGo-T5-Epaper-Series/blob/master/schematic/T5V2.3.pdf
    // Flip the board to read values from the back
    // Some reference values
    // https://github.com/Xinyuan-LilyGO/LilyGo-T5-Epaper-Series/blob/master/lib/GxEPD/src/boards.h
    // C++ implementation
    // https://github.com/Xinyuan-LilyGO/LilyGo-T5-Epaper-Series/blob/master/lib/GxEPD/src/GxGDEH0213B72/GxGDEH0213B72.cpp
    // Python implementation
    // https://github.com/Inqbus/micropython_DEPG0213BN/blob/main/DEPG0213BN.py

    delay.delay_ms(10u32);

    let busy = io.pins.gpio4.into_floating_input();
    let mut rst = io.pins.gpio16.into_push_pull_output();
    let mosi = io.pins.gpio23.into_push_pull_output();
    let miso = io.pins.gpio19.into_floating_input();
    let mut sck = io.pins.gpio18.into_push_pull_output();
    let dc = io.pins.gpio17.into_push_pull_output();
    let mut cs = io.pins.gpio5.into_push_pull_output();

    // let spi_controller = SpiBusController::from_spi(Spi::new_no_cs(
    //     peripherals.SPI3, // Real HW working with SPI2, but Wokwi seems to work only with SPI3
    //     sck,
    //     mosi,
    //     miso,
    //     4u32.MHz(),
    //     SpiMode::Mode0,
    //     &mut system.peripheral_clock_control,
    //     &clocks,
    // ));
    // let mut eink_device = spi_controller.add_device(cs);
    let mut spi = Spi::new_no_cs_no_miso(
    // let spi_controller = SpiBusController::from_spi(Spi::new_no_cs(
        peripherals.SPI3, // Real HW working with SPI2, but Wokwi seems to work only with SPI3
        sck,
        mosi,
        4u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );
    println!("Init");
    let mut epd = Epd2in13bc::new( & mut spi, cs, busy, dc, rst, & mut delay, None).unwrap();
    let mut display = Display2in13bc::default();
    display.set_rotation(DisplayRotation::Rotate0);
    
    println!("Clear");
    display.clear(Color::White).ok();
//     println!("Draw");
    draw_text(&mut display, "Hello Espressif!", 5, 50);
//     // display.clear(Color::White).ok();
//         // Transfer the frame data to the epd and display it
    println!("Update");
    epd.update_and_display_frame( & mut spi, & display.buffer(), &mut delay).unwrap();
    println!("Done");
    
    // println!("Setup");
    // rst.set_low().unwrap();
    // delay.delay_ms(10u32);
    // rst.set_high().unwrap();
    // delay.delay_ms(10u32);
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(10u32);
    // }

    // println!("Init");
    // let write = [0x12];
    // let mut read: [u8; 4] = [0x00u8; 4];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(10u32);
    // }

    // println!("Set RAM Data Entry Mode");
    // let write = [0x11];
    // let mut read: [u8; 4] = [0x00u8; 4];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(10u32);
    // }

    // let write = [0x03];
    // let mut read: [u8; 4] = [0x00u8; 4];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();


    // println!("Set RAM area");
    // let x_pixels_par:u8 = 212 - 1;
    // let y_pixels_par:u8 = 104 - 1;
    // let x_start:u8 = 0;
    // let x_end = x_pixels_par / 8;
    // let y_start:u8 = 0x00;
    // let y_start1:u8 = 0x00;
    // let y_end = y_pixels_par;// % 256;
    // let y_end1 = y_pixels_par;// / 256;
    // let write = [0x44];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(10u32);
    // }

    // let write = [x_start + 1, x_end + 1, ];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();

    // let write = [0x45];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(10u32);
    // }

    // let write = [y_start, y_start1, y_end, y_end1];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();





    // println!("Set RAM pointer");

    // let write = [0x4e];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(10u32);
    // }

    // let write = [0x01];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();


    // let write = [0x4f];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(10u32);
    // }

    // let write = [0x00, 0x00];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();





    // // let write = [0x74, 0x54, 0x7E, 0x3B, 0x01, 0xF9, 0x00, 0x00, 0x3C, 0x03, 0x2C, 0x70, 0x03, 0x15, 0x04, 0x41, 0xA8, 0x32, 0x3A, 0x30, 0x3B, 0x0A];
    // // eink_device.transfer(&mut read[..], &write[..]).unwrap();



    // let write = [0x32];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();

    // println!("LUT Data");
    // let write:[u8;70]= [
    //     0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, //LUT0: BB:     VS 0 ~7
    //     0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, //LUT1: BW:     VS 0 ~7
    //     0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, //LUT2: WB:     VS 0 ~7
    //     0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, //LUT3: WW:     VS 0 ~7
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT4: VCOM:   VS 0 ~7
    //     0x03, 0x03, 0x00, 0x00, 0x02, // TP0 A~D RP0
    //     0x09, 0x09, 0x00, 0x00, 0x02, // TP1 A~D RP1
    //     0x03, 0x03, 0x00, 0x00, 0x02, // TP2 A~D RP2
    //     0x00, 0x00, 0x00, 0x00, 0x00, // TP3 A~D RP3
    //     0x00, 0x00, 0x00, 0x00, 0x00, // TP4 A~D RP4
    //     0x00, 0x00, 0x00, 0x00, 0x00, // TP5 A~D RP5
    //     0x00, 0x00, 0x00, 0x00, 0x00, // TP6 A~D RP6
    // ];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();

    // println!("Power On");
    // let write = [0x22, 0xc0, 0x20];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(100u32);
    // }

    // println!("update");
    // let write = [0x24];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(100u32);
    // }

    // let write = &[0xff;212*104];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();

    // let write = [0x22, 0xc4, 0x20];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();

    // println!("Wait for operation to finish");
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(100u32);
    // }

    // println!("Power off");

    // let write = [0x22, 0xc3, 0x20];
    // eink_device.transfer(&mut read[..], &write[..]).unwrap();
    // while busy.is_high().unwrap() {
    //     delay.delay_ms(100u32);
    // }

    // println!("Done");
    loop {}
}
