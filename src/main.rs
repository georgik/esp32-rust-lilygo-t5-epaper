#![no_std]
#![no_main]
use embedded_hal_1::spi::SpiDevice;
use esp32_hal::{clock::{ClockControl, CpuClock}, pac::Peripherals, prelude::*, spi::{ Spi, SpiBusController, SpiMode}, timer::TimerGroup, Delay, Rtc, IO};
use esp_backtrace as _;

use epd_waveshare::{prelude::*, color::*, epd2in13_v2::{Display2in13, Epd2in13}};

use esp_println::println;

use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    text::{Baseline, Text, TextStyleBuilder},
};



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
    // https://github.com/Xinyuan-LilyGO/LilyGo-T5-Epaper-Series/blob/master/lib/GxEPD/src/boards.h
    //#define EPD_MOSI                (23)
// #define EPD_MISO                (-1)
// #define EPD_SCLK                (18)
// #define EPD_CS                  (5)

// #define EPD_BUSY                (4)
// #define EPD_RSET                (16)
// #define EPD_DC                  (17)

// #define SDCARD_CS               (13)
// #define SDCARD_MOSI             (15)
// #define SDCARD_MISO             (2)
// #define SDCARD_SCLK             (14)

// #define BUTTON_1                (39)
// #define BUTTONS                 {39}

// #define BUTTON_COUNT            (1)

// #define LED_PIN                 (19)
// #define LED_ON                  (LOW)

// #define ADC_PIN                 (35)

// #define _HAS_ADC_DETECTED_
// #define _HAS_LED_
// #define _HAS_SDCARD_
    delay.delay_ms(10u32);

    let miso = io.pins.gpio2.into_floating_input();
    let cs = io.pins.gpio5.into_push_pull_output();
    let mut rst = io.pins.gpio16.into_push_pull_output();
    let busy = io.pins.gpio4.into_floating_input();
    let dc = io.pins.gpio17.into_push_pull_output();

    // let mut spi = spi::Spi::new_no_cs_no_miso(
    let spi_controller = SpiBusController::from_spi(Spi::new_no_cs(
        peripherals.SPI3, // Real HW working with SPI2, but Wokwi seems to work only with SPI3
        miso,
        io.pins.gpio18,   // SCLK
        io.pins.gpio23,   // MOSI
        4u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    ));
    let mut eink_device = spi_controller.add_device(cs);

    println!("Setup");
    rst.set_low().unwrap();
    delay.delay_ms(10u32);
    rst.set_high().unwrap();
    delay.delay_ms(10u32);
    while busy.is_high().unwrap() {
        delay.delay_ms(10u32);
    }


//https://github.com/Xinyuan-LilyGO/LilyGo-T5-Epaper-Series/blob/master/lib/GxEPD/src/GxGDEH0213B72/GxGDEH0213B72.cpp

    println!("Init");
    let write = [0x12];
    let mut read: [u8; 4] = [0x00u8; 4];
    eink_device.transfer(&mut read[..], &write[..]).unwrap();
    while busy.is_high().unwrap() {
        delay.delay_ms(10u32);
    }

    // case 0x03: // x increase, y increase : normal mode
    //     _SetRamArea(0x00, xPixelsPar / 8, 0x00, 0x00, yPixelsPar % 256, yPixelsPar / 256);  // X-source area,Y-gate area
    //     _SetRamPointer(0x00, 0x00, 0x00); // set ram
    //     break;

    println!("_setRamDataEntryMode");
    // _setRamDataEntryMode
    // spi.write(&[0x11, 0x03]).unwrap();
    let write = [0x11, 0x03];
    let mut read: [u8; 4] = [0x00u8; 4];
    eink_device.transfer(&mut read[..], &write[..]).unwrap();

    println!("_SetRamArea");
    // _SetRamArea
    let x_pixels_par:u8 = 212 - 1;
    let y_pixels_par:u8 = 104 - 1;
    let x_start:u8 = 0;
    let x_end = x_pixels_par / 8;
    let y_start:u8 = 0x00;
    let y_start1:u8 = 0x00;
    let y_end = y_pixels_par;// % 256;
    let y_end1 = y_pixels_par;// / 256;
    let write = [0x44, x_start + 1, x_end + 1, 0x45, y_start, y_start1, y_end, y_end1];
    eink_device.transfer(&mut read[..], &write[..]).unwrap();

    // spi.write_data(&[0x00]).unwrap();
//     void GxDEPG0213BN::_SetRamArea(uint8_t Xstart, uint8_t Xend, uint8_t Ystart, uint8_t Ystart1, uint8_t Yend, uint8_t Yend1)
// {
//     _writeCommand(0x44);
//     _writeData(Xstart + 1);
//     _writeData(Xend + 1);
//     _writeCommand(0x45);
//     _writeData(Ystart);
//     _writeData(Ystart1);
//     _writeData(Yend);
//     _writeData(Yend1);
// }

// void GxDEPG0213BN::_SetRamPointer(uint8_t addrX, uint8_t addrY, uint8_t addrY1)
// {
//     _writeCommand(0x4e);
//     _writeData(addrX + 1);
//     _writeCommand(0x4f);
//     _writeData(addrY);
//     _writeData(addrY1);
// }
println!("_SetRamPointer");

let write = [0x4e, 0x01, 0x4f, 0x00, 0x00];
    eink_device.transfer(&mut read[..], &write[..]).unwrap();



    // void GxDEPG0213BN::update(void)
    // {
    //     if (_current_page != -1) return;
    //     _using_partial_mode = false;
    //     _Init_Full(0x03);
    //     _writeCommand(0x24);
    //     for (uint16_t y = 0; y < GxDEPG0213BN_HEIGHT; y++) {
    //         for (uint16_t x = 0; x < GxDEPG0213BN_WIDTH / 8; x++) {
    //             uint16_t idx = y * (GxDEPG0213BN_WIDTH / 8) + x;
    //             uint8_t data = (idx < sizeof(_buffer)) ? _buffer[idx] : 0x00;
    //             _writeData(~data);
    //         }
    //     }
    //     _Update_Full();
    //     _PowerOff();
    // }
    
    println!("update");
    // spi.write(&[0x24]).unwrap();
    let write = [0x24];
    eink_device.transfer(&mut read[..], &write[..]).unwrap();

    let write = &[0xff;212*104];
    eink_device.transfer(&mut read[..], &write[..]).unwrap();



//     void GxDEPG0213BN::_Update_Full(void)
// {
//     _writeCommand(0x20);
//     _waitWhileBusy("_Update_Full", full_refresh_time);
// }
let write = [0x20];
eink_device.transfer(&mut read[..], &write[..]).unwrap();

println!("Wait for operation to finish");
    // spi.write(&[0x20]).unwrap();
    while busy.is_high().unwrap() {
        delay.delay_ms(100u32);
    }

    println!("Done");
    // let mut epd = Epd2in13::new(&mut spi, cs.into_push_pull_output(), busy, dc, rst, &mut delay).unwrap();
    // let mut display = Display2in13::default();
    // display.set_rotation(DisplayRotation::Rotate90);

    // draw_text(&mut display, " Hello Rust from ESP32! ", 15, 50);
    // epd.update_and_display_frame(&mut spi, display.buffer(), &mut delay).expect("Frame cannot be cleared and updated!");

    loop {}
}
