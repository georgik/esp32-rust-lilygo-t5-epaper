#![no_std]
#![no_main]

// extern crate alloc;

use hal::{psram, prelude::*, peripherals::Peripherals, spi, timer::TimerGroup, clock::{ClockControl, CpuClock}, Delay, Rtc, Rng, IO};

use embedded_io::blocking::*;
use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{AccessPointInfo, ClientConfiguration, Configuration, Wifi};

use esp_backtrace as _;
use esp_println::{print, println};
use esp_println::println as esp_println;
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::{WifiError, WifiMode};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize, EspWifiInitFor};


// use embedded_svc::http::client::{HttpClient, HttpClientBuilder};
// use embedded_svc::http::method::Method;
// use embedded_svc::http::request::Request;
// use embedded_svc::http::response::Response;

// use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*};
// use hal::Rng;

use smoltcp::iface::SocketStorage;
use smoltcp::wire::IpAddress;
use smoltcp::wire::Ipv4Address;


use embedded_graphics::{
    fonts::{Font6x8, Text},
    prelude::*,
    style::PrimitiveStyle,
    text_style,
};

use ssd1680::prelude::*;
use ssd1680::color::{Black, White};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");


// #[global_allocator]
// static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

// fn init_heap() {
//     const HEAP_SIZE: usize = 32 * 1024;
//     extern "C" {
//         static mut _heap_start: u32;
//         static mut _heap_end: u32;
//     }
//     unsafe {
//         let heap_start = &_heap_start as *const _ as usize;
//         let heap_end = &_heap_end as *const _ as usize;
//         assert!(
//             heap_end - heap_start > HEAP_SIZE,
//             "Not enough available heap memory."
//         );
//         ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
//     }
// }

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
    // Initialize heap and other system resources
    // init_heap();
    let peripherals = Peripherals::take();

    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let timer = timer_group1.timer0;
    

    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    esp_println!("Logger is setup");
    // Initialize WiFi
    let (wifi, ..) = peripherals.RADIO.split();
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiMode::Sta, &mut socket_set_entries).unwrap();
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });

    controller.set_configuration(&client_config).unwrap();
    controller.start().unwrap();

    // Assuming you have imported the required crates and modules
    use hal::spi::Spi;
    use embedded_hal::blocking::delay::DelayMs;

    // Create an SPI interface and pins
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);

    let busy = io.pins.gpio4.into_floating_input();
    let rst = io.pins.gpio16.into_push_pull_output();
    let mosi = io.pins.gpio23.into_push_pull_output();
    // let miso = io.pins.gpio19.into_floating_input();
    let sclk = io.pins.gpio18.into_push_pull_output();
    let dc = io.pins.gpio17.into_push_pull_output();
    let cs = io.pins.gpio5.into_push_pull_output();
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
    // Initialize ePaper display
    ssd1680.clear_bw_frame(&mut spi).unwrap();
    let mut display_bw = Display2in13::bw();

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    println!("{:?}", controller.get_capabilities());
    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    loop {
        println!("Making HTTP request");
        socket.work();

        socket
            .open(IpAddress::Ipv4(Ipv4Address::new(93, 99, 115, 9)), 80)
            .unwrap();

        socket
            .write(b"GET /info.txt HTTP/1.0\r\nHost: iot.georgik.rocks\r\n\r\n")
            .unwrap();
        socket.flush().unwrap();

        let wait_end = current_millis() + 20 * 1000;
        // A fixed-size buffer to hold the HTTP response.
        let mut full_response = [0u8; 2048];
        let mut full_len = 0;

        // Read the HTTP response into the buffer.
        loop {
            let mut buffer = [0u8; 512];
            if let Ok(len) = socket.read(&mut buffer) {
                // Copy the newly read bytes into `full_response`.
                full_response[full_len..full_len + len].copy_from_slice(&buffer[0..len]);
                full_len += len;
            } else {
                break;
            }

            if current_millis() > wait_end {
                println!("Timeout");
                break;
            }
        }

        // Search for the double CRLF sequence to find the start of the HTTP body.
        let mut body_start = 0;
        for i in 0..(full_len - 3) {
            if &full_response[i..i + 4] == b"\r\n\r\n" {
                body_start = i + 4;
                break;
            }
        }

        if body_start != 0 {
            let body = &full_response[body_start..full_len];
            let to_print = unsafe { core::str::from_utf8_unchecked(body) };
            draw_text(&mut display_bw, &to_print, 0, 0); // Assuming draw_text function is defined
            print!("{}", to_print);
        }

        println!();

        socket.disconnect();

        ssd1680.update_bw_frame(&mut spi, display_bw.buffer()).unwrap();
        ssd1680.display_frame(&mut spi, &mut delay).unwrap();


        // Update the frame buffer with the black and white content
        ssd1680.update_bw_frame(&mut spi, display_bw.buffer()).unwrap();

        // Display the frame on the ePaper screen
        ssd1680.display_frame(&mut spi, &mut delay).unwrap();



        // Delay before repeating
        delay.delay_ms(500000u32);
    }
}
