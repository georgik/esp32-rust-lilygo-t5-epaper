#![no_std]
#![no_main]

extern crate alloc;
use core::mem::MaybeUninit;
use hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::TimerGroup,
    Delay, Rng, IO,
};

// use embedded_io::blocking::*;
use embedded_svc::io::{Read, Write};
use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{AccessPointInfo, ClientConfiguration, Configuration, Wifi};

use esp_backtrace as _;
use esp_println::{print, println};
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::{WifiError, WifiStaDevice};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize, EspWifiInitFor};

use smoltcp::iface::SocketStorage;
use smoltcp::wire::IpAddress;
use smoltcp::wire::Ipv4Address;

use embedded_graphics::{
    fonts::{Font24x32, Text},
    prelude::*,
    text_style,
};

use ssd1680::color::{Black, White};
use ssd1680::prelude::*;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

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
            font = Font24x32,
            text_color = Black,
            background_color = White
        ))
        .draw(display);
}

#[entry]
fn main() -> ! {
    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut buffer = [0u8; 512];
    let mut socket_set_entries: [SocketStorage; 5] = Default::default();

    init_heap();
    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer = TimerGroup::new(peripherals.TIMG1, &clocks).timer0;

    let rng = Rng::new(peripherals.RNG);
    let radio_clock_control = system.radio_clock_control;

    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        rng,
        radio_clock_control,
        &clocks,
    )
    .unwrap();

    // Create an SPI interface and pins
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);
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
    // Initialize ePaper display
    ssd1680.clear_bw_frame(&mut spi).unwrap();

    let mut buf_array = alloc::vec![0u8; 4000];

    for i in 0..4000 {
        buf_array[i] = 255;
    }
    println!("Creating display_bw");
    let mut display_bw = Display2in13::bw_with_buffer(buf_array).unwrap();
    println!("Drawing text");

    println!("Initializing");

    // Initialize WiFi
    println!("Acquiring WiFi interface");
    let wifi = peripherals.WIFI;
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();

    println!("Creating WifiStack");
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    // let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);
    println!("Creating ClientConfiguration");
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    println!("Setting configuration");
    controller.set_configuration(&client_config).unwrap();
    println!("Starting WiFi controller");
    match controller.start() {
        Ok(_) => println!("WiFi controller started"),
        Err(e) => println!("WiFi controller error {:?}", e),
    }

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
                println!("WiFi Error - {:?} - delay 1000 ms", err);
                delay.delay_ms(1000u32);
                controller.connect().unwrap();
            }
        }
    }
    println!("Is connected: {:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();
        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            use core::fmt::Write as FmtWrite;
            let mut ip_addr: heapless::String<256> = heapless::String::new();
            let bytes = wifi_stack.get_ip_info().unwrap().ip.octets();
            write!(
                ip_addr,
                "{}.{}.{}.{}",
                bytes[0], bytes[1], bytes[2], bytes[3]
            )
            .unwrap();
            break;
        }
    }

    println!("Start busy loop on main");

    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    loop {
        println!("Making HTTP request");
        socket.work();
        println!("Opening socket");

        match socket.open(IpAddress::Ipv4(Ipv4Address::new(93, 99, 115, 9)), 80) {
            Ok(_) => {
                // Successfully opened the socket
                // Continue with the next steps
            }
            Err(e) => {
                println!("error{:?}", e);
                loop {}
            }
        }

        println!("Sendig GET request");
        socket
            .write(b"GET /info.txt HTTP/1.0\r\nHost: iot.georgik.rocks\r\n\r\n")
            .unwrap();
        socket.flush().unwrap();

        let wait_end = current_millis() + 20 * 1000;
        // A fixed-size buffer to hold the HTTP response.
        let mut full_response = [0u8; 2048];
        let mut full_len = 0;

        println!("Reading response");

        // Read the HTTP response into the buffer.
        loop {
            if let Ok(len) = socket.read(&mut buffer) {
                // Copy the newly read bytes into `full_response`.
                full_response[full_len..full_len + len].copy_from_slice(&buffer[0..len]);
                full_len += len;
                println!("Read {} bytes", len);
            } else {
                println!("Read error");
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
            draw_text(&mut display_bw, &to_print, 0, 10); // Assuming draw_text function is defined
            print!("{}", to_print);
        }

        println!("Closing socket");

        socket.disconnect();

        println!("Updating frame");
        ssd1680
            .update_bw_frame(&mut spi, display_bw.buffer())
            .unwrap();
        println!("Updating display");
        ssd1680.display_frame(&mut spi, &mut delay).unwrap();

        println!("Sleeping");
        // Delay before repeating
        delay.delay_ms(500000u32);
        println!("Waking up");
    }
}
