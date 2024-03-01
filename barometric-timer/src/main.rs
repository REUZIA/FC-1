#![no_std]
#![no_main]

mod rp_2040_lib;

use core::iter::once;
use embedded_hal::timer::CountDown;
use embedded_hal::{digital::v2::InputPin, PwmPin};
use fugit::{ExtU32, RateExtU32};
use micromath::F32Ext;
use panic_halt as _;
use rp_2040_lib::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock, PeripheralClock, SystemClock, UsbClock},
        gpio, i2c, pac,
        pio::PIOExt,
        pwm, reset,
        timer::Timer,
        usb::UsbBus,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};

//Addressable LED imports
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;
//BMP280 imports
use bmp280_rs::{Config, I2CAddress, BMP280};
// USB imports
use core::fmt::Write;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

extern crate alloc;
use alloc::{string::String, vec::Vec};

use littlefs2::{
    consts::U256,
    driver::Storage,
    fs::{FileType, Filesystem},
    io::SeekFrom,
    path::{Path, PathBuf},
};
use rp2040_flash::flash;
const XIP_BASE: u32 = 0x10000000;
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4 * 1024;
const FLASH_SIZE: u32 = 2 * 1024 * 1024;
const PROGRAM_SIZE: u32 = SECTOR_SIZE * 60; // 245KB

const ALTITUDE_THRESHOLD: f32 = 15.0_f32;

struct RpRom;
impl Storage for RpRom {
    type CACHE_SIZE = U256;
    type LOOKAHEAD_SIZE = U256;

    const READ_SIZE: usize = 256;
    const WRITE_SIZE: usize = PAGE_SIZE as usize;
    const BLOCK_SIZE: usize = SECTOR_SIZE as usize;
    const BLOCK_COUNT: usize = (FLASH_SIZE - PROGRAM_SIZE / SECTOR_SIZE) as usize;
    const BLOCK_CYCLES: isize = 1000;

    fn read(&mut self, addr: usize, buf: &mut [u8]) -> Result<usize, littlefs2::io::Error> {
        for i in 0..(buf.len() / PAGE_SIZE as usize) {
            unsafe {
                let read_buffer =
                    *((addr as u32 + XIP_BASE + PROGRAM_SIZE) as *const [u8; PAGE_SIZE as usize]);
                buf[(i * PAGE_SIZE as usize)..((i + 1) * PAGE_SIZE as usize)]
                    .copy_from_slice(&read_buffer[..]);
            }
        }
        Ok(buf.len())
    }
    fn write(&mut self, addr: usize, buf: &[u8]) -> Result<usize, littlefs2::io::Error> {
        unsafe {
            cortex_m::interrupt::free(|_cs| {
                flash::flash_range_program(addr as u32 + PROGRAM_SIZE, &buf, true);
            });
        }
        Ok(buf.len())
    }
    fn erase(&mut self, addr: usize, len: usize) -> Result<usize, littlefs2::io::Error> {
        unsafe {
            cortex_m::interrupt::free(|_cs| {
                flash::flash_range_erase(addr as u32 + PROGRAM_SIZE, len as u32, true);
            });
        }
        Ok(len)
    }
}

struct PWMParams {
    sys_freq: u32,
    top: u16,
    div_int: u8,
    div_frac: u8, // 4 bits
}
fn compute_pwm_params(sys_freq: u32, freq: u32) -> PWMParams {
    let freq_count = freq * 65536;
    let mut div_int = sys_freq / freq_count;
    let mut div_frac = ((((sys_freq % freq_count) * 16) / freq_count) as f32).round() as u32;

    if div_frac >= 16 {
        div_int += 1;
        div_frac = 0;
    }
    PWMParams {
        sys_freq,
        top: 65535,
        div_int: div_int as u8,
        div_frac: div_frac as u8,
    }
}
fn get_count_from_us(us: u32, pwm_params: &PWMParams) -> u16 {
    let count = (us * (pwm_params.sys_freq / 1_000_000)) as f32
        / (pwm_params.div_int as f32 + (pwm_params.div_frac / 16) as f32);
    let round = count.round() as u32;

    if round >= 65536 {
        65535
    } else {
        round as u16
    }
}

#[entry]
fn main() -> ! {
    // Create Heap
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mode_selector = pins.gp26.into_pull_up_input();

    let mut storage = RpRom;

    let mut alloc = Filesystem::allocate();
    if !Filesystem::is_mountable(&mut storage) {
        Filesystem::format(&mut storage).unwrap();
    }
    let fs = Filesystem::mount(&mut alloc, &mut storage).unwrap();

    let is_cli_mode = mode_selector.is_low().unwrap();
    if is_cli_mode {
        cli_mode(
            pac.PIO0,
            pac.PWM,
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            &mut pac.RESETS,
            pins.neopixel,
            pins.gp10,
            timer,
            &clocks.peripheral_clock,
            clocks.usb_clock,
            fs,
        )
    }

    normal_mode(
        pac.PIO0,
        pac.PWM,
        &mut pac.RESETS,
        mode_selector,
        pac.I2C1,
        pins.gp2.reconfigure(),
        pins.gp3.reconfigure(),
        pins.neopixel,
        pins.gp10,
        timer,
        &clocks.peripheral_clock,
        &clocks.system_clock,
        &fs,
    )
}

enum FrameLog {
    Event = 1,
    BMP = 2,
}
struct Logger<'a> {
    fs: &'a Filesystem<'a, RpRom>,
    system_clock: &'a SystemClock,
    run_path: String,
    baked_log_path: String,
}
impl<'a> Logger<'a> {
    pub fn new(fs: &'a Filesystem<'a, RpRom>, system_clock: &'a SystemClock) -> Self {
        let mut run_path: String = String::new();
        // Create a new dir for the current run
        fs.read_dir_and_then(&Path::from_str_with_nul("/\0"), |dirs| {
            let dir_count = dirs.count() - 2; // -2 because of /. and /..
            if dir_count > 9999 {
                panic!("Too many runs, please clear the storage");
            }
            write!(&mut run_path, "run_{}", dir_count).unwrap();
            let mut temp_str: String = String::new();
            write!(&mut temp_str, "{}\0", run_path).unwrap();
            fs.create_dir(&Path::from_str_with_nul(&temp_str)).unwrap();
            Ok(())
        })
        .unwrap();
        let mut log = Logger {
            fs,
            system_clock,
            run_path,
            baked_log_path: String::new(),
        };
        log.write_event_log("System initialized!");
        log
    }

    pub fn write_event_log(&mut self, log: &str) {
        // Baked log path for farter execution
        if self.baked_log_path.is_empty() {
            let mut temp_path: String = String::new();
            write!(&mut temp_path, "{}/log\0", self.run_path).unwrap();
            self.baked_log_path = temp_path;
        }

        self.fs
            .open_file_with_options_and_then(
                |options| options.read(true).append(true).create(true),
                &Path::from_str_with_nul(self.baked_log_path.as_str()),
                |file| {
                    let mut temp_str = String::new();
                    temp_str.push((FrameLog::Event as u8) as char);
                    temp_str.push('|');
                    temp_str.push_str(log);
                    temp_str.push('\0');
                    file.write(temp_str.as_bytes())?;
                    Ok(())
                },
            )
            .unwrap();
    }
}

fn normal_mode<'a>(
    pac_pio0: pac::PIO0,
    pac_pwm: pac::PWM,
    pac_resets: &mut pac::RESETS,
    modeselect_pin: gpio::Pin<gpio::bank0::Gpio26, gpio::FunctionSioInput, gpio::PullUp>,
    pac_i2c0: pac::I2C1,
    sda_pin: gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionI2c, gpio::PullUp>,
    scl_pin: gpio::Pin<gpio::bank0::Gpio3, gpio::FunctionI2c, gpio::PullUp>,
    neopixel: gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionNull, gpio::PullDown>,
    servo_pin: gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionNull, gpio::PullDown>,
    timer: Timer,
    peripheral_clock: &PeripheralClock,
    system_clock: &'a SystemClock,
    file_system: &'a Filesystem<'a, RpRom>,
) -> ! {
    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac_pio0.split(pac_resets);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        neopixel.into_function(),
        &mut pio,
        sm0,
        peripheral_clock.freq(),
        timer.count_down(),
    );
    ws.write(brightness(once(RGB8::new(255, 0, 0)), 100))
        .unwrap();

    let mut pwm_slices = pwm::Slices::new(pac_pwm, pac_resets);
    let pwm = &mut pwm_slices.pwm5;
    pwm.clr_ph_correct(); // Not use phase correct PWM Datasheet 4.5.2.1
                          // Require 20ms period period(s) = ((TOP + 1) * (DIV_INT + DIV_FRAC / 16)) / fclk_MHz
    let pwm_params = compute_pwm_params(peripheral_clock.freq().to_Hz(), 50);
    pwm.set_top(pwm_params.top);
    pwm.set_div_int(pwm_params.div_int);
    pwm.set_div_frac(pwm_params.div_frac);
    let servo_open: u16 = get_count_from_us(1000, &pwm_params); // Duty cycle 1000us
    let servo_close: u16 = get_count_from_us(1800, &pwm_params); // Duty cycle 1000us
    pwm.enable();
    let servo_pwm = &mut pwm.channel_a;
    servo_pwm.output_to(servo_pin);
    servo_pwm.set_duty(servo_close);

    /*
     * Create the I²C drive, using the two pre-configured pins. This will fail
     * at compile time if the pins are in the wrong mode, or if this I²C
     * peripheral isn't available on these pins!
     */
    let mut i2c = i2c::I2C::i2c1(
        pac_i2c0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        pac_resets,
        system_clock,
    );
    // Create a BMP280 driver
    let mut bmp = BMP280::new(
        &mut i2c,
        I2CAddress::SdoGrounded,
        Config::handheld_device_dynamic(),
    )
    .unwrap()
    .into_normal_mode(&mut i2c)
    .unwrap();

    // Creating logger handler
    let mut logger = Logger::new(file_system, system_clock);

    // FORCE DEPLOYEMENT
    ws.write(brightness(once(RGB8::new(0, 255, 0)), 100))
        .unwrap();
    let mut delay = timer.count_down();
    delay.start(2.secs());
    let _ = nb::block!(delay.wait());
    if modeselect_pin.is_low().unwrap() {
        servo_pwm.set_duty(servo_open);
        logger.write_event_log("Force deployement!");
        loop {
            // Turn off the LED (black)
            ws.write(brightness(once(RGB8::new(0, 0, 0)), 0)).unwrap();
            delay.start(800.millis());
            let _ = nb::block!(delay.wait());
            // Turn on the LED (white)
            ws.write(brightness(once(RGB8::new(255, 255, 255)), 100))
                .unwrap();
            delay.start(100.millis());
            let _ = nb::block!(delay.wait());
        }
    }

    // STANDBY / WAIT FOR LAUNCH
    logger.write_event_log("Waiting for launch...");
    ws.write(brightness(once(RGB8::new(255, 0, 0)), 100))
        .unwrap();
    let mut buffer_index: usize = 0;
    let mut circ_buffer = [0u8; PAGE_SIZE as usize];
    let mut sea_level_pressure = 0;
    loop {
        let _ = bmp.read_temperature(&mut i2c).unwrap();
        let pressure = bmp.read_pressure(&mut i2c).unwrap();
        if sea_level_pressure == 0 {
            sea_level_pressure = pressure;
        }
        circ_buffer[buffer_index..buffer_index + 4].copy_from_slice(&pressure.to_be_bytes());
        buffer_index = (buffer_index + 4) % PAGE_SIZE as usize;
        let altitude: f32 =
            44330.0 * (1.0 - ((pressure as f32) / (sea_level_pressure as f32)).powf(0.1903));

        if altitude > ALTITUDE_THRESHOLD {
            break;
        }

        if (timer.get_counter().ticks() / 1_000) % 1_000 < 500 {
            ws.write(brightness(once(RGB8::new(0, 255, 0)), 100))
                .unwrap();
        } else if (timer.get_counter().ticks() / 1_000) % 1000 >= 500 {
            ws.write(brightness(once(RGB8::new(255, 0, 0)), 100))
                .unwrap();
        }
    }

    logger.write_event_log("Launch detected!");
    ws.write(brightness(once(RGB8::new(0, 0, 255)), 100))
        .unwrap();

    loop {}
}

const SERIAL_BUFFER_SIZE: usize = 64;

fn cli_mode(
    pac_pio0: pac::PIO0,
    pac_pwm: pac::PWM,
    pac_usbctrl_regs: pac::USBCTRL_REGS,
    pac_usbctrl_dpram: pac::USBCTRL_DPRAM,
    pac_resets: &mut pac::RESETS,
    neopixel: gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionNull, gpio::PullDown>,
    servo_pin: gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionNull, gpio::PullDown>,
    timer: Timer,
    peripheral_clock: &PeripheralClock,
    usb_clock: UsbClock,
    file_system: Filesystem<'_, RpRom>,
) -> ! {
    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac_pio0.split(pac_resets);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        neopixel.into_function(),
        &mut pio,
        sm0,
        peripheral_clock.freq(),
        timer.count_down(),
    );

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac_usbctrl_regs,
        pac_usbctrl_dpram,
        usb_clock,
        true,
        pac_resets,
    ));
    // Use serial over USB
    let mut serial = SerialPort::new(&usb_bus);
    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("PI_PICO")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    while timer.get_counter().ticks() < 1_000_000 {
        usb_dev.poll(&mut [&mut serial]);
    }

    let mut pwm_slices = pwm::Slices::new(pac_pwm, pac_resets);
    let pwm = &mut pwm_slices.pwm5;
    pwm.clr_ph_correct(); // Not use phase correct PWM Datasheet 4.5.2.1
                          // Require 20ms period period(s) = ((TOP + 1) * (DIV_INT + DIV_FRAC / 16)) / fclk_MHz
    let pwm_params = compute_pwm_params(peripheral_clock.freq().to_Hz(), 50);
    pwm.set_top(pwm_params.top);
    pwm.set_div_int(pwm_params.div_int);
    pwm.set_div_frac(pwm_params.div_frac);
    let servo_open: u16 = get_count_from_us(1000, &pwm_params); // Duty cycle 1000us
    pwm.enable();
    let servo_pwm = &mut pwm.channel_a;
    servo_pwm.output_to(servo_pin);
    servo_pwm.set_duty(servo_open);

    ws.write(brightness(once(RGB8::new(255, 0, 255)), 80))
        .unwrap();
    let _ = serial.write(b"-- Running Baromiter timer CLI mode --\r\n");
    usb_dev.poll(&mut [&mut serial]);
    let mut delay = timer.count_down();

    loop {
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; SERIAL_BUFFER_SIZE];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    let mut raw_cmd_str =
                        String::from_utf8(buf[0..count].to_vec()).unwrap_or(String::new());

                    // Sanitize input
                    while !raw_cmd_str.is_empty()
                        && (raw_cmd_str.chars().last().unwrap_or(' ') == '\n'
                            || raw_cmd_str.chars().last().unwrap_or(' ') == '\r')
                    {
                        raw_cmd_str.pop();
                    }
                    if raw_cmd_str.is_empty() {
                        continue;
                    }

                    let mut text: String = String::new();
                    write!(&mut text, "> {}\r\n", raw_cmd_str).unwrap();
                    let _ = serial.write(text.as_bytes());

                    // Parsing args from cmd
                    let args: Vec<&str> = raw_cmd_str.split_whitespace().collect();

                    match args[0] {
                        "ls" => {
                            // info and content of dir
                            let used_percentage: f32 = 1.0
                                - (file_system.available_space().unwrap() as f32)
                                    / file_system.total_space() as f32;
                            let mut text: String = String::new();
                            write!(&mut text, "used:{}%\r\n", used_percentage).unwrap();
                            let _ = serial.write(text.as_bytes());
                            text.clear();

                            let bytes_path = if args.len() >= 2 {
                                args[1].as_bytes()
                            } else {
                                b"/"
                            };
                            let mut string_path = String::from_utf8(bytes_path.to_vec()).unwrap();
                            string_path.push('\0');

                            match file_system.read_dir_and_then(
                                &Path::from_str_with_nul(string_path.as_str()),
                                |dirs| {
                                    for dir in dirs {
                                        let dir = dir.unwrap();

                                        let mut text: String = String::new();
                                        let path = dir.path().as_str_ref_with_trailing_nul();
                                        write!(
                                            &mut text,
                                            "{}({}B):{}\r\n",
                                            if dir.file_type().is_file() {
                                                "file"
                                            } else {
                                                "dir"
                                            },
                                            dir.metadata().len(),
                                            &path[..path.len() - 1],
                                        )
                                        .unwrap();
                                        let _ = serial.write(text.as_bytes());
                                    }
                                    Ok(())
                                },
                            ) {
                                Ok(()) => {}
                                Err(_) => {
                                    let _ = serial.write(b"err:PathNotFound\r\n");
                                }
                            }
                        }
                        "rm" => {
                            // delete file or folder
                            if args.len() < 2 {
                                let _ = serial.write(b"err:NoPathProvide\r\n");
                                continue;
                            }

                            for bytes_path in args.iter().skip(1) {
                                let mut string_path = String::from_utf8(bytes_path.as_bytes().to_vec()).unwrap();
                                string_path.push('\0');

                                match file_system
                                    .metadata(&Path::from_str_with_nul(string_path.as_str()))
                                {
                                    Err(err) => {
                                        let mut text: String = String::new();
                                        write!(&mut text, "err:{:?}\r\n", err).unwrap();
                                        let _ = serial.write(text.as_bytes());
                                        continue;
                                    }
                                    Ok(metadata) => match metadata.file_type() {
                                        FileType::File => {
                                            file_system
                                                .remove(&Path::from_str_with_nul(string_path.as_str()))
                                                .unwrap_or_else(|err| {
                                                    let mut text: String = String::new();
                                                    write!(&mut text, "err:{:?}\r\n", err).unwrap();
                                                    let _ = serial.write(text.as_bytes());
                                                });
                                        }
                                        FileType::Dir => {
                                            file_system
                                                .remove_dir_all(&Path::from_str_with_nul(
                                                    string_path.as_str(),
                                                ))
                                                .unwrap_or_else(|err| {
                                                    let mut text: String = String::new();
                                                    write!(&mut text, "err:{:?}\r\n", err).unwrap();
                                                    let _ = serial.write(text.as_bytes());
                                                });
                                        }
                                    },
                                }
                            }
                        }
                        "cat" => {
                            // read file
                            if args.len() != 2 {
                                let _ = serial.write(b"err:PathNotFound\r\n");
                                continue;
                            }

                            let bytes_path = args[1].as_bytes();
                            let mut string_path = String::from_utf8(bytes_path.to_vec()).unwrap();
                            string_path.push('\0');

                            const BUFFER_READ_SIZE: usize = 64;
                            let mut buff = [0u8; BUFFER_READ_SIZE];
                            file_system
                                .open_file_with_options_and_then(
                                    |options| options.read(true),
                                    &Path::from_str_with_nul(string_path.as_str()),
                                    |file| {
                                        let mut offset = 0;
                                        let mut read_length;
                                        let _ = serial.write(b"read:");
                                        loop {
                                            file.seek(SeekFrom::Start(offset as u32)).unwrap();
                                            read_length = file.read(&mut buff).unwrap();
                                            if read_length == 0 {
                                                break;
                                            }

                                            match serial.write(&buff[0..read_length]) {
                                                Ok(send_size) => {
                                                    offset += send_size;
                                                }
                                                Err(e) => {
                                                    match e {
                                                        UsbError::WouldBlock => {
                                                            ws.write(brightness(
                                                                once(RGB8::new(0, 0, 255)),
                                                                80,
                                                            ))
                                                            .unwrap();
                                                            let _ = serial
                                                                .write(b"\r\nError sending file");
                                                        }
                                                        _ => {
                                                            let _ = serial
                                                                .write(b"\r\nError reading file");
                                                        }
                                                    }
                                                    break;
                                                }
                                            }
                                            delay.start(10.millis());
                                            let _ = nb::block!(delay.wait());
                                        }
                                        let _ = serial.write(b"\r\nend\r\n");
                                        Ok(())
                                    },
                                )
                                .unwrap_or_else(|err| {
                                    let mut text: String = String::new();
                                    write!(&mut text, "err:{:?}\r\n", err).unwrap();
                                    let _ = serial.write(text.as_bytes());
                                });
                        }
                        "test" => {
                            // run test suite
                            let _ = serial.write(b"LAUNCH TEST\r\n");
                            match file_system.create_dir(&Path::from_str_with_nul("test\0")) {
                                Ok(()) => {
                                    let _ = serial.write(b"TEST PASSED: Created dir!\r\n");
                                }
                                Err(err) => {
                                    let mut text: String = String::new();
                                    write!(&mut text, "TEST FAILED: create dir err: {:?}\r\n", err)
                                        .unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
                            }

                            // Delete dir
                            match file_system.remove_dir(&Path::from_str_with_nul("test\0")) {
                                Ok(()) => {
                                    let _ = serial.write(b"TEST PASSED: Removed dir!\r\n");
                                }
                                Err(err) => {
                                    let mut text: String = String::new();
                                    write!(&mut text, "Fail remove dir err: {:?}\r\n", err)
                                        .unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
                            }

                            // Test file manipulation
                            match file_system.open_file_with_options_and_then(
                                |options| options.read(true).write(true).create(true),
                                &Path::from_str_with_nul("example.txt\0"),
                                |file| {
                                    let mut buf = [0u8; 11];
                                    file.write(b"hello")?;
                                    file.seek(SeekFrom::Start(0))?;
                                    let read_length = file.read(&mut buf)?;
                                    if read_length != 5 {
                                        let mut text: String = String::new();
                                        write!(
                                            &mut text,
                                            "TEST FAILED: Read not good length ({})!\r\n",
                                            read_length
                                        )
                                        .unwrap();
                                        let _ = serial.write(text.as_bytes());
                                    } else {
                                        if &buf[0..5] != b"hello" {
                                            let _ = serial
                                                .write(b"TEST FAILED: Read not good text!\r\n");
                                        } else {
                                            let _ = serial.write(
                                                b"TEST PASSED: Create, write and read file!\r\n",
                                            );
                                        }
                                    }
                                    Ok(())
                                },
                            ) {
                                Ok(()) => {}
                                Err(err) => {
                                    let mut text: String = String::new();
                                    write!(
                                        &mut text,
                                        "TEST FAILED: file manipulation err: {:?}\r\n",
                                        err
                                    )
                                    .unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
                            }

                            let _ = serial.write(b"END TEST\r\n");
                        }
                        "exit" => {
                            // q (quit)
                            let _ = serial.write(b"Exiting CLI mode\r\n");
                            reset();
                        }
                        _ => {
                            let _ = serial.write(b"Unknown command\r\n");
                        }
                    }
                }
            }
        }
    }
}
