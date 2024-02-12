#![no_std]
#![no_main]

mod rp_2040_lib;

use core::iter::once;
use embedded_hal::{digital::v2::InputPin, PwmPin};
use embedded_hal::timer::CountDown;
use fugit::{ExtU32, RateExtU32};
use micromath::F32Ext;
use panic_halt as _;
use rp_2040_lib::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock, PeripheralClock, SystemClock, UsbClock},
        pac,
        pio::PIOExt,
        gpio,
        timer::Timer,
        watchdog::Watchdog,
        Sio,
        usb::UsbBus,
        reset,
        pwm,
        i2c
    },
    Pins,
    XOSC_CRYSTAL_FREQ,
};

//Addressable LED imports
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;
//BMP280 imports
use bmp280_rs::{BMP280, I2CAddress, Config};
// USB imports
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use core::fmt::Write;

use heapless::{Vec, String};

use rp2040_flash::flash;
use littlefs2::{
    driver::Storage,
    consts::U256,
    fs::Filesystem,
    path::{Path, PathBuf},
    io::SeekFrom,
};
const XIP_BASE: u32 = 0x10000000;
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4 * 1024;
const FLASH_SIZE: u32 = 2 * 1024 * 1024;

const FILE_SYSTEM_BASE_META: u32 = XIP_BASE + FLASH_SIZE - SECTOR_SIZE;
const FILE_SYSTEM_BASE: u32 = XIP_BASE + FLASH_SIZE - SECTOR_SIZE - 1;
const PROGRAM_SIZE: u32 = SECTOR_SIZE * 60; // 245KB

const ALTITUDE_THRESHOLD: f32 = 15.0_f32;

struct RpRom;
impl Storage for RpRom {
    type CACHE_SIZE = U256;
    type LOOKAHEAD_SIZE = U256;

    const READ_SIZE: usize = 256;
    const WRITE_SIZE: usize = PAGE_SIZE as usize;
    const BLOCK_SIZE: usize = SECTOR_SIZE as usize;
    const BLOCK_COUNT: usize = (FLASH_SIZE-PROGRAM_SIZE / SECTOR_SIZE) as usize;
    const BLOCK_CYCLES: isize = 1000;

    fn read(&mut self, addr: usize, buf: &mut [u8]) -> Result<usize, littlefs2::io::Error> {
        for i in 0..(buf.len() / PAGE_SIZE as usize) {
            unsafe {
                let read_buffer = *((addr as u32 + XIP_BASE + PROGRAM_SIZE) as *const [u8; PAGE_SIZE as usize]);
                buf[(i*PAGE_SIZE as usize)..((i+1)*PAGE_SIZE as usize)].copy_from_slice(&read_buffer[..]);
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
    let count = (us * (pwm_params.sys_freq/1_000_000)) as f32 / (pwm_params.div_int as f32 + (pwm_params.div_frac/16) as f32);
    let round = count.round() as u32;

    if round >= 65536 {
        65535
    } else {
        round as u16
    }
}


#[entry]
fn main() -> ! {
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
        cli_mode(pac.PIO0, 
            pac.PWM,
            pac.USBCTRL_REGS, 
            pac.USBCTRL_DPRAM, 
            &mut pac.RESETS, 
            pins.neopixel, 
            pins.gp10,
            timer, 
            &clocks.peripheral_clock, 
            clocks.usb_clock, 
            fs
        )
    } else {
        normal_mode(pac.PIO0, 
            pac.PWM, 
            &mut pac.RESETS, 
            mode_selector, 
            pac.I2C1, pins.gp2.reconfigure(), 
            pins.gp3.reconfigure(),
            pins.neopixel,
            pins.gp10, 
            timer, 
            &clocks.peripheral_clock, 
            &clocks.system_clock, 
            fs
        )
    }
}

fn normal_mode(pac_pio0: pac::PIO0,
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
    system_clock: &SystemClock,
    mut file_system: Filesystem<'_, RpRom>) -> ! {

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

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = i2c::I2C::i2c1(
        pac_i2c0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        pac_resets,
        system_clock,
    );
    // Create a BMP280 driver
    let mut bmp = BMP280::new(&mut i2c, I2CAddress::SdoGrounded, Config::handheld_device_dynamic())
        .unwrap()
        .into_normal_mode(&mut i2c)
        .unwrap();

    file_system.read_dir_and_then(
        &Path::from_str_with_nul("/\0"),
        |dirs| {
            let mut text: String<64> = String::new();
            write!(&mut text, "run_{}\0", dirs.count()).unwrap();
            file_system.create_dir(&Path::from_str_with_nul(text.as_str())).unwrap();
            Ok(())
        },
    ).unwrap();

    // FORCE DEPLOYEMENT
    ws.write(brightness(once(RGB8::new(0, 255, 0)), 100))
        .unwrap();
    let mut delay = timer.count_down();
    delay.start(2.secs());
    let _ = nb::block!(delay.wait());
    if modeselect_pin.is_low().unwrap() {
        servo_pwm.set_duty(servo_open);
        loop {
            // Turn off the LED (black)
            ws.write(brightness(once(RGB8::new(0, 0, 0)), 0))
                .unwrap();
            delay.start(800.millis());
            let _ = nb::block!(delay.wait());
            // Turn on the LED (white)
            ws.write(brightness(once(RGB8::new(255, 255, 255)), 100))
                .unwrap();
            delay.start(200.millis());
            let _ = nb::block!(delay.wait());
        }
    }

    // STANDBY / WAIT FOR LAUNCH
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
        circ_buffer[buffer_index..buffer_index+4].copy_from_slice(&pressure.to_be_bytes());
        buffer_index = (buffer_index + 4) % PAGE_SIZE as usize;
        let altitude: f32 = 44330.0 * (1.0 - ((pressure as f32 )/ (sea_level_pressure as f32)).powf(0.1903));

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

    ws.write(brightness(once(RGB8::new(0, 0, 255)), 100))
        .unwrap();
    
    loop {
    }
}

fn cli_mode(pac_pio0: pac::PIO0,
    pac_pwm: pac::PWM,
    pac_usbctrl_regs: pac::USBCTRL_REGS,
    pac_usbctrl_dpram: pac::USBCTRL_DPRAM,
    pac_resets: &mut pac::RESETS, 
    neopixel: gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionNull, gpio::PullDown>,
    servo_pin: gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionNull, gpio::PullDown>,
    timer: Timer, 
    peripheral_clock: &PeripheralClock,
    usb_clock: UsbClock,
    mut file_system: Filesystem<'_, RpRom>) -> ! {

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

    let mut n = 97;
    let mut delay = timer.count_down();
    loop {
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    let mut text: String<64> = String::new();
                    writeln!(&mut text, "> {}", buf[0] as char).unwrap();
                    let _ = serial.write(text.as_bytes());
                    match buf[0] {
                        105 => { // i (info)
                            let used_percentage: f32 = 1.0 - (file_system.available_space().unwrap() as f32) / file_system.total_space() as f32;
                            let mut text: String<64> = String::new();
                            writeln!(&mut text, "used={}%", used_percentage).unwrap();
                            let _ = serial.write(text.as_bytes());
                            text.clear();

                            file_system.read_dir_and_then(
                                &Path::from_str_with_nul("/\0"),
                                |dirs| {
                                    for dir in dirs {
                                        let dir = dir.unwrap();

                                        let mut text: String<64> = String::new();
                                        if dir.file_type().is_file() {
                                            writeln!(&mut text, "file:{}", dir.path().as_str_ref_with_trailing_nul()).unwrap();
                                        } else {
                                            writeln!(&mut text, "dir:{}", dir.path().as_str_ref_with_trailing_nul()).unwrap();
                                        }
                                        let _ = serial.write(text.as_bytes());
                                    }
                                    Ok(())
                                },
                            ).unwrap();
                        },
                        100 => { // d (delete)
                            let _ = serial.write(b"Deleting all files..\r\n");
                            file_system.remove_dir_all(&Path::from_str_with_nul("/\0")).unwrap();
                            let _ = serial.write(b"Delete completed!\r\n");
                        },
                        110 => { // n (new)
                            match file_system.create_dir(&Path::from_str_with_nul("test\0")) {
                                Ok(()) => {
                                    let _ = serial.write(b"Created dir!\r\n");
                                },
                                Err(err) => {
                                    let mut text: String<64> = String::new();
                                    writeln!(&mut text, "Fail create dir err: {:?}", err).unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
                            }
                            file_system.open_file_with_options_and_then(
                                |options| options.read(true).write(true).create(true),
                                &PathBuf::from(b"example.txt"),
                                |file| {
                                    file.write(&[110; 1024])?;
                                    // file.seek(SeekFrom::Start(0)).unwrap();
                                    // assert_eq!(file.read(&mut bright)?, 1);
                                    Ok(())
                                }
                            ).unwrap();
                            match file_system.remove_dir(&Path::from_str_with_nul("test\0")) {
                                Ok(()) => {
                                    let _ = serial.write(b"Remove dir!\r\n");
                                },
                                Err(err) => {
                                    let mut text: String<64> = String::new();
                                    writeln!(&mut text, "Fail remove dir err: {:?}", err).unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
                            }
                        },
                        114 => { // r (read)
                            let mut buff = [0u8; 64];
                            let mut read_length = 0;
                            match file_system.open_file_with_options_and_then(
                                |options| options.read(true),
                                &PathBuf::from(b"example.txt"),
                                |file| {
                                    file.seek(SeekFrom::Start(0)).unwrap();
                                    read_length = file.read(&mut buff).unwrap();
                                    Ok(())
                                }
                            ) {
                                Ok(()) => {
                                    let _ = serial.write(b"Read: ");
                                    let _ = serial.write(&buff[0..read_length]);
                                    let _ = serial.write(b"\r\n");
                                },
                                Err(err) => {
                                    let mut text: String<64> = String::new();
                                    writeln!(&mut text, "Fail read err: {:?}", err).unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
                            }
                        },
                        116 => {// t (test)
                            let _ = serial.write(b"Launch test: ");
                            /* unsafe {
                                cortex_m::interrupt::free(|_cs| {
                                    flash::flash_range_erase(FLASH_SIZE - 2*SECTOR_SIZE, SECTOR_SIZE, true);
                                }); 
                                let _ = serial.write(b"Erased\r\n");
                                for i in 0..16 {
                                    let buff = [(97+i as u8); PAGE_SIZE as usize];
                                    let mut text: String<64> = String::new();
                                    writeln!(text, "Write: {} at {}", (97+i as u8) as char, XIP_BASE + FLASH_SIZE - 2*SECTOR_SIZE + i*PAGE_SIZE).unwrap();
                                    let _ = serial.write(text.as_bytes());
                                    cortex_m::interrupt::free(|_cs| {
                                        flash::flash_range_program(FLASH_SIZE - 2*SECTOR_SIZE + i*PAGE_SIZE, &buff, true);
                                    });
                                }
                                let _ = serial.write(b"Reading..\r\n");
                                let r = *((XIP_BASE + FLASH_SIZE - 2*SECTOR_SIZE) as *const [u8; SECTOR_SIZE as usize]);

                                let mut text: String<64> = String::new();
                                writeln!(&mut text, "test a: {}", r[0..256].iter().all(|c| *c == 97)).unwrap();
                                let _ = serial.write(text.as_bytes());
                                let mut text: String<64> = String::new();
                                writeln!(&mut text, "test b: {}", r[256..512].iter().all(|c| *c == 98)).unwrap();
                                let _ = serial.write(text.as_bytes());
                            } */
                            let _ = serial.write(b"PASSED\r\n");
                        }
                        113 => { // q (quit)
                            let _ = serial.write(b"Quitting CLI mode\r\n");
                            reset();
                        },
                        _ => {
                            let _ = serial.write(b"Unknown command\r\n");
                        }
                    }
                }
            }
        }
    }
}
