#![no_std]
#![no_main]

mod rp_2040_lib;

use core::iter::once;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use panic_halt as _;
use rp2040_hal::rom_data::connect_internal_flash;
use rp_2040_lib::entry;
use rp_2040_lib::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        pio::PIOExt,
        timer::Timer,
        watchdog::Watchdog,
        Sio,
        usb::UsbBus,
        reset
    },
    Pins, XOSC_CRYSTAL_FREQ,
};

//Addressable LED imports
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;
// USB imports
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use core::fmt::Write;

use heapless::{Vec, String};

use rp2040_flash::flash;
const XIP_BASE: u32 = 0x10000000;
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4 * 1024;
const FLASH_SIZE: u32 = 2 * 1024 * 1024;

const FILE_SYSTEM_BASE_META: u32 = XIP_BASE + FLASH_SIZE - SECTOR_SIZE;
const FILE_SYSTEM_BASE: u32 = XIP_BASE + FLASH_SIZE - SECTOR_SIZE - 1;
const PROGRAM_SIZE: u32 = SECTOR_SIZE * 30; // 120KB

#[repr(C)]
struct FileSystem {
    number_of_files: u8,
    files: Vec<File, 255>,
    current_buffer_addr: u32,
}
impl FileSystem {
    pub unsafe fn init() -> Self {
        let number_of_files: u8 = *(FILE_SYSTEM_BASE_META as *const u8);
        let mut files: Vec<File, 255> = Vec::new();
        for i in 0..number_of_files {
            let addr = FILE_SYSTEM_BASE_META + 1 + (i as usize * File::size_of()) as u32;
            
            files.push(File {
                address: u32::from_le_bytes(*(addr as *const [u8; 4])),
                size: u32::from_le_bytes(*((addr+4) as *const [u8; 4])),
            }).unwrap();
        }
        let current_buffer_addr = files.last().map_or(FILE_SYSTEM_BASE_META - PAGE_SIZE, |f| f.address - ((f.size / PAGE_SIZE)+1)*PAGE_SIZE + 1);
        Self {
            number_of_files,
            files,
            current_buffer_addr
        }
    }

    pub unsafe fn flush(&mut self) {
        self.number_of_files = 0;
        self.files.clear();
        
        let buffer = [0u8; SECTOR_SIZE as usize];
        cortex_m::interrupt::free(|_cs| {
            flash::flash_range_erase_and_program(FILE_SYSTEM_BASE_META - XIP_BASE, &buffer, true);
        });
    }

    pub unsafe fn save_metedata(&self) {
        let mut buffer = [0u8; SECTOR_SIZE as usize];
        buffer[0] = self.number_of_files;

        for (i, file) in self.files.iter().enumerate() {
            let addr = 1 + (i as usize * File::size_of());
            buffer[addr..addr+8].copy_from_slice(&file.to_bytes());
        }

        cortex_m::interrupt::free(|_cs| {
            flash::flash_range_erase_and_program(FILE_SYSTEM_BASE_META - XIP_BASE, &buffer, true);
        });
    } 

    pub fn create_new_file(&mut self) -> Option<()> {
        if self.number_of_files == 255 {
            return None;
        }
        let last_file = self.files.last().map_or(File { address: FILE_SYSTEM_BASE, size: 0 }, |f| *f);
        let new_addr = last_file.address - last_file.size.div_ceil(SECTOR_SIZE)*SECTOR_SIZE;
        if new_addr < XIP_BASE + PROGRAM_SIZE {
            return None;
        }
        unsafe {
            cortex_m::interrupt::free(|_cs| {
                flash::flash_range_erase(new_addr - XIP_BASE, SECTOR_SIZE, true);
            }); 
        }
        self.files.push(File {
            address: new_addr,
            size: 0,
        }).unwrap();
        self.number_of_files += 1;
        self.current_buffer_addr = self.files.last().map_or(FILE_SYSTEM_BASE_META - PAGE_SIZE, |f| f.address - ((f.size / PAGE_SIZE)+1)*PAGE_SIZE + 1);
        unsafe { self.save_metedata() };
        Some(())
    }

    pub unsafe fn push_buffer(&mut self, data: &[u8; PAGE_SIZE as usize]) {
        if self.files.is_empty() {
            return;
        }
        let file = self.files.last_mut().unwrap();
        if file.size % SECTOR_SIZE == 0 {
            let new_addr = file.address - ((file.size / SECTOR_SIZE)+1) * SECTOR_SIZE;
            cortex_m::interrupt::free(|_cs| {
                flash::flash_range_erase(new_addr - XIP_BASE, SECTOR_SIZE, true);
            }); 
        }
        file.size += PAGE_SIZE;
        
        let mut rev_buffer = [0xff; PAGE_SIZE as usize];
        rev_buffer.copy_from_slice(data);
        rev_buffer.reverse();
        cortex_m::interrupt::free(|_cs| {
            flash::flash_range_program(self.current_buffer_addr as u32 - XIP_BASE, &rev_buffer, true);
        });
        self.current_buffer_addr -= PAGE_SIZE;
        self.save_metedata();
    }

    pub unsafe fn read_relative_slice(file: &File, start_addr: u32) -> ([u8; PAGE_SIZE as usize], usize) {
        let size = core::cmp::min(file.size - start_addr, PAGE_SIZE) as usize;
        let addr = file.address - start_addr - PAGE_SIZE as u32 + 1;
        
        let mut read_buffer = unsafe { *(addr as *const [u8; PAGE_SIZE as usize]) };
        read_buffer.reverse();
    
        let mut buffer = [0xffu8; PAGE_SIZE as usize];
        buffer[0..size].copy_from_slice(&read_buffer[0..size]);
        
        (buffer, size)
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct File {
    address: u32,
    size: u32,
}
impl File {
    pub fn size_of() -> usize {
        core::mem::size_of::<File>()
    }
    pub fn to_bytes(&self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        bytes[0..4].copy_from_slice(self.address.to_le_bytes().as_ref());
        bytes[4..8].copy_from_slice(self.size.to_le_bytes().as_ref());
        bytes
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
    let mut delay = timer.count_down();
    
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
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

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mode_selector = pins.gp3.into_pull_down_input();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    ws.write(brightness(once(RGB8::new(255, 0, 0)), 100))
        .unwrap();

    
    let mut file_system = unsafe { FileSystem::init() };
    
    while timer.get_counter().ticks() < 1_000_000 {
        usb_dev.poll(&mut [&mut serial]);
    }

    let is_cli_mode = mode_selector.is_high().unwrap();
    if is_cli_mode {
        ws.write(brightness(once(RGB8::new(255, 0, 255)), 80))
            .unwrap();
        let _ = serial.write(b"-- Running Baromiter timer CLI mode --\r\n");
        usb_dev.poll(&mut [&mut serial]);
    } else {
        ws.write(brightness(once(RGB8::new(0, 255, 0)), 80))
            .unwrap();
        let _ = file_system.create_new_file().unwrap();
    }

    // Infinite colour wheel loop
    let mut n: u8 = 0;
    let mut run_buffer = [0u8; PAGE_SIZE as usize];
    loop {
        if !is_cli_mode {
            run_buffer[n as usize] = n;
            
            if n == 255 {
                unsafe {
                    file_system.push_buffer(&run_buffer);
                }
                run_buffer = [0u8; PAGE_SIZE as usize];
            }
            
            n = n.wrapping_add(1);
            delay.start(10.millis());
            let _ = nb::block!(delay.wait());
        } else {
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
                                let mut text: String<64> = String::new();
                                writeln!(&mut text, "f={}", file_system.number_of_files).unwrap();
                                let _ = serial.write(text.as_bytes());
                                for (i, file) in file_system.files.iter().enumerate() {
                                    let mut text: String<64> = String::new();
                                    writeln!(&mut text, "{i}:a={}|s={}", file.address, file.size).unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
                                let mut text: String<64> = String::new();
                                writeln!(text, "b={}", file_system.current_buffer_addr).unwrap();
                                let _ = serial.write(text.as_bytes());

                            },
                            100 => { // d (delete)
                                let _ = serial.write(b"Deleting all files..\r\n");
                                unsafe { file_system.flush() };
                                let _ = serial.write(b"Delete completed!\r\n");
                            },
                            110 => { // n (new file)
                                let _ = serial.write(b"Creating new file..\r\n");
                                file_system.create_new_file().unwrap();
                                let mut text: String<64> = String::new();
                                writeln!(&mut text, "New file at {}|Base={}!", file_system.files.last().unwrap().address, FILE_SYSTEM_BASE).unwrap();
                                let _ = serial.write(text.as_bytes());
                            },
                            119 => { // w (write)
                                let _ = serial.write(b"Writing to file..\r\n");
                                if n < 97 || n > 122 {
                                    n = 97;
                                }
                                let buff = [n; PAGE_SIZE as usize];
                                unsafe { file_system.push_buffer(&buff) };
                                n += 1;
                                let _ = serial.write(&buff);
                                let _ = serial.write(b"\r\nWrite completed!\r\n");
                            },
                            // e (erase flash)
                            101 => {
                                let _ = serial.write(b"Erasing flash..\r\n");
                                unsafe { 
                                    cortex_m::interrupt::free(|_cs| {
                                        flash::flash_range_erase(FLASH_SIZE - 2*SECTOR_SIZE, SECTOR_SIZE, true);
                                    }); 
                                }
                                let _ = serial.write(b"Erasing completed!\r\n");
                            },
                            114 => { // r (read)
                                if file_system.files.is_empty() {
                                    let _ = serial.write(b"No files to read..\r\n");
                                    continue;
                                }
                                let index = if count == 3 { buf[1] } else { file_system.files.len() as u8 - 1 };
                                let mut text: String<64> = String::new();
                                writeln!(&mut text, "Reading file {}|{}", index+1, file_system.files.len()).unwrap();
                                let _ = serial.write(text.as_bytes());

                                let file = file_system.files.get(index as usize).unwrap();
                                let mut offset = 0;
                                if file.size <= 0 {
                                    let _ = serial.write(b"File is empty..\r\n");
                                    continue;
                                }
                                loop {
                                    let (buffer, size) = unsafe { FileSystem::read_relative_slice(&file, offset) };
                                    match serial.write(&buffer[0..size]) {
                                        Ok(send_size) => {offset += send_size as u32;},
                                        Err(e) => {
                                            match e {
                                                UsbError::WouldBlock => {
                                                    ws.write(brightness(once(RGB8::new(0, 0, 255)), 80))
                                                        .unwrap();
                                                    let _ = serial.write(b"\r\nError sending file");
                                                },
                                                _ => {
                                                    let _ = serial.write(b"\r\nError reading file");
                                                }
                                            }
                                            break;
                                        }
                                    }
                                    let _ = serial.write(b"\r\n");
                                    delay.start(10.millis());
                                    let _ = nb::block!(delay.wait());
                                    if offset >= file.size {
                                        let mut text: String<64> = String::new();
                                        writeln!(&mut text, "read: {} bytes", offset).unwrap();
                                        let _ = serial.write(text.as_bytes());
                                        break;
                                    }
                                }
                            },
                            116 => {// t (test)
                                let _ = serial.write(b"Launch test: ");
                                unsafe {
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
                                    writeln!(&mut text, "test t: {}", r[256..512].iter().all(|c| *c == 116)).unwrap();
                                    let _ = serial.write(text.as_bytes());
                                }
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
}
