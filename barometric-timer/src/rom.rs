use littlefs2::{
    consts::U256,
    driver::Storage,
};
use rp2040_flash::flash;

const XIP_BASE: u32 = 0x10000000;
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4 * 1024;
const FLASH_SIZE: u32 = 2 * 1024 * 1024;
const PROGRAM_SIZE: u32 = SECTOR_SIZE * 67; // 270KB

pub struct RpRom;
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
