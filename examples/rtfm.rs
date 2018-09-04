#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate usb_device;
extern crate stm32f103xx_usb;

use usb_device::prelude::*;

static DEV: Option<UsbDevice<stm32f103xx_usb::UsbBus>> = None;
static BUS: Option<stm32f103xx_usb::UsbBus> = None;

entry!(main);
fn main() -> ! {
    loop { }
}