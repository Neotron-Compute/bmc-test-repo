//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use embedded_hal::prelude::*;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, InputPin};
use embedded_time::{fixed_point::FixedPoint, rate::Hertz};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let _spi_cipo = pins.gpio16.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let _spi_clk = pins.gpio18.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let _spi_copi = pins.gpio19.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let bmc_irq = pins.gpio20.into_pull_up_input();
    let mut spi_cs = pins.gpio17.into_push_pull_output();

    // | Pin | Name | Function |
    // | --- | ---- | -------- |
    // | 21  | GP16 | SPI_CIPO |
    // | 22  | GP17 | SPI_CS   |
    // | 24  | GP18 | SPI_CLK  |
    // | 25  | GP19 | SPI_COPI |
    // | 26  | GP20 | nIRQ_IN  |

    let mut spi: bsp::hal::Spi<bsp::hal::spi::Enabled, pac::SPI0, 8> = bsp::hal::Spi::new(pac.SPI0)
        .init(
            &mut pac.RESETS,
            clocks.peripheral_clock,
            Hertz::new(8_000_000),
            &embedded_hal::spi::MODE_1,
        );

    loop {
        for data in 0..=255 {
            info!("on!");
            led_pin.set_high().unwrap();
            let out_buffer = [4, data, data + 1, data + 2, data + 3];
            info!("Sending {:?}", out_buffer);
            spi_cs.set_low().unwrap();
            let _ = spi.write(&out_buffer);
            // A brief turnaround pause
            delay.delay_us(5);           
            // Get back four words
            let mut in_buffer = [0xFF; 4];
            let _ = spi.transfer(&mut in_buffer);
            spi_cs.set_high().unwrap();
            info!("Got {:?}", in_buffer);
            for (inb, out) in in_buffer.iter().zip(&out_buffer[1..]) {
                if inb.wrapping_add(128) != *out {
                    defmt::panic!("Bad value {} != {}", inb, out);
                }
            }
            delay.delay_ms(50);
            led_pin.set_low().unwrap();
            delay.delay_ms(50);
        }
    }
}

// End of file
