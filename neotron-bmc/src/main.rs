#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

use stm32f3xx_hal as f3hal;

use f3hal::{
    pac,
    prelude::*,
    rcc::{Enable, Reset},
};

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    // Set up the system clock.
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

    defmt::info!("This is Neotron BMC");
    defmt::debug!("hclk = {} Hz", clocks.hclk().0);
    defmt::debug!("pclk1 = {} Hz", clocks.pclk1().0);
    defmt::debug!("pclk2 = {} Hz", clocks.pclk2().0);
    defmt::debug!("sysclk = {} Hz", clocks.sysclk().0);

    // We need Alternate Mode 5, to connect the pins to SPI2
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let _spi2_ssel =
        gpiob
            .pb12
            .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let _spi2_sclk: f3hal::gpio::Pin<_, _, f3hal::gpio::Alternate<f3hal::gpio::PushPull, 5>> =
        gpiob
            .pb13
            .into_af_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let _spi2_cipo: f3hal::gpio::Pin<_, _, f3hal::gpio::Alternate<f3hal::gpio::PushPull, 5>> =
        gpiob
            .pb14
            .into_af_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let _spi2_copi: f3hal::gpio::Pin<_, _, f3hal::gpio::Alternate<f3hal::gpio::PushPull, 5>> =
        gpiob
            .pb15
            .into_af_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let mut _irq = gpiob
        .pb11
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let mode = f3hal::hal::spi::MODE_1;

    // TODO: Call the PAC methods manually, as I don't think STM32F4 SPI
    // Peripheral mode works. Or at least, it only seems to work with DMA
    // enabled. See https://github.com/stm32-rs/stm32f4xx-hal/blob/master/examples/spi_slave_dma_rtic.rs.

    pac::SPI2::enable(&mut rcc.apb1);
    pac::SPI2::reset(&mut rcc.apb1);

    // What we /expect/ to be clocked at
    let spi_clock_speed = 8_000_000;

    defmt::info!(
        "pclk1 = {}, spi_clock = {}",
        clocks.pclk1().0,
        spi_clock_speed
    );

    // We are following DM00043574, Section 30.5.1 Configuration of SPI

    // 2. Write to the SPI_CR1 register
    dp.SPI2.cr1.write(|w| {
        // 2a. Configure the serial clock baud rate
        match clocks.pclk1().0 / spi_clock_speed {
            0 => unreachable!(),
            1..=2 => {
                w.br().div2();
            }
            3..=5 => {
                w.br().div4();
            }
            6..=11 => {
                w.br().div8();
            }
            12..=23 => {
                w.br().div16();
            }
            24..=47 => {
                w.br().div32();
            }
            48..=95 => {
                w.br().div64();
            }
            96..=191 => {
                w.br().div128();
            }
            _ => {
                w.br().div256();
            }
        }
        // 2b. Configure the CPHA and CPOL bits
        if mode.phase == f3hal::spi::Phase::CaptureOnSecondTransition {
            w.cpha().second_edge();
        } else {
            w.cpha().first_edge();
        }
        if mode.polarity == f3hal::spi::Polarity::IdleHigh {
            w.cpol().idle_high();
        } else {
            w.cpol().idle_low();
        }
        // 2c. Select simplex or half-duplex mode (nope, neither of those)
        w.rxonly().clear_bit();
        w.bidimode().clear_bit();
        w.bidioe().clear_bit();
        // 2d. Configure the LSBFIRST bit to define the frame format
        w.lsbfirst().clear_bit();
        // 2e. Configure the CRCL and CRCEN bits if CRC is needed (it is not)
        w.crcen().disabled();
        // 2f. Configure SSM and SSI
        w.ssm().disabled();
        w.ssi().slave_selected();
        // 2g. Configure the MSTR bit. Apologies for the outdated terminology.
        w.mstr().slave();
        w
    });

    // 3. Write to SPI_CR2 register
    dp.SPI2.cr2.write(|w| {
        // 3a. Configure the DS[3:0] bits to select the data length for the transfer.
        unsafe { w.ds().bits(0b111) };
        // 3b. Configure SSOE
        w.ssoe().disabled();
        // 3c. Frame Format
        w.frf().motorola();
        // 3d. Set NSSP bit if required (we don't want NSS Pulse mode)
        w.nssp().no_pulse();
        // 3e. Configure the FRXTH bit.
        w.frxth().quarter();
        // 3f. LDMA_TX and LDMA_RX for DMA mode - not used
        w
    });

    // 4. SPI_CRCPR - not required

    // 5. DMA registers - not required

    // Finally, enable SPI
    dp.SPI2.cr1.modify(|_r, w| {
        w.spe().enabled();
        w
    });

    let mut spi = SpiInPeripheralMode { spi: dp.SPI2 };

    let mut buffer = [0u8; 32];
    loop {

        // spin waiting for frame to end
        while _spi2_ssel.is_low().unwrap() {
        }

        defmt::debug!("end");

        // Empty FIFO
        while spi.has_rx_data() {
            let _ = spi.read();
        }

        // spin waiting for frame to start
        while _spi2_ssel.is_high().unwrap() {
        }

        // Get Packet Length
        let packet_len = spi.blocking_read() as usize;

        if packet_len > buffer.len() {
            defmt::debug!("{}", packet_len);
            continue;
        }

        // Grab data
        for slot in buffer[0..packet_len].iter_mut() {
            *slot = spi.blocking_read();
        }

        // Send data
        for item in buffer[0..packet_len].iter() {
            let value = (*item).wrapping_add(128);
            spi.blocking_write(value);
        }

        if spi.has_overrun() {
            defmt::panic!("overrun");
        }

        if spi.has_underrun() {
            defmt::panic!("underrun");
        }

        defmt::info!("{} {:?}", packet_len, buffer[0..packet_len]);
    }
}

struct SpiInPeripheralMode {
    spi: f3hal::pac::SPI2
}

impl SpiInPeripheralMode {
    fn has_rx_data(&self) -> bool {
        self.spi.sr.read().rxne().is_not_empty()
    }

    fn has_tx_space(&self) -> bool {
        self.spi.sr.read().txe().is_empty()
    }

    fn read(&mut self) -> u8 {
        let p = self.spi.dr.as_ptr() as *const u8;
        unsafe { p.read_volatile() }
    }

    fn blocking_read(&mut self) -> u8 {
        while !self.has_rx_data() {
            // spin
        }
        self.read()
    }

    fn write(&mut self, data: u8) {
        let p = self.spi.dr.as_ptr() as *mut u8;
        unsafe { p.write_volatile(data) }
    }

    fn blocking_write(&mut self, data: u8) {
        while !self.has_tx_space() {
            // spin
        }
        self.write(data)
    }

    fn has_overrun(&self) -> bool {
        self.spi.sr.read().ovr().is_overrun()
    }

    fn has_underrun(&self) -> bool {
        self.spi.sr.read().udr().is_underrun()
    }
}