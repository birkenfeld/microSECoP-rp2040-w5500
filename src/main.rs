//! Raspberry Pi Pico and Wiznet W5500 experiment.
//!
//! Initially adapted from https://github.com/newAM/ambientsensor-rs
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use embedded_hal::digital::v2::OutputPin;
use fugit::{ExtU64, RateExtU32};
use rp_pico::hal::{
    self,
    adc::Adc,
    gpio::{bank0::*, FunctionSio, FunctionSpi, Interrupt, Pin, PullDown,
           SioOutput, SioInput},
    pac::SPI1,
    spi::{Enabled, Spi},
    Clock,
    multicore::{Multicore, Stack},
};
use systick_monotonic::Systick;
use w5500_dhcp::{
    hl::{Hostname, Tcp, Udp, io::Read},
    ll::{
        SocketCommand,
        eh0::{reset, vdm_infallible_gpio::W5500, MODE as W5500_MODE},
        net::Eui48Addr, net::SocketAddrV4,
        LinkStatus, OperationMode, PhyCfg, Registers, Sn, SOCKETS, SocketInterruptMask,
    },
    Client as DhcpClient,
};
use usecop::{ClientId, Timestamp};

mod node;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const N_CONN: usize = 6;
const BUF_SIZE: usize = 1024;

const SECOP_SN: [Sn; N_CONN] = [Sn::Sn0, Sn::Sn1, Sn::Sn2, Sn::Sn3, Sn::Sn4, Sn::Sn5];
const DHCP_SN: Sn = Sn::Sn6;
const NTP_SN: Sn = Sn::Sn7;
const IRQ_MASK: SocketInterruptMask = SocketInterruptMask::ALL_MASKED
    .unmask_recv().unmask_con().unmask_discon();

const NAME: &str = "pinode";
const HOSTNAME: Hostname<'static> = Hostname::new_unwrapped(NAME);
const SECOP_PORT: u16 = 10767;
const TIME_GRANULARITY: u32 = 10;  // 10 Hz / 0.1 s granularity

const NTP_PORT: u16 = 123;
// Simple packet requesting current timestamp
const NTP_REQUEST: &[u8] = b"\xE3\x00\x06\xEC\x00\x00\x00\x00\x00\x00\x00\x00\
                             \x31\x4E\x31\x34\x00\x00\x00\x00\x00\x00\x00\x00\
                             \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\
                             \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

fn monotonic_secs() -> u32 {
    app::monotonics::now()
        .duration_since_epoch()
        .to_secs()
        .try_into()
        .unwrap()
}

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task() -> () {
    loop {
        cortex_m::asm::delay(10);
    }
}

type MyW5500 = W5500<Spi<Enabled, SPI1, (Pin<Gpio11, FunctionSpi, PullDown>,
                                         Pin<Gpio12, FunctionSpi, PullDown>,
                                         Pin<Gpio10, FunctionSpi, PullDown>), 8>,
                     Pin<Gpio13, FunctionSio<SioOutput>, PullDown>>;

#[rtic::app(
    device = crate::hal::pac,
    dispatchers = [UART0_IRQ, UART1_IRQ],
)]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type Time = Systick<TIME_GRANULARITY>;

    #[shared]
    struct Shared {
        w5500: MyW5500,
        node: node::SecNode<N_CONN>,
        dhcp: DhcpClient<'static>,
        dhcp_spawn_at: Option<u32>,
        ntp_time: Option<f64>,
    }

    #[local]
    struct Local {
        irq_pin: Pin<Gpio14, FunctionSio<SioInput>, PullDown>,
        buffer: [[u8; BUF_SIZE]; N_CONN],
        filled: [usize; N_CONN],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Setup clocks and the watchdog.
        let mut dp = cx.device;
        let mut watchdog = hal::watchdog::Watchdog::new(dp.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            dp.XOSC,
            dp.CLOCKS,
            dp.PLL_SYS,
            dp.PLL_USB,
            &mut dp.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Setup the pins.
        let mut sio = hal::sio::Sio::new(dp.SIO);
        let pins = hal::gpio::Pins::new(dp.IO_BANK0, dp.PADS_BANK0, sio.gpio_bank0, &mut dp.RESETS);

        // Other init code above this line
        let mut mc = Multicore::new(&mut dp.PSM, &mut dp.PPB, &mut sio.fifo);
        let cores = mc.cores();
        let core1 = &mut cores[1];
        let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, core1_task);

        let systick = cx.core.SYST;
        let mut delay = cortex_m::delay::Delay::new(systick, clocks.system_clock.freq().to_Hz());

        let mut w5500 = {
            let cs = pins.gpio13.into_push_pull_output();
            let mosi = pins.gpio11.into_function::<hal::gpio::FunctionSpi>();
            let miso = pins.gpio12.into_function::<hal::gpio::FunctionSpi>();
            let sclk = pins.gpio10.into_function::<hal::gpio::FunctionSpi>();
            let spi1 = hal::spi::Spi::<_, _, _, 8>::new(dp.SPI1, (mosi, miso, sclk));
            let spi1 = spi1.init(
                &mut dp.RESETS,
                clocks.peripheral_clock.freq(),
                1_000_000u32.Hz(),
                &W5500_MODE,
            );
            W5500::new(spi1, cs)
        };

        let mut w5500_rst = pins.gpio15.into_push_pull_output();
        let w5500_int = pins.gpio14.into_pull_down_input();
        w5500_int.set_interrupt_enabled(Interrupt::EdgeLow, true);

        info!("Initialized");

        let mac = Eui48Addr::new(0x46, 0x52, 0x4d, 0x01, 0x02, 0x03);

        reset(&mut w5500_rst, &mut delay).unwrap();

        // continually initialize the W5500 until we link up
        'outer: loop {
            // sanity check W5500 communications
            core::assert_eq!(w5500.version().unwrap(), w5500_dhcp::ll::VERSION);

            // load the MAC address
            w5500.set_shar(&mac).unwrap();
            core::debug_assert_eq!(w5500.shar().unwrap(), mac);

            // wait for the PHY to indicate the Ethernet link is up
            let mut attempts: u32 = 0;
            info!("Polling for link up");
            const PHY_CFG: PhyCfg = PhyCfg::DEFAULT.set_opmdc(OperationMode::Auto);
            w5500.set_phycfgr(PHY_CFG).unwrap();

            const LINK_UP_POLL_PERIOD_MILLIS: u32 = 100;
            const LINK_UP_POLL_ATTEMPTS: u32 = 50;
            loop {
                let phy_cfg: PhyCfg = w5500.phycfgr().unwrap();
                if phy_cfg.lnk() == LinkStatus::Up {
                    break 'outer;
                }
                if attempts >= LINK_UP_POLL_ATTEMPTS {
                    info!(
                        "Failed to link up in {} ms",
                        attempts * LINK_UP_POLL_PERIOD_MILLIS,
                    );
                    break;
                }
                delay.delay_ms(LINK_UP_POLL_PERIOD_MILLIS);
                attempts += 1;
            }

            w5500_rst.set_low().unwrap();
            delay.delay_ms(1);
            w5500_rst.set_high().unwrap();
            delay.delay_ms(3);
        }
        info!("Done link up");

        let seed: u64 = u64::from(cortex_m::peripheral::SYST::get_current()) << 32
            | u64::from(cortex_m::peripheral::SYST::get_current());

        // additional delay seems to be required until DHCP request can be sent
        delay.delay_ms(500);

        let dhcp = DhcpClient::new(DHCP_SN, seed, mac, HOSTNAME);
        dhcp.setup_socket(&mut w5500).unwrap();

        let mut simr = w5500.simr().unwrap();
        simr |= NTP_SN.bitmask();
        w5500.set_sn_imr(NTP_SN, IRQ_MASK).unwrap();
        w5500.udp_bind(NTP_SN, 12345).unwrap();

        for sn in SECOP_SN {
            simr |= sn.bitmask();
            w5500.set_sn_imr(sn, IRQ_MASK).unwrap();
            // yes, all sockets can listen on the same port :)
            w5500.tcp_listen(sn, SECOP_PORT).unwrap();
        }

        w5500.set_simr(simr).unwrap();

        // start the DHCP client
        dhcp_client::spawn().unwrap();

        // start the timeout tracker
        timeout_tracker::spawn().unwrap();

        // start the SECoP polling task
        secop_poll::spawn_after(500.millis()).unwrap();

        // use systick for monotonic clock now
        let mono = Systick::new(delay.free(), clocks.system_clock.freq().to_Hz());

        let mut adc = Adc::new(dp.ADC, &mut dp.RESETS);
        let sensor = adc.take_temp_sensor().unwrap();
        let node = node::create(adc, sensor);

        (
            Shared {
                w5500,
                node,
                dhcp,
                dhcp_spawn_at: None,
                ntp_time: None,
            },
            Local {
                irq_pin: w5500_int,
                buffer: [[0; BUF_SIZE]; N_CONN],
                filled: [0; N_CONN],
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            compiler_fence(SeqCst);
        }
    }

    /// IRQ handler for the W5500, dispatching tasks by socket
    #[task(binds = IO_IRQ_BANK0, local = [irq_pin], shared = [w5500, node], priority = 3)]
    fn irq_bank0(mut cx: irq_bank0::Context) {
        cx.local.irq_pin.clear_interrupt(Interrupt::EdgeLow);

        debug!("[W5500] got interrupt");

        cx.shared.w5500.lock(|w5500| {
            let sir: u8 = w5500.sir().unwrap();

            // may occur when there are power supply issues
            if sir == 0 {
                warn!("[W5500] spurious interrupt");
                return;
            }

            if sir & DHCP_SN.bitmask() != 0 {
                let sn_ir = w5500.sn_ir(DHCP_SN).unwrap();
                w5500.set_sn_ir(DHCP_SN, sn_ir).unwrap();
                if dhcp_client::spawn().is_err() {
                    error!("DHCP task already spawned")
                }
            }

            if sir & NTP_SN.bitmask() != 0 {
                let sn_ir = w5500.sn_ir(NTP_SN).unwrap();
                w5500.set_sn_ir(NTP_SN, sn_ir).unwrap();
                if ntp_client::spawn().is_err() {
                    error!("NTP task already spawned")
                }
            }

            for sn in SECOP_SN {
                if sir & sn.bitmask() != 0 {
                    let sn_ir = w5500.sn_ir(sn).unwrap();
                    w5500.set_sn_ir(sn, sn_ir).unwrap();

                    if sn_ir.discon_raised() {
                        info!("[SECoP] client disconnected");
                        cx.shared.node.lock(|node| node.client_finished(sn as ClientId));
                        w5500.tcp_listen(sn, SECOP_PORT).unwrap();
                    }
                    if sn_ir.con_raised() {
                        info!("[SECoP] client connected");
                        cx.shared.node.lock(|node| node.client_connected(sn as ClientId));
                        let _ = secop_request::spawn_after(100.millis(), sn);
                    }
                    if sn_ir.recv_raised() {
                        if secop_request::spawn(sn).is_err() {
                            error!("SECoP task already spawned")
                        }
                    }
                }
            }
        });
    }

    /// Task to spawn DHCP actions when necessary
    #[task(shared = [dhcp_spawn_at])]
    fn timeout_tracker(mut cx: timeout_tracker::Context) {
        timeout_tracker::spawn_after(1.secs()).unwrap();

        let now: u32 = monotonic_secs();

        cx.shared.dhcp_spawn_at.lock(|dhcp_spawn_at| {
            if let Some(then) = dhcp_spawn_at {
                if now >= *then {
                    if dhcp_client::spawn().is_err() {
                        error!("DHCP task already spawned")
                    }
                    *dhcp_spawn_at = None;
                }
            }
        });
    }

    /// DHCP client task
    #[task(shared = [w5500, dhcp, dhcp_spawn_at])]
    fn dhcp_client(cx: dhcp_client::Context) {
        (cx.shared.w5500, cx.shared.dhcp, cx.shared.dhcp_spawn_at).lock(
            |w5500, dhcp, dhcp_spawn_at| {
                let now = monotonic_secs();
                let spawn_after_secs = match dhcp.process(w5500, now) {
                    Ok(sec) => sec,
                    Err(e) => {
                        error!("[DHCP] error {}", e);
                        5
                    }
                };

                if let Some(ip) = dhcp.ntp() {
                    info!("[NTP] requesting time from {}", ip);
                    let addr = SocketAddrV4::new(ip, NTP_PORT);
                    w5500.udp_send_to(NTP_SN, NTP_REQUEST, &addr).unwrap();
                }

                let spawn_at = now + spawn_after_secs;
                *dhcp_spawn_at = Some(spawn_at);
                info!("[DHCP] spawning after {} seconds, at {}",
                      spawn_after_secs, spawn_at);
            },
        )
    }

    /// NTP client task for getting absolute time
    #[task(shared = [w5500, ntp_time])]
    fn ntp_client(cx: ntp_client::Context) {
        let now = monotonics::Time::now().ticks() as f64 / TIME_GRANULARITY as f64;
        let mut buf = [0; 44];  // we need bytes 40-44 only
        (cx.shared.w5500, cx.shared.ntp_time).lock(|w5500, time| {
            w5500.udp_recv_from(NTP_SN, &mut buf).unwrap();
            let stamp = u32::from_be_bytes([buf[40], buf[41], buf[42], buf[43]]);
            // conversion to Unix time, differs by seventy years
            let stamp = stamp - 2208988800;
            defmt::info!("[NTP] got timestamp {}", stamp);
            *time = Some(stamp as f64 - now);
        });
    }

    fn get_time(mut epoch: impl rtic::Mutex<T=Option<f64>>) -> Timestamp {
        let ticks = monotonics::now().ticks() as f64 / TIME_GRANULARITY as f64;
        if let Some(epoch) = epoch.lock(|v| *v) {
            Timestamp::Abs(epoch + ticks)
        } else {
            Timestamp::Rel(ticks)
        }
    }

    /// SECoP request server task
    #[task(shared = [w5500, node, ntp_time], local = [buffer, filled], capacity = 6)]
    fn secop_request(mut cx: secop_request::Context, sn: Sn) {
        let time = get_time(cx.shared.ntp_time);
        cx.shared.w5500.lock(|w5500| {
            let buf = &mut cx.local.buffer[sn as usize];
            let filled = &mut cx.local.filled[sn as usize];

            let Ok(mut reader) = w5500.tcp_reader(sn) else {
                // Nothing to read, retry later
                let _ = secop_request::spawn_after(100.millis(), sn);
                return;
            };
            let n_recvd = reader.read(&mut buf[*filled..]).unwrap_or(0) as usize;
            info!("[SECoP] read {} bytes", n_recvd);
            *filled += n_recvd;
            reader.done().unwrap();

            if *filled == BUF_SIZE {
                error!("[SECoP] buffer overflow");
                w5500.tcp_disconnect(sn).unwrap();
                return;
            }

            let result = cx.shared.node.lock(|node| node.process(
                time, &mut buf[..*filled], sn as ClientId,
                |sn, callback: &dyn Fn(&mut dyn usecop::io::Write)| {
                    let mut wrap = Writer::new(w5500, SOCKETS[sn]).unwrap();
                    callback(&mut wrap);
                    wrap.send().unwrap();
                }
            ));

            match result {
                Err(_) => {
                    info!("[SECoP] error, disconnect");
                    w5500.tcp_disconnect(sn).unwrap();
                    return;
                }
                Ok(0) => {
                    debug!("[SECoP] no data processed, wait for more");
                }
                Ok(n_processed) => {
                    debug!("[SECoP] processed {} bytes", n_processed);
                    buf.copy_within(n_processed.., 0);
                    *filled -= n_processed;
                }
            }
            // Try to read more later
            let _ = secop_request::spawn_after(100.millis(), sn);
        })
    }

    /// SECoP regular polling task
    #[task(shared = [w5500, node, ntp_time])]
    fn secop_poll(cx: secop_poll::Context) {
        let time = get_time(cx.shared.ntp_time);
        (cx.shared.w5500, cx.shared.node).lock(|w5500, node| {
            node.poll(time, |sn, callback: &dyn Fn(&mut dyn usecop::io::Write)| {
                let mut wrap = Writer::new(w5500, SOCKETS[sn]).unwrap();
                callback(&mut wrap);
                wrap.send().unwrap();
            });
        });
        secop_poll::spawn_after(500.millis()).unwrap();
    }
}

// Mostly reimplemented from w5500's TcpWriter to send inbetween packets
// if the buffer is full.

struct Writer<'w, W5500> {
    w5500: &'w mut W5500,
    sn: Sn,
    tail_ptr: u16,
    ptr: u16,
}

impl<'w, W5500: Registers> Writer<'w, W5500> {
    fn new(w5500: &'w mut W5500, sn: Sn) -> Result<Self, W5500::Error> {
        let tx_ptrs = w5500.sn_tx_ptrs(sn)?;
        let ptr = tx_ptrs.wr;
        let tail_ptr = tx_ptrs.wr.wrapping_add(tx_ptrs.fsr);
        Ok(Self { w5500, sn, tail_ptr, ptr })
    }

    fn remain(&self) -> u16 {
        self.tail_ptr.wrapping_sub(self.ptr)
    }

    fn send(&mut self) -> Result<(), W5500::Error> {
        self.w5500.set_sn_tx_wr(self.sn, self.ptr)?;
        self.w5500.set_sn_cr(self.sn, SocketCommand::Send)?;
        // Wait for send to complete. Hack, should use the interrupt instead...
        loop {
            let tx_ptrs = self.w5500.sn_tx_ptrs(self.sn)?;
            if tx_ptrs.fsr != 0 {
                self.ptr = tx_ptrs.wr;
                self.tail_ptr = tx_ptrs.wr.wrapping_add(tx_ptrs.fsr);
                return Ok(());
            }
        }
    }
}

impl<'w, W5500: Registers> usecop::io::Write for Writer<'w, W5500> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, usecop::io::Error> {
        let write_size: u16 = self.remain().min(buf.len() as u16);
        if write_size != 0 {
            self.w5500
                .set_sn_tx_buf(self.sn, self.ptr, &buf[..usize::from(write_size)])
                .map_err(|_| usecop::io::Error::new(usecop::io::ErrorKind::BrokenPipe, ""))?;
            self.ptr = self.ptr.wrapping_add(write_size);
        }
        if write_size < buf.len() as u16 {
            self.send().map_err(|_| usecop::io::Error::new(usecop::io::ErrorKind::BrokenPipe, ""))?;
        }
        Ok(write_size as usize)
    }

    fn flush(&mut self) -> Result<(), usecop::io::Error> {
        Ok(())
    }
}
