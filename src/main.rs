//! Raspberry Pi Pico and Wiznet W5500 experiment.
//!
//! Initially adapted from https://github.com/newAM/ambientsensor-rs
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::fmt;
use core::marker::PhantomData;
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
};
use systick_monotonic::Systick;
use w5500_dhcp::{
    hl::{Hostname, Tcp, Udp, io::Read, io::Write},
    ll::{
        eh0::{reset, vdm_infallible_gpio::W5500, MODE as W5500_MODE},
        net::Eui48Addr, net::SocketAddrV4,
        LinkStatus, OperationMode, PhyCfg, Registers, Sn, SOCKETS, SocketInterruptMask,
    },
    Client as DhcpClient,
};
use usecop::proto::Timestamp;

mod node;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const SECOP_SN: [Sn; 6] = [Sn::Sn0, Sn::Sn1, Sn::Sn2, Sn::Sn3, Sn::Sn4, Sn::Sn5];
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
        node: node::SecNode,
        dhcp: DhcpClient<'static>,
        dhcp_spawn_at: Option<u32>,
        ntp_time: Option<f64>,
    }

    #[local]
    struct Local {
        irq_pin: Pin<Gpio14, FunctionSio<SioInput>, PullDown>,
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
        let sio = hal::sio::Sio::new(dp.SIO);
        let pins = hal::gpio::Pins::new(dp.IO_BANK0, dp.PADS_BANK0, sio.gpio_bank0, &mut dp.RESETS);

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
    #[task(binds = IO_IRQ_BANK0, local = [irq_pin], shared = [w5500, node])]
    fn irq_bank0(mut cx: irq_bank0::Context) {
        debug!("[W5500] got interrupt");

        cx.shared.w5500.lock(|w5500| {
            let sir: u8 = w5500.sir().unwrap();

            cx.local.irq_pin.clear_interrupt(Interrupt::EdgeLow);

            // may occur when there are power supply issues
            if sir == 0 {
                warn!("[W5500] spurious interrupt");
                return;
            }

            if sir & DHCP_SN.bitmask() != 0 {
                let sn_ir = w5500.sn_ir(DHCP_SN).unwrap();
                let _ = w5500.set_sn_ir(DHCP_SN, sn_ir);
                if dhcp_client::spawn().is_err() {
                    error!("DHCP task already spawned")
                }
            }

            if sir & NTP_SN.bitmask() != 0 {
                let sn_ir = w5500.sn_ir(NTP_SN).unwrap();
                let _ = w5500.set_sn_ir(NTP_SN, sn_ir);
                if ntp_client::spawn().is_err() {
                    error!("NTP task already spawned")
                }
            }

            for sn in SECOP_SN {
                if sir & sn.bitmask() != 0 {
                    let sn_ir = w5500.sn_ir(sn).unwrap();
                    let _ = w5500.set_sn_ir(sn, sn_ir);

                    if sn_ir.discon_raised() {
                        info!("[SECoP] client disconnected");
                        cx.shared.node.lock(|node| node.client_finished(sn as usize));
                        w5500.tcp_listen(sn, SECOP_PORT).unwrap();
                    } else if sn_ir.con_raised() {
                        info!("[SECoP] client connected");
                        cx.shared.node.lock(|node| node.client_connected(sn as usize));
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
    #[task(shared = [w5500, node, ntp_time], capacity = 5)]
    fn secop_request(mut cx: secop_request::Context, sn: Sn) {
        let time = get_time(cx.shared.ntp_time);
        // TODO: this doesn't consider fragmented requests
        cx.shared.w5500.lock(|w5500| {
            let mut msg_len = 0;
            let mut buf = [0; 1024];
            if let Ok(mut reader) = w5500.tcp_reader(sn) {
                msg_len = reader.read(&mut buf).unwrap_or(0) as usize;
                reader.done().unwrap();
            } else {
                error!("[SECoP] failed to get reader");
            }
            if msg_len == 0 {
                return;
            }
            info!("[SECoP] incoming: {:?}",
                  core::str::from_utf8(&buf[..msg_len]).map_err(|_| ()));
            cx.shared.node.lock(|node| node.process(
                time, &buf[..msg_len], sn as usize,
                |sn, callback: &dyn Fn(&mut dyn fmt::Write)| {
                    if let Ok(writer) = w5500.tcp_writer(SOCKETS[sn]) {
                        let mut wrap = WriterWrap(writer, PhantomData);
                        callback(&mut wrap);
                        wrap.0.send().unwrap();
                    }
                }
            ));
        })
    }

    /// SECoP regular polling task
    #[task(shared = [w5500, node, ntp_time])]
    fn secop_poll(cx: secop_poll::Context) {
        let time = get_time(cx.shared.ntp_time);
        (cx.shared.w5500, cx.shared.node).lock(|w5500, node| {
            node.poll(time, |sn, callback: &dyn Fn(&mut dyn fmt::Write)| {
                if let Ok(writer) = w5500.tcp_writer(SOCKETS[sn]) {
                    let mut wrap = WriterWrap(writer, PhantomData);
                    callback(&mut wrap);
                    wrap.0.send().unwrap();
                }
            });
        });
        secop_poll::spawn_after(500.millis()).unwrap();
    }
}

struct WriterWrap<E, T: Write<E>>(T, PhantomData<E>);

impl<E, T: Write<E>> fmt::Write for WriterWrap<E, T> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.0.write(s.as_bytes()).map_err(|_| fmt::Error).map(|_| ())
    }
}
