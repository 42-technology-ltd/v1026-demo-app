//! V1026 Rust IoT Demo Application
//!
//! This is a demo application for the Actinius Icarus. See the [README.md]
//! file for more details.
//!
//! Copyright (c) 42 Technology Ltd 2019
//! All rights reserved.

#![no_std]
#![no_main]
#![allow(deprecated)]
#![feature(alloc_error_handler)]
#![feature(new_uninit)]

// ==========================================================================
//
// Modules and Crates
//
// ==========================================================================

#[macro_use]
extern crate alloc;

extern crate actinius_icarus_bsp as bsp;
extern crate cortex_m_rt as rt;

mod secrets;

// ==========================================================================
//
// Imports
//
// ==========================================================================

use core::fmt::Write;
use core::panic::PanicInfo;

use bsp::pac::interrupt;
use bsp::prelude::*;
use rt::entry;

use log::{debug, error, info, warn, Level, Metadata, Record};

use alloc_cortex_m::CortexMHeap;

// ==========================================================================
//
// Private Types
//
// ==========================================================================

/// Describes an instant in time. The system only supports local time and has
/// no concept of time zones.
#[repr(C)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Timestamp {
	/// The Gregorian calendar year, minus 1970 (so 10 is 1980, and 30 is the year 2000)
	pub year_from_1970: u8,
	/// The month of the year, where January is 1 and December is 12
	pub month: u8,
	/// The day of the month where 1 is the first of the month, through to 28,
	/// 29, 30 or 31 (as appropriate)
	pub days: u8,
	/// The hour in the day, from 0 to 23
	pub hours: u8,
	/// The minutes past the hour, from 0 to 59
	pub minutes: u8,
	/// The seconds past the minute, from 0 to 59. Note that some filesystems
	/// only have 2-second precision on their timestamps.
	pub seconds: u8,
}

/// Represents the seven days of the week
#[repr(C)]
#[derive(Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum DayOfWeek {
	/// First day of the week
	Monday,
	/// Comes after Monday
	Tuesday,
	/// Middle of the week
	Wednesday,
	/// Between Wednesday and Friday
	Thursday,
	/// Almost the weekend
	Friday,
	/// First day of the weekend
	Saturday,
	/// Last day of the week
	Sunday,
}

/// An event we can log
#[derive(Debug)]
enum LogEvent {
	/// GPS Result
	GpsFix {
		/// Time to first fix, in 100ms units
		time: u16,
		/// Greatest number of SVs seen during TTFF
		max_svs: u8,
		/// A fix, no fix, or an error
		result: Result<Option<Position>, Error>,
	},
	/// Temperature in degrees C
	Temp(i32),
	/// Battery Voltage in Volts
	Vbat(f32),
	/// Registration Result
	Registration {
		/// true if we registered, false if we timed out, or an error
		result: Result<bool, Error>,
		/// A copy of the signal status from AT%XMONITOR
		signal_status: heapless::String<heapless::consts::U128>,
	},
	/// Successful TCP connection made
	// Connect(Result<(), Error>),
	/// Successful HTTP POST completed
	Uploaded,
}

/// Records a particular event occurring at a particular time (represented by our connection index)
#[derive(Debug)]
struct LogRecord {
	index: u32,
	event: LogEvent,
}

/// Our application is state-machine based
#[derive(Debug)]
enum ProgramState {
	// Finding a GNSS fix
	FindingFix,
	// Start the LTE, register, connect to the cloud and upload the fix/temp/voltage.
	UploadingPayload(Payload),
	// A keypress was detected, so we're now running the console
	Console,
	// The unit is asleep
	Asleep,
}

/// Our menu system holds a context object for us, of this type.
struct Context {
	delay_timer: bsp::hal::Timer<bsp::pac::TIMER0_NS>,
	function_timer: bsp::hal::Timer<bsp::pac::TIMER1_NS>,
	adc: bsp::hal::Saadc,
	vbat_pin: bsp::hal::gpio::p0::P0_13<bsp::hal::gpio::Input<bsp::hal::gpio::Floating>>,
	state: ProgramState,
	index: u32,
	leds: bsp::Leds,
	#[allow(dead_code)]
	rtc: bsp::hal::Rtc<bsp::pac::RTC1_NS, bsp::hal::rtc::Started>,
	naps_left: u32,
	usual_total_naps: u32,
}

/// Identifies in which way a particular function call failed
#[derive(Debug, Clone)]
enum Error {
	// ConnectionFail,
	Library(nrfxlib::Error),
	Quiche(quiche::Error),
	Read,
	Write,
	Timeout,
	BadParameters,
}

/// Describes a 2D position on the surface of the earth
#[derive(Debug, Clone)]
struct Position {
	latitude: f64,
	longitude: f64,
}

/// A packet of data we can upload to the cloud
#[derive(Debug, Clone)]
struct Payload {
	/// A possible GNSS fix
	fix: Option<Position>,
	/// SiP temperature in degrees C
	temp: i32,
	/// Battery voltage in Volts
	vbat: f32,
	/// Cell tower info
	signal_status: heapless::String<heapless::consts::U128>,
}

/// Empty holder for our logging functionality.
struct SimpleLogger;

/// Our version of `time_t`. It's a 64-bit value.
type CTime = i64;

/// Our version of `struct timeval`.
#[repr(C)]
struct Timeval {
	tv_sec: CTime,
	tv_usec: i32,
}

/// Our version of `struct timezone`.
#[repr(C)]
struct Timezone {
	tz_minuteswest: i32,
	tz_dsttime: i32,
}

/// Tracks the most recent date/time stamp, and the tick count at which we
/// calculated that date/time stamp. Whenever we ask for the time, we move the
/// date/time stamp forwards based on how many ticks had elapsed since we
/// last asked.
struct TimeContextInner {
	/// The tick count at which `timestamp` was calculated.
	timestamp_tick_count: u64,
	/// The calendar date / time at `timestamp_tick_count`.
	timestamp: Timestamp,
}

struct TimeContext {
	inner: spin::Mutex<TimeContextInner>,
}

//Mode for quiche usage
#[derive(PartialEq)]
enum QuicheMode {
	SERIAL,
	RADIO,
	INVALID_MODE,
}
// ==========================================================================
//
// Private Global Data
//
// ==========================================================================

/// Rate at which RTC ticks
const RTC_TICK_RATE: u64 = 32768;

/// Tracks how many times the RTC has overflowed. At 32768 Hz this is every 512 seconds.
static TIMER_OVERFLOWS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Tracks the current system time in a race-hazard safe way.
static TIME_CONTEXT: TimeContext = TimeContext {
	inner: spin::Mutex::new(TimeContextInner {
		timestamp_tick_count: 0,
		timestamp: Timestamp {
			year_from_1970: 0,
			month: 1,
			days: 1,
			hours: 0,
			minutes: 0,
			seconds: 0,
		},
	}),
};

/// A global system log, which is a fixed-size FIFO of logging objects.
static mut SYSTEM_LOG: heapless::spsc::Queue<LogRecord, heapless::consts::U32> =
	heapless::spsc::Queue(heapless::i::Queue::new());

/// This is the main menu
static ROOT_MENU: menu::Menu<Context> = menu::Menu {
	label: "root",
	items: &[
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_stop,
				parameters: &[],
			},
			command: "stop",
			help: Some("Stop the main loop"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_start,
				parameters: &[],
			},
			command: "start",
			help: Some("Start the main loop"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_mode,
				parameters: &[
					menu::Parameter::Named {
						parameter_name: "ltem",
						help: Some("Enables LTE-M"),
					},
					menu::Parameter::Named {
						parameter_name: "nbiot",
						help: Some("Enables NB-IOT"),
					},
					menu::Parameter::Named {
						parameter_name: "gnss",
						help: Some("Enables GNSS"),
					},
				],
			},
			command: "mode",
			help: Some("Get/set XSYSTEMMODE (NB-IOT, LTE-M, and/or GNSS)"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_at,
				parameters: &[menu::Parameter::Mandatory {
					parameter_name: "CMD",
					help: Some("AT Command to send"),
				}],
			},
			command: "at",
			help: Some("Send an AT command"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_naps,
				parameters: &[menu::Parameter::Mandatory {
					parameter_name: "NUM",
					help: Some("Number of naps to take"),
				}],
			},
			command: "naps",
			help: Some("Set how long we sleep for (in naps)"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_stat,
				parameters: &[],
			},
			command: "stat",
			help: Some("Show registration and general modem status"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_wakeup,
				parameters: &[],
			},
			command: "wakeup",
			help: Some("Wake up now."),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_post,
				parameters: &[],
			},
			command: "post",
			help: Some("Post to cloud"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_panic,
				parameters: &[],
			},
			command: "panic",
			help: Some("Deliberately crash"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_random,
				parameters: &[menu::Parameter::Mandatory {
					parameter_name: "BYTES",
					help: Some("Number of bytes to print"),
				}],
			},
			command: "random",
			help: Some("Get some random bytes"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_fix,
				parameters: &[],
			},
			command: "fix",
			help: Some("Get a GNSS fix"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_vbat,
				parameters: &[],
			},
			command: "vbat",
			help: Some("Get the battery voltage"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_go_at,
				parameters: &[],
			},
			command: "go_at",
			help: Some("Enter AT over UART mode"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_go_at_fun,
				parameters: &[],
			},
			command: "AT+CFUN?",
			help: Some("Enter AT mode if an AT command is entered..."),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_log,
				parameters: &[],
			},
			command: "log",
			help: Some("Dumps and clears the log"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_quiche,
				parameters: &[menu::Parameter::Mandatory {
					parameter_name: "MODE",
					help: Some("SERIAL for quiche over serial, RADIO for wireless quiche"),
				}],
			},
			command: "quiche",
			help: Some("Do an QUIC test with quiche"),
		},
		&menu::Item {
			item_type: menu::ItemType::Callback {
				function: command_date,
				parameters: &[menu::Parameter::Optional {
					parameter_name: "TIMESTAMP",
					help: Some("The date/time string in YYYY-MM-DDTHH:MM:SS format"),
				}],
			},
			command: "date",
			help: Some("Get/set the date/time"),
		},
	],
	entry: None,
	exit: None,
};

/// A UART we can access from anywhere (with run-time lock checking).
static GLOBAL_UART: spin::Mutex<Option<bsp::hal::uarte::Uarte<bsp::pac::UARTE0_NS>>> =
	spin::Mutex::new(None);

/// Our global `info!/debug!/trace!` etc logger. Prints to the UART console.
static LOGGER: SimpleLogger = SimpleLogger;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

/// Amount of time an LED is on for when everything is working. Keep short - uses power!
const PIP_TIME: u32 = bsp::hal::timer::Timer::<bsp::pac::TIMER0_NS>::TICKS_PER_SECOND / 50;

/// Amount of time an LED is on for when a connection has failed.
const ERR_TIME: u32 = bsp::hal::timer::Timer::<bsp::pac::TIMER0_NS>::TICKS_PER_SECOND * 2;

/// Amount of time we spend asleep in one go.
const NAP_TIME: u32 = bsp::hal::timer::Timer::<bsp::pac::TIMER0_NS>::TICKS_PER_SECOND * 1;

/// Number of naps while we wait for registration
const REGISTRATION_TIME_NAPS: u16 = 30;

/// Number of naps in a sleep
const SLEEP_TOTAL_NAPS: u32 = 300;

/// Number of times we try to read the GNSS socket. The system blocks for 1 second per read.
/// 300 should be enough for a cold start.
const GNSS_READ_TRIES: usize = 300;

/// Default value for AT%XSYSTEMMODE. Change this to pick NB-IOT mode instead of LTE-M mode.
const DEFAULT_XSYSTEMMODE: nrfxlib::modem::SystemMode = nrfxlib::modem::SystemMode::LteM;
// const DEFAULT_XSYSTEMMODE: nrfxlib::modem::SystemMode = nrfxlib::modem::SystemMode::NbIot;

/// The version, as reported by `git describe`.
static BUILD_GIT_VERSION: &'static str = env!("BUILD_GIT_VERSION");

/// The hash, as reported by `git rev-parse --short`.
static BUILD_GIT_HASH: &'static str = env!("BUILD_GIT_HASH");

/// ID used by quiche for HTTP requests
const HTTP_REQ_STREAM_ID: u64 = 4;

/// Maximum size for our quiche packets
const MAX_DATAGRAM_SIZE: usize = 1370;

// ==========================================================================
//
// Macros
//
// ==========================================================================

#[macro_export]
macro_rules! print {
	 ($($arg:tt)*) => {
		  {
				use core::fmt::Write as _;
				if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
					 let _err = write!(*uart, $($arg)*);
				}
		  }
	 };
}

#[macro_export]
macro_rules! println {
	 () => (print!("\n"));
	 ($($arg:tt)*) => {
		  {
				use core::fmt::Write as _;
				if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
					 let _err = writeln!(*uart, $($arg)*);
				}
		  }
	 };
}

// ==========================================================================
//
// Public Functions and Impls
//
// ==========================================================================

#[entry]
fn main() -> ! {
	log::set_logger(&LOGGER)
		.map(|()| log::set_max_level(log::LevelFilter::Debug))
		.expect("set_logger");

	// Initialize the allocator BEFORE you use it
	let start = rt::heap_start() as usize;
	let size = 32768; // in bytes
	unsafe { ALLOCATOR.init(start, size) }

	let mut board = bsp::Board::take().unwrap();

	board.NVIC.enable(bsp::pac::Interrupt::EGU1);
	board.NVIC.enable(bsp::pac::Interrupt::EGU2);
	// Enabled by nrfxlib::init();
	// board.NVIC.enable(bsp::pac::Interrupt::IPC);
	// Only use top three bits, so shift by up by 8 - 3 = 5 bits
	unsafe {
		board.NVIC.set_priority(bsp::pac::Interrupt::EGU2, 4 << 5);
		board.NVIC.set_priority(bsp::pac::Interrupt::EGU1, 4 << 5);
		board.NVIC.set_priority(bsp::pac::Interrupt::IPC, 0 << 5);
	}

	*GLOBAL_UART.lock() = Some(board.cdc_uart);

	info!("This is Rust on the nRF9160 LTE SiP");
	info!("Copyright (c) 42 Technology Ltd, 2019.");
	info!("Version: {}", BUILD_GIT_VERSION);
	info!("Git Hash: {}", BUILD_GIT_HASH);

	// Work around https://www.nordicsemi.com/DocLib/Content/Errata/nRF9160_EngA/latest/ERR/nRF9160/EngineeringA/latest/anomaly_160_17
	// *(volatile uint32_t *)0x40005C04 = 0x02ul;
	unsafe {
		core::ptr::write_volatile(0x4000_5C04 as *mut u32, 0x02);
	}

	// Enable RTC @ 32768 Hz and enable interrupt on overflow (every 512 seconds)
	let mut rtc = bsp::hal::Rtc::new(board.RTC1_NS);

	rtc.disable_event(bsp::hal::rtc::RtcInterrupt::Compare0);
	rtc.disable_event(bsp::hal::rtc::RtcInterrupt::Compare1);
	rtc.disable_event(bsp::hal::rtc::RtcInterrupt::Compare2);
	rtc.disable_event(bsp::hal::rtc::RtcInterrupt::Compare3);
	rtc.disable_event(bsp::hal::rtc::RtcInterrupt::Tick);
	rtc.disable_interrupt(bsp::hal::rtc::RtcInterrupt::Compare0, Some(&mut board.NVIC));
	rtc.disable_interrupt(bsp::hal::rtc::RtcInterrupt::Compare1, Some(&mut board.NVIC));
	rtc.disable_interrupt(bsp::hal::rtc::RtcInterrupt::Compare2, Some(&mut board.NVIC));
	rtc.disable_interrupt(bsp::hal::rtc::RtcInterrupt::Compare3, Some(&mut board.NVIC));
	rtc.disable_interrupt(bsp::hal::rtc::RtcInterrupt::Tick, Some(&mut board.NVIC));

	rtc.enable_event(bsp::hal::rtc::RtcInterrupt::Overflow);
	rtc.enable_interrupt(bsp::hal::rtc::RtcInterrupt::Overflow, Some(&mut board.NVIC));

	let rtc = rtc.enable_counter();

	// Start the Nordic library
	debug!("Calling nrfxlib::init()...");
	nrfxlib::init();

	// Make the RTC overflow quickly
	unsafe {
		let rtc_naughty = bsp::hal::pac::Peripherals::steal().RTC1_NS;
		rtc_naughty
			.tasks_trigovrflw
			.write(|w| w.tasks_trigovrflw().trigger());
	}

	// // Store our certificate
	// nrfxlib::tls::provision_certificates(SECURITY_TAG, Some(secrets::CA_CHAIN), None, None)
	//    .expect("Certificates stored");

	// Now initialise the GNSS RF switch (same on DK as Icarus)
	nrfxlib::modem::configure_gnss_on_pca10090ns().expect("Enable GNSS LNA");

	// System context that we can pass around
	let mut context = Context {
		delay_timer: bsp::hal::timer::Timer::new(board.TIMER0_NS),
		function_timer: bsp::hal::timer::Timer::new(board.TIMER1_NS),
		adc: bsp::hal::Saadc::new(board.SAADC_NS, bsp::hal::saadc::SaadcConfig::default()),
		vbat_pin: board.vbat,
		state: ProgramState::Console,
		leds: board.leds,
		naps_left: 0,
		index: 0,
		rtc: rtc,
		usual_total_naps: SLEEP_TOTAL_NAPS,
	};

	loop {
		match context.state {
			ProgramState::FindingFix => {
				context.state = finding_fix(&mut context);
			}
			ProgramState::UploadingPayload(ref payload) => {
				context.state = uploading_payload(payload.clone(), &mut context);
			}
			ProgramState::Console => {
				context = run_console(context);
			}
			ProgramState::Asleep => {
				context.state = asleep(&mut context);
			}
		}
	}
}

/// Real-time clock interrupt - fires whenever 32768 Hz, 24-bit counter overflows.
#[interrupt]
fn RTC1() {
	// Increment our overflow counter
	TIMER_OVERFLOWS.fetch_add(1, core::sync::atomic::Ordering::Release);
	// Clear interrupt by clearing event
	unsafe {
		let rtc = bsp::hal::pac::Peripherals::steal().RTC1_NS;
		rtc.events_ovrflw.write(|w| w.events_ovrflw().clear_bit());
	}
	// Ensure the main thread wakes up
	cortex_m::asm::sev();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn EGU1() {
	nrfxlib::application_irq_handler();
	cortex_m::asm::sev();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn EGU2() {
	nrfxlib::trace_irq_handler();
	cortex_m::asm::sev();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn IPC() {
	nrfxlib::ipc_irq_handler();
	cortex_m::asm::sev();
}

/// Debug function our C code can use to print messages to the UART.
#[no_mangle]
unsafe extern "C" fn rust_print(data: *const u8) {
	extern "C" {
		fn strlen(s: *const u8) -> isize;
	}
	let len = strlen(data);
	let slice = core::slice::from_raw_parts(data, len as usize);
	let string = core::str::from_utf8_unchecked(slice);
	print!("{}", &string);
}

#[no_mangle]
unsafe extern "C" fn abort() {
	panic!("Error!");
}

/// This is the mechanism by which newlib gets the current time. It is called
/// by `time`, and `_gettimeofday_r`.
///
/// We need to set up an RTC, and query the network for the current time. Time
/// is important for verifying certificate validity.
#[no_mangle]
unsafe extern "C" fn _gettimeofday(ptv: *mut Timeval, ptz: *mut Timezone) -> i32 {
	if !ptv.is_null() {
		let tv = Timeval {
			tv_sec: TIME_CONTEXT.get_posix_time(),
			tv_usec: 0,
		};
		ptv.write(tv);
	}
	if !ptz.is_null() {
		let tz = Timezone {
			tz_minuteswest: 0,
			tz_dsttime: 0,
		};
		ptz.write(tz);
	}
	0
}

#[no_mangle]
unsafe extern "C" fn _sbrk(increment: isize) -> usize {
	static mut MALLOC_BUFFER: [u8; 63 * 1024] = [0u8; 63 * 1024];
	static mut HEAP_END: usize = 0;
	if HEAP_END == 0 {
		HEAP_END = MALLOC_BUFFER.as_ptr() as *const u8 as usize;
	}
	debug!(
		"sbrk +{} = {}",
		increment,
		HEAP_END - (MALLOC_BUFFER.as_ptr() as *const u8 as usize)
	);
	let previous_heap_end = HEAP_END;
	HEAP_END = (previous_heap_end as isize + increment) as usize;
	if HEAP_END > ((MALLOC_BUFFER.as_ptr() as *const u8 as usize) + MALLOC_BUFFER.len()) {
		panic!("sbrk exhausted");
	}
	previous_heap_end
}

#[no_mangle]
unsafe extern "C" fn __aeabi_d2f(input: f64) -> f32 {
	input as f32
}

#[no_mangle]
unsafe extern "C" fn CRYPTO_sysrand(buf: *mut u8, len: usize) {
	debug!("Getting {} bytes of entropy", len);

	// Function pointer to the non-secure callable area in the bootloader
	let spm_request_random_number: extern "C" fn(
		output: *mut u8,
		len: usize,
		olen: *mut usize,
	) -> i32 = ::core::mem::transmute(0x00007fe1);

	// This API can only give blocks of 144 bytes
	const RANDOM_BLOCK_SIZE: usize = 144;

	static mut RANDOM_BLOCK: [u8; RANDOM_BLOCK_SIZE] = [0u8; RANDOM_BLOCK_SIZE];

	let buffer = core::slice::from_raw_parts_mut(buf, len);
	for chunk in buffer.chunks_mut(RANDOM_BLOCK.len()) {
		let mut amount_generated = 0;
		match spm_request_random_number(
			RANDOM_BLOCK.as_mut_ptr(),
			RANDOM_BLOCK.len(),
			&mut amount_generated,
		) {
			0 => {
				// OK
			}
			e => {
				panic!("spm_request_random_number({}) returned {}", len, e);
			}
		}
		if amount_generated != RANDOM_BLOCK.len() {
			panic!(
				"Didn't generate enough entropy: {} != {}",
				amount_generated,
				RANDOM_BLOCK.len()
			);
		}
		chunk.copy_from_slice(&RANDOM_BLOCK[0..chunk.len()]);
	}
}

/// Called when our code panics.
#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
	println!("{:?}", info);
	loop {
		cortex_m::asm::nop();
	}
}

// ==========================================================================
//
// Private Functions and Impls
//
// ==========================================================================

/// Stops the main loop from running.
fn command_stop(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	_context: &mut Context,
) {
	println!("Not implemented");
}

/// Starts the main loop running again.
fn command_start(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	_context: &mut Context,
) {
	println!("Not implemented");
}

/// The modem starts up in the powered-off state. This turns it on.
fn command_mode(
	_menu: &menu::Menu<Context>,
	item: &menu::Item<Context>,
	args: &[&str],
	_context: &mut Context,
) {
	let mut modes = (0, 0, 0);

	if let Some(_) = ::menu::argument_finder(item, args, "ltem").expect("Valid argument name") {
		println!("Enabling LTE-M.");
		modes.0 = 1;
	}
	if let Some(_) = ::menu::argument_finder(item, args, "nbiot").expect("Valid argument name") {
		println!("Enabling NB-IoT.");
		modes.1 = 1;
	}
	if let Some(_) = ::menu::argument_finder(item, args, "gnss").expect("Valid argument name") {
		println!("Enabling GNSS.");
		modes.2 = 1;
	}

	let system_mode = match modes {
		(1, 0, 1) => nrfxlib::modem::SystemMode::LteMAndGnss,
		(0, 1, 1) => nrfxlib::modem::SystemMode::NbIotAndGnss,
		(1, 0, 0) => nrfxlib::modem::SystemMode::LteM,
		(0, 0, 1) => nrfxlib::modem::SystemMode::GnssOnly,
		(0, 1, 0) => nrfxlib::modem::SystemMode::NbIot,
		(0, 0, 0) => {
			println!("Current mode: {:?}", nrfxlib::modem::get_system_mode());
			return;
		}
		(1, 1, _) => {
			println!("Can't enable both NB-IOT and LTE-M");
			return;
		}
		(_, _, _) => {
			println!("Invalid options selected.");
			return;
		}
	};

	if let Err(e) = nrfxlib::modem::set_system_mode(system_mode) {
		println!("Error setting system mode: {:?}", e);
	}
}

/// Set the new number of naps to take.
fn command_naps(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	args: &[&str],
	context: &mut Context,
) {
	if let Ok(n) = args[0].parse::<u32>() {
		println!("# naps set to {}", n);
		context.usual_total_naps = n;
	} else {
		println!("Didn't understand {:?}", args[0]);
	}
}

/// Set the new number of naps to take.
fn command_wakeup(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	context: &mut Context,
) {
	context.naps_left = 1;
	println!("Ok, will wake-up on Ctrl+Z");
}

/// Send an AT command to the modem and wait for a response.
fn command_at(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	args: &[&str],
	_context: &mut Context,
) {
	print_at_results(args[0]);
}

/// Show registration and general modem status.
fn command_stat(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	_context: &mut Context,
) {
	for cmd in &[
		"AT+CFUN?",
		"AT+CEREG?",
		"AT%XSNRSQ?",
		"AT+CESQ",
		"AT%XTEMP?",
		"AT+CGCONTRDP=0",
		"AT+CCLK?",
		"AT%XMONITOR",
		"AT+CGDCONT?",
		"AT+CGPADDR?",
		"AT%XCONNSTAT?",
	] {
		print_at_results(cmd);
	}
}

/// Push some data into AWS
fn command_post(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	context: &mut Context,
) {
	// Start a timer
	context.function_timer.start(0xFFFF_FFFFu32);
	// Run the function
	let result = do_post(
		context,
		Payload {
			fix: None,
			temp: 25,
			vbat: 4.0,
			signal_status: "Test status".into(),
		},
	);
	context.index = context.index.wrapping_add(1);
	// Print the result
	let now = context.function_timer.read();
	println!(
		"Got {:?} after {} seconds",
		result,
		(now as f32) / 1_000_000.0
	);
}

/// Deliberately crash
fn command_panic(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	_context: &mut Context,
) {
	panic!("command_panic was called!")
}

/// Deliberately crash
fn command_random(
	_menu: &menu::Menu<Context>,
	item: &menu::Item<Context>,
	args: &[&str],
	_context: &mut Context,
) {
	let mut buffer = [0; 512];
	if let Some(s) = ::menu::argument_finder(item, args, "BYTES").expect("Valid argument name") {
		if let Ok(mut n) = s.parse::<usize>() {
			while n > 0 {
				let this_time = n.min(buffer.len());
				println!("Asking for {} bytes", this_time);
				unsafe {
					CRYPTO_sysrand(buffer.as_mut_ptr(), this_time);
				};
				println!("{:x?}", &buffer[0..this_time]);
				n -= this_time;
			}
		} else {
			println!("Bad argument: {:?}", s);
		}
	}
}

/// Get the battery voltage
fn command_vbat(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	context: &mut Context,
) {
	let voltage = get_battery_voltage(context);
	println!("Battery voltage: {}", voltage);
}

/// Get a GNSS fix
fn command_fix(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	context: &mut Context,
) {
	let p = get_position(context);
	println!("Position: {:?}", p);
}

/// Print the system log
fn command_log(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	_context: &mut Context,
) {
	let log = unsafe { &mut SYSTEM_LOG };
	while let Some(n) = log.dequeue() {
		println!("{:?}", n);
	}
}

/// Enter AT over UART mode
fn command_go_at(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	context: &mut Context,
) {
	const UART_READ_SIZE: usize = 64;
	const AT_READ_SIZE: usize = 64;
	let mut f = || -> Result<(), Error> {
		let mut at_socket = nrfxlib::at::AtSocket::new()?;
		let mut input_buffer: heapless::Vec<u8, heapless::consts::U256> = heapless::Vec::new();
		loop {
			let mut temp_buf = [0u8; UART_READ_SIZE];
			// Read from console UART
			let res = if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
				Some(uart.read_timeout(&mut temp_buf, &mut context.delay_timer, 100))
			} else {
				None
			};
			match res {
				Some(Err(bsp::hal::uarte::Error::Timeout(n))) => {
					// Process partial buffer...
					if load_chars(&mut at_socket, &mut input_buffer, &temp_buf[0..n])? {
						break;
					}
				}
				Some(Err(_)) => {
					return Err(Error::Read);
				}
				Some(Ok(_)) => {
					// Process full buffer...
					if load_chars(&mut at_socket, &mut input_buffer, &temp_buf)? {
						break;
					}
				}
				None => {
					println!("Failed to grab UART lock!");
				}
			}
			// Poll the AT socket for 100 ms

			let mut socket_list = [nrfxlib::PollEntry::new(
				&mut at_socket,
				nrfxlib::PollFlags::Read,
			)];
			let result = nrfxlib::poll(&mut socket_list[..], 100)?;
			match result {
				0 => {
					// No data
				}
				_ => {
					if socket_list[0].result().is_readable() {
						// Data available. Now read the AT socket to get the data.
						let mut buffer = [0u8; AT_READ_SIZE];
						match at_socket.recv(&mut buffer)? {
							Some(n) => {
								if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
									// Subtract 1 to avoid printing final NUL byte
									uart.write(&buffer[0..n - 1]).map_err(|_| Error::Write)?;
								}
							}
							None => {
								// Do nothing
							}
						}
					}
				}
			}
		}
		Ok(())
	};
	println!("OK\r\n");
	if let Err(e) = f() {
		println!("Error: {:?}", e);
	}
}

/// Handle the first AT command the link monitor sends, then enter AT mode.
fn command_go_at_fun(
	menu: &menu::Menu<Context>,
	item: &menu::Item<Context>,
	args: &[&str],
	context: &mut Context,
) {
	match nrfxlib::at::send_at_command("AT+CFUN?", |s| {
		println!("{}", s);
	}) {
		Ok(_) => {
			// Jump to the normal AT handler (which prints OK when it starts)
			command_go_at(menu, item, args, context);
		}
		Err(_) => {
			// Quit with an error.
			println!("ERROR");
		}
	}
}

fn dehex(byte: u8) -> u8 {
	match byte {
		b'0'..=b'9' => byte - b'0',
		b'a'..=b'f' => byte + 10 - b'a',
		b'A'..=b'F' => byte + 10 - b'F',
		_ => {
			panic!("Bad hex {}", byte);
		}
	}
}

fn send_packet(packet: &[u8]) {
	if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
		write!(uart, "TX{:04x}", packet.len()).unwrap();
		for b in packet.iter() {
			write!(uart, "{:02x}", b).unwrap();
		}
		write!(uart, "\r\n").unwrap();
	} else {
		panic!("No UART lock");
	}
}

fn poll_packet() {
	println!("?");
}

fn read_timeout<I>(
	buffer: &mut [u8],
	timer: &mut bsp::hal::timer::Timer<I>,
	timeout_cycles: u32,
) -> Result<(), ()>
where
	I: bsp::hal::timer::Instance,
{
	if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
		uart.read_timeout(buffer, timer, timeout_cycles)
			.map_err(|_e| ())
	} else {
		Err(())
	}
}

fn read_packet_timeout(
	buffer: &mut [u8],
	_timeout_ms: u32,
	context: &mut Context,
) -> Option<usize> {
	let mut uart_buf = [0u8; 4];
	if read_timeout(&mut uart_buf, &mut context.delay_timer, 64_000_000).is_ok() {
		// Got four bytes of length
		let l1 = dehex(uart_buf[0]) as usize;
		let l2 = dehex(uart_buf[1]) as usize;
		let l3 = dehex(uart_buf[2]) as usize;
		let l4 = dehex(uart_buf[3]) as usize;
		let length: usize = (l1 << 12) | (l2 << 8) | (l3 << 4) | l4;
		debug!("Reading {} bytes", length);
		for i in 0..length {
			if read_timeout(&mut uart_buf[0..=1], &mut context.delay_timer, 64_000_000).is_ok() {
				let b1 = dehex(uart_buf[0]);
				let b2 = dehex(uart_buf[1]);
				let data = (b1 << 4) | b2;
				buffer[i] = data;
			} else {
				panic!("Read timeout!");
			}
		}
		Some(length)
	} else {
		None
	}
}

/// Send a QUIC request using quiche
fn command_quiche(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	_args: &[&str],
	context: &mut Context,
) {
	let mut f = || -> Result<(), Error> {
		let mut req_sent = false;
		let server_name = secrets::ENDPOINT_ADDRESS;
		let mut config = quiche::Config::new(quiche::PROTOCOL_VERSION)?;
		config.verify_peer(false);
		config
			.set_application_protos(b"\x05hq-23\x08http/0.9")
			.unwrap();
		config.set_idle_timeout(5000);
		config.set_max_packet_size(MAX_DATAGRAM_SIZE as u64);
		config.set_initial_max_data(10000000);
		config.set_initial_max_stream_data_bidi_local(1000000);
		config.set_initial_max_stream_data_bidi_remote(1000000);
		config.set_initial_max_streams_bidi(2);
		config.set_initial_max_streams_uni(2);
		config.set_disable_active_migration(true);

		let mut scid = [0u8; quiche::MAX_CONN_ID_LEN];
		unsafe {
			CRYPTO_sysrand(scid.as_mut_ptr(), scid.len());
		}

		println!(
			"Opening QUIC connection to {:?} with SCID {:x?}",
			server_name, &scid
		);
		let mut conn = quiche::connect(Some(server_name), &scid, &mut config)?;
		let mut buf = vec![0; MAX_DATAGRAM_SIZE];
		let arg_string = menu::argument_finder(_item, _args, "MODE").expect("Valid argument name");
		let mode = match arg_string {
			Some("SERIAL") => QuicheMode::SERIAL,
			Some("RADIO") => QuicheMode::RADIO,
			_ => QuicheMode::INVALID_MODE,
		};

		let write = conn.send(&mut buf)?;
		match mode {
			QuicheMode::INVALID_MODE => {
				return Err(Error::BadParameters);
			}
			QuicheMode::SERIAL => {
				debug!("Opening socket...");
				debug!(">> socket.write({})", write);
				send_packet(&buf[..write]);
				let mut timeout_count = 0;
				while timeout_count < 10 {
					let timeout = conn.timeout();
					let timeout_ms = match timeout {
						Some(n) => n.as_millis() as u32,
						None => 65535,
					};
					if timeout_ms > 65535 {
						panic!("Timeout too large!");
					} else {
						debug!("Polling for {} ms", timeout_ms);
					}
					// let mut poll_list = [nrfxlib::PollEntry::new(

					poll_packet();

					// Read incoming UDP packets from the socket and feed them to quiche,
					// until there are no more packets to read.
					'readSerial: loop {
						let len = read_packet_timeout(&mut buf, timeout_ms, context).unwrap_or(0);

						// If the event loop reported no events, it means that the timeout
						// has expired, so handle it without attempting to read packets. We
						// will then proceed with the send loop.
						if len == 0 {
							// No character received
							warn!("timed out");
							timeout_count += 1;
							conn.on_timeout();
							break 'readSerial;
						} else {
							timeout_count = 0;
						}

						debug!("<< got {} bytes from network: {:x?}", len, &buf[..len]);

						// Process potentially coalesced packets.
						match conn.recv(&mut buf[..len]) {
							Ok(v) => {
								debug!("<< quiche ate {} bytes", v);
								v
							}

							Err(quiche::Error::Done) => {
								debug!("<< done reading");
								break;
							}

							Err(e) => {
								error!("<< recv failed: {:?}", e);
								break 'readSerial;
							}
						};

						break 'readSerial;
					}

					if conn.is_closed() {
						info!("-- connection closed, {:?}", conn.stats());
						break;
					}

					// Send an HTTP request as soon as the connection is established.
					if conn.is_established() && !req_sent {
						info!(">> sending HTTP request");
						let req = "GET /index.html\r\n";
						conn.stream_send(HTTP_REQ_STREAM_ID, req.as_bytes(), true)
							.unwrap();

						req_sent = true;
					}

					// Process all readable streams.
					for s in conn.readable() {
						debug!("<< reading plaintext from quiche on {:?}", s);
						while let Ok((read, fin)) = conn.stream_recv(s, &mut buf) {
							let stream_buf = &buf[..read];

							debug!(
								"<< stream {} has {} bytes (fin? {})",
								s,
								stream_buf.len(),
								fin
							);

							print!("{}", unsafe { core::str::from_utf8_unchecked(&stream_buf) });

							// The server reported that it has no more data to send, which means
							// we got the full response. Close the connection.
							if s == HTTP_REQ_STREAM_ID && fin {
								info!("<< response received, closing...");
								conn.close(true, 0x00, b"kthxbye").unwrap();
								return Ok(());
							}
						}
					}

					// Generate outgoing QUIC packets and send them on the UDP socket, until
					// quiche reports that there are no more packets to be sent.
					loop {
						let chunk = match conn.send(&mut buf) {
							Ok(v) => {
								debug!(">> read {} bytes ciphertext from quiche", v);
								v
							}

							Err(quiche::Error::Done) => {
								debug!(">> done writing");
								break;
							}

							Err(e) => {
								error!(">> send failed: {:?}", e);
								conn.close(false, 0x1, b"fail").ok();
								break;
							}
						};

						debug!(">> socket.write({})", chunk);
						send_packet(&buf[..chunk]);
					}

					if conn.is_closed() {
						info!("-- connection closed, {:?}", conn.stats());
						break;
					}
				}

				Err(Error::Timeout)
			}
			QuicheMode::RADIO => {
				debug!("Opening socket...");
				let mut socket = nrfxlib::udp::UdpSocket::new()?;
				debug!(">> socket.write({})", write);
				socket.connect(server_name, secrets::ENDPOINT_PORT)?;
				socket.write(&buf[..write])?;

				let mut timeout_count = 0;
				while timeout_count < 10 {
					let timeout = conn.timeout();
					let timeout_ms = match timeout {
						Some(n) => n.as_millis() as u32,
						None => 65535,
					};
					if timeout_ms > 65535 {
						panic!("Timeout too large!");
					} else {
						debug!("Polling for {} ms", timeout_ms);
					}
					let mut poll_list = [nrfxlib::PollEntry::new(
						&mut socket,
						nrfxlib::PollFlags::Read,
					)];
					let events = nrfxlib::poll(&mut poll_list, timeout_ms as u16)?;

					// Read incoming UDP packets from the socket and feed them to quiche,
					// until there are no more packets to read.
					'readRadio: loop {
						// If the event loop reported no events, it means that the timeout
						// has expired, so handle it without attempting to read packets. We
						// will then proceed with the send loop.
						if events == 0 {
							// No character received
							warn!("timed out");
							timeout_count += 1;
							conn.on_timeout();
							break 'readRadio;
						} else {
							timeout_count = 0;
						}

						let len = match socket.recv(&mut buf) {
							// Read hex bytes from serial port into buf.
							Ok(None) => {
								// There are no more UDP packets to read, so end the read
								// loop.
								break 'readRadio;
							}
							Ok(Some(v)) => v,
							Err(e) => {
								panic!("recv() failed: {:?}", e);
							}
						};

						debug!("<< got {} bytes from network: {:x?}", len, &buf[..len]);

						// Process potentially coalesced packets.
						match conn.recv(&mut buf[..len]) {
							Ok(v) => {
								debug!("<< quiche ate {} bytes", v);
								v
							}

							Err(quiche::Error::Done) => {
								debug!("<< done reading");
								break;
							}

							Err(e) => {
								error!("<< recv failed: {:?}", e);
								break 'readRadio;
							}
						};

						break 'readRadio;
					}

					if conn.is_closed() {
						info!("-- connection closed, {:?}", conn.stats());
						break;
					}

					// Send an HTTP request as soon as the connection is established.
					if conn.is_established() && !req_sent {
						info!(">> sending HTTP request");
						let req = "GET /index.html\r\n";
						conn.stream_send(HTTP_REQ_STREAM_ID, req.as_bytes(), true)
							.unwrap();

						req_sent = true;
					}

					// Process all readable streams.
					for s in conn.readable() {
						debug!("<< reading plaintext from quiche on {:?}", s);
						while let Ok((read, fin)) = conn.stream_recv(s, &mut buf) {
							let stream_buf = &buf[..read];

							debug!(
								"<< stream {} has {} bytes (fin? {})",
								s,
								stream_buf.len(),
								fin
							);

							print!("{}", unsafe { core::str::from_utf8_unchecked(&stream_buf) });

							// The server reported that it has no more data to send, which means
							// we got the full response. Close the connection.
							if s == HTTP_REQ_STREAM_ID && fin {
								info!("<< response received, closing...");
								conn.close(true, 0x00, b"kthxbye").unwrap();
								return Ok(());
							}
						}
					}

					// Generate outgoing QUIC packets and send them on the UDP socket, until
					// quiche reports that there are no more packets to be sent.
					loop {
						let chunk = match conn.send(&mut buf) {
							Ok(v) => {
								debug!(">> read {} bytes ciphertext from quiche", v);
								v
							}

							Err(quiche::Error::Done) => {
								debug!(">> done writing");
								break;
							}

							Err(e) => {
								error!(">> send failed: {:?}", e);
								conn.close(false, 0x1, b"fail").ok();
								break;
							}
						};

						debug!(">> socket.write({})", chunk);
						socket.write(&buf[..chunk])?;
					}

					if conn.is_closed() {
						info!("-- connection closed, {:?}", conn.stats());
						break;
					}
				}

				Err(Error::Timeout)
			}
		}
	};
	println!("Starting test...");
	let result = f();
	println!("Test gave: {:?}", result);
}

/// Set the current date/time
/// Get/set the date
fn command_date<'a>(
	_menu: &menu::Menu<Context>,
	_item: &menu::Item<Context>,
	args: &[&str],
	_context: &mut Context,
) {
	if args.len() > 0 {
		let f = || -> Result<(), (&'static str, core::num::ParseIntError)> {
			// Set new date
			let mut iter = args[0].split(|c| " -T:/".contains(c));
			let timestamp = Timestamp {
				year_from_1970: (iter
					.next()
					.unwrap_or("1970")
					.parse::<u32>()
					.map_err(|e| ("Bad year", e))?
					- 1970) as u8,
				month: iter
					.next()
					.unwrap_or("1")
					.parse::<u8>()
					.map_err(|e| ("Bad month", e))?,
				days: iter
					.next()
					.unwrap_or("1")
					.parse::<u8>()
					.map_err(|e| ("Bad days", e))?,
				hours: iter
					.next()
					.unwrap_or("0")
					.parse::<u8>()
					.map_err(|e| ("Bad hours", e))?,
				minutes: iter
					.next()
					.unwrap_or("0")
					.parse::<u8>()
					.map_err(|e| ("Bad minutes", e))?,
				seconds: iter
					.next()
					.unwrap_or("0")
					.parse::<u8>()
					.map_err(|e| ("Bad seconds", e))?,
			};
			crate::TIME_CONTEXT.set_timestamp(timestamp);
			Ok(())
		};
		println!("Setting the time - {:?}", f());
	}
	println!("Date: {}", crate::TIME_CONTEXT.get_timestamp());
}

/// Returns `true` if the user has pressed the `Esc` key on the console. We
/// can then abort whatever long-running operation we are performing.
fn check_for_escape(context: &mut Context) -> bool {
	let maybe_c = if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
		let mut uart_rx_buf = [0u8; 1];
		if uart
			.read_timeout(&mut uart_rx_buf, &mut context.delay_timer, 1000)
			.is_ok()
		{
			Some(uart_rx_buf[0])
		} else {
			None
		}
	} else {
		None
	};
	if let Some(c) = maybe_c {
		debug!("Got 0x{:2x}", c);
	}

	// Is escape pressed?
	maybe_c == Some(0x1B)
}

/// Handles the `Finding Fix` state. In this state we are collecting the data
/// we need for our payload, including the GPS fix.
fn finding_fix(context: &mut Context) -> ProgramState {
	debug!("Turing modem on for GPS...");
	context.leds.blue.enable();
	nrfxlib::modem::off().expect("Turn modem off");
	nrfxlib::modem::set_system_mode(nrfxlib::modem::SystemMode::GnssOnly)
		.expect("Set to GNSS mode");
	nrfxlib::modem::on().expect("Turn modem on");
	info!("Modem in GPS mode");

	debug!("Finding fix...");
	let fix = get_position(context);
	info!("Found fix {:?}", fix);

	debug!("Getting temperature..");
	let mut temp = 0;
	nrfxlib::at::send_at_command("AT%XTEMP?", |s| {
		temp = (&s[8..]).parse::<i32>().unwrap_or(-127);
	})
	.expect("Call XTEMP?");
	info!("Got temperature {}", temp);

	log_event(LogEvent::Temp(temp), context);

	debug!("Getting voltage...");
	let vbat = get_battery_voltage(context);
	info!("Got voltage...{}V", vbat);

	log_event(LogEvent::Vbat(vbat), context);

	debug!("Turing modem off...");
	nrfxlib::modem::off().expect("Turn modem off");
	context.leds.blue.disable();
	info!("Modem turned off");

	// Now upload the fix
	ProgramState::UploadingPayload(Payload {
		fix,
		temp,
		vbat,
		signal_status: heapless::String::new(),
	})
}

/// Handles the `Uploading Payload` state. Turns the modem on and uploads the
/// payload to the cloud.
fn uploading_payload(mut payload: Payload, context: &mut Context) -> ProgramState {
	debug!("Turing modem on for {:?}...", DEFAULT_XSYSTEMMODE);
	context.leds.green.enable();
	nrfxlib::modem::off().expect("Turn modem off");
	nrfxlib::modem::set_system_mode(DEFAULT_XSYSTEMMODE).expect("Set to LTE mode");
	info!("Modem in {:?} mode", DEFAULT_XSYSTEMMODE);
	nrfxlib::modem::on().expect("Turn modem on");

	// Wait for network
	let registration = register_on_network(context, &mut payload.signal_status);
	log_event(
		LogEvent::Registration {
			result: registration.clone(),
			signal_status: payload.signal_status.clone(),
		},
		context,
	);
	match registration {
		Ok(true) => {
			info!("Registered on network.");
			// Registered
			match do_post(context, payload) {
				Ok(_) => {
					info!("Uploaded OK!");
				}
				Err(e) => {
					error!("Failed to upload: {:?}", e);
					context.leds.red.enable();
					context.delay_timer.delay(ERR_TIME);
					context.leds.red.disable();
				}
			}
		}
		Ok(false) => {
			// Not registered
			warn!("Failed to register on network.");
		}
		Err(e) => error!("Failed to register: {:?}", e),
	}

	context.index = context.index.wrapping_add(1);

	debug!("Turing modem off...");
	nrfxlib::modem::off().expect("Turn modem off");
	context.leds.green.disable();
	info!("Modem turned off");

	context.naps_left = context.usual_total_naps;
	ProgramState::Asleep
}

/// Waits until the modem has registered on the LTE network and obtained an IP address.
fn register_on_network(
	context: &mut Context,
	signal_status: &mut heapless::String<heapless::consts::U128>,
) -> Result<bool, Error> {
	info!("Waiting {} naps for IP address...", REGISTRATION_TIME_NAPS);
	let mut is_registered = false;
	for _ in 0..REGISTRATION_TIME_NAPS {
		let mut have_ip = false;
		let mut have_network = false;
		if check_for_escape(context) {
			debug!("Breaking out...");
			break;
		}
		nrfxlib::at::send_at_command("AT+CGDCONT?", |ind| {
			// Any response means we have an IP address from the APN
			if ind.starts_with("+CGDCONT:") {
				let data = ind.trim_start_matches("+CGDCONT:").trim();
				let mut items_iter = data.split(',').map(|s| s.trim_matches('"'));
				let _cid = items_iter.next();
				let pdp_type = items_iter.next();
				let _apn = items_iter.next();
				let pdp_addr = items_iter.next();
				match (pdp_type, pdp_addr) {
					// No IP address
					(_, Some("")) => {}
					// Non-empty IP address, and valid PDP type
					(Some("IP"), Some(_)) | (Some("IPV4V6"), Some(_)) | (Some("IPV6"), Some(_)) => {
						have_ip = true;
					}
					// Anything else
					(_, _) => {}
				}
			}
			have_ip = true;
		})?;
		nrfxlib::at::send_at_command("AT+CEREG?", |ind| {
			if ind.starts_with("+CEREG") {
				let data = ind.trim_start_matches("+CEREG:").trim();
				let mut items_iter = data.split(',');
				let _ind_status = items_iter.next();
				let registration_status = items_iter.next();
				debug!(
					"Registration status: ({:?}) {:?}",
					data, registration_status
				);
				match registration_status {
					Some("1") | Some("5") => have_network = true,
					_ => {}
				}
			}
		})?;
		if have_ip && have_network {
			info!("Registered OK.");
			is_registered = true;
			break;
		} else {
			context.delay_timer.delay(NAP_TIME);
		}
	}
	// AT%XMONITOR returns:
	// <reg_status>,[<full_name>,<short_name>,<plmn>,<tac>,<AcT>,<band>,
	// <cell_id>,<phys_cell_id>,<EARFCN>,<rsrp>,<snr>,
	// <NW-provided_eDRX_value>,<Active-Time>,<Periodic-TAU>]
	let _ = nrfxlib::at::send_at_command("AT%XMONITOR", |i| {
		let data = i.trim_start_matches("%XMONITOR:");
		let _ = signal_status.push_str(data.trim());
	});
	return Ok(is_registered);
}

/// Handles the interactive console. It's disabled most of the time to avoid
/// wasting power polling the UART, but once we're in this function we stay
/// here until the user presses Control + Z.
fn run_console(mut context: Context) -> Context {
	let mut buffer = [0u8; 80];
	let mut runner = menu::Runner::new(&ROOT_MENU, &mut buffer, context);
	loop {
		let maybe_c = if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
			let mut uart_rx_buf = [0u8; 1];
			if uart
				.read_timeout(&mut uart_rx_buf, &mut runner.context.delay_timer, 1_000_000)
				.is_ok()
			{
				Some(uart_rx_buf[0])
			} else {
				None
			}
		} else {
			None
		};
		// If we did, give it to the menu (without holding the UART lock)
		if let Some(c) = maybe_c {
			// Ctrl+Z to exit
			if c == 0x1A {
				break;
			} else if c == 0x19 {
				println!(
					"Time: {} {:?}",
					get_time_ticks(),
					TIME_CONTEXT.get_timestamp()
				);
			} else {
				runner.input_byte(c);
			}
		}
	}
	// Go back to sleep for however many naps we have left
	context = runner.context;
	context.state = ProgramState::Asleep;
	context
}

/// Handles the 'Asleep' state. Checks for an `Esc`, has a nap and then pips
/// the LED.
fn asleep(context: &mut Context) -> ProgramState {
	// Has escape been pressed?
	if check_for_escape(context) {
		return ProgramState::Console;
	}
	info!("Sleeping...");
	context.delay_timer.delay(NAP_TIME);
	context.leds.green.enable();
	context.delay_timer.delay(PIP_TIME);
	context.leds.green.disable();
	if context.naps_left == 0 {
		// Time to wake up
		ProgramState::FindingFix
	} else {
		context.naps_left -= 1;
		// Go around again
		ProgramState::Asleep
	}
}

/// Send an AT command and print any results to the screen
fn print_at_results(cmd: &str) {
	if let Err(e) = nrfxlib::at::send_at_command(cmd, |s| {
		println!("{}", s);
	}) {
		println!("Err running {:?}: {:?}", cmd, e);
	}
}

/// Write an event to the system log
fn log_event(event: LogEvent, context: &mut Context) {
	let log = unsafe { &mut SYSTEM_LOG };
	if log.len() == log.capacity() {
		let old_record = log.dequeue();
		warn!("Queue overflow - dropping {:?}", old_record);
	}
	let record = LogRecord {
		index: context.index,
		event,
	};
	info!("Logging {:?}", record);
	let _ = log.enqueue(record);
}

/// Push the payload up to the cloud
fn do_post(context: &mut Context, payload: Payload) -> Result<(), Error> {
	let mut json: heapless::String<heapless::consts::U256> = heapless::String::new();
	if let Some(position) = payload.fix {
		write!(
			json,
			r#"{{"values":{{"temperature":{},"index":{},"battery_voltage":{},"signal_status":{:?}}},"location":{{"latitude":{},"longitude":{}}}}}"#,
			payload.temp, context.index, payload.vbat, payload.signal_status, position.latitude, position.longitude
		)
		.map_err(|_e| Error::Write)?;
	} else {
		write!(
			json,
			r#"{{"values":{{"temperature":{},"index":{},"battery_voltage":{},"signal_status":{:?}}}}}"#,
			payload.temp, context.index, payload.vbat, payload.signal_status
		)
		.map_err(|_e| Error::Write)?;
	}
	println!("Constructed payload {}", json);

	log_event(LogEvent::Uploaded, context);
	Ok(())
}

/// Transfer chars to the line buffer, then push to the socket when a full
/// line is received.
fn load_chars(
	socket: &mut nrfxlib::at::AtSocket,
	buffer: &mut heapless::Vec<u8, heapless::consts::U256>,
	new_chars: &[u8],
) -> Result<bool, Error> {
	for &ch in new_chars.iter() {
		// Send character to modem, unless it's Ctrl+C (0x03), in which
		// case exit.
		// We assume the input is ASCII, but all the AT commands are.
		print!("{}", ch as char);
		if ch == 0x03 {
			return Ok(true);
		} else if ch == b'\n' || ch == b'\r' {
			println!();
			buffer.extend(b"\r\n");
			socket.write(&buffer)?;
			buffer.clear();
		} else {
			let _ = buffer.push(ch);
		}
	}
	Ok(false)
}

/// Get the current Li-Po battery voltage
fn get_battery_voltage(context: &mut Context) -> f32 {
	// Default ADC reference is VDD/4 with a Gain of x4.
	const VDD_IO: f32 = 3.3;
	// The default ADC resolution is 14 bits
	const ADC_MAX: f32 = 16384.0;
	// Scale according to the 4.7M and 10M resistors in the resistive divider
	const R1: f32 = 4.7;
	const R2: f32 = 10.0;
	let input_reading = context.adc.read(&mut context.vbat_pin).unwrap_or(0);
	let input_voltage = (VDD_IO * (input_reading as f32)) / ADC_MAX;
	let battery_voltage = (input_voltage * (R1 + R2)) / R2;
	battery_voltage
}

/// Enables the GNSS subsystem and tries to get a fix.
///
/// Because of poor PSM support with the networks, we fully disable LTE-M
/// while we get the fix. We leave the modem powered off when we are finished.
///
/// Each fix read blocks for up to 1 second, and we perform up to
/// GNSS_READ_TRIES times. We return the first fix we get, or None if we don't
/// get one.
fn get_position(context: &mut Context) -> Option<Position> {
	let mut max_svs = 0;
	let mut f = |context: &mut Context| -> Result<Option<Position>, Error> {
		debug!("Opening socket...");
		let gnss = nrfxlib::gnss::GnssSocket::new()?;
		let agps_delete_mask = nrfxlib::gnss::DeleteMask::new();
		debug!("Starting GNSS with {:?}", agps_delete_mask);
		gnss.start(agps_delete_mask)?;
		for i in 1..=GNSS_READ_TRIES {
			debug!("Checking for GNSS fix ({}/{})...", i, GNSS_READ_TRIES);
			if check_for_escape(context) {
				debug!("Breaking out...");
				break;
			}
			match gnss.get_fix_blocking()? {
				None => {
					debug!("Trying again...");
				}
				Some(fix) => {
					match fix {
						nrfxlib::gnss::GnssData::Position(p) => {
							let num_svs = p.sv.iter().filter(|obj| obj.sv > 0).count();
							max_svs = max_svs.max(num_svs);
							if fix.is_valid() {
								info!("Got fix {:?}", fix);
								// Convert to our format and set our RTC
								crate::TIME_CONTEXT.set_timestamp(p.datetime.into());
								// Return the lat/long
								return Ok(Some(Position {
									latitude: p.latitude,
									longitude: p.longitude,
								}));
							} else {
								debug!("No fix, but found {} SVs", num_svs);
							}
						}
						_ => {
							// Ignore any NMEA strings
							debug!("Ignoring {:?}", fix);
						}
					}
				}
			}
		}
		Ok(None)
	};

	context.function_timer.start(0xFFFF_FFFFu32);
	let result = f(context);
	let now = context.function_timer.read();
	// Timer runs at 1 MHz, so 1_000_000 ticks per second.
	let elapsed_time_100ms = (now as f32) / (1_000_000.0 / 10.0);

	info!("Elapsed raw: {:?}", now);
	info!("Elapsed ticks: {:?}", elapsed_time_100ms);

	log_event(
		LogEvent::GpsFix {
			time: elapsed_time_100ms as u16,
			max_svs: max_svs as u8,
			result: result.clone(),
		},
		context,
	);

	match result {
		Err(e) => {
			error!("Error: {:?}", e);
			None
		}
		Ok(n) => n,
	}
}

impl From<nrfxlib::Error> for Error {
	fn from(err: nrfxlib::Error) -> Error {
		Error::Library(err)
	}
}

impl From<quiche::Error> for Error {
	fn from(err: quiche::Error) -> Error {
		Error::Quiche(err)
	}
}

impl core::fmt::Write for Context {
	fn write_str(&mut self, message: &str) -> core::fmt::Result {
		if let Some(ref mut uart) = *crate::GLOBAL_UART.lock() {
			write!(uart, "{}", message)?;
		}
		Ok(())
	}
}

impl log::Log for SimpleLogger {
	fn enabled(&self, metadata: &Metadata) -> bool {
		metadata.level() <= Level::Debug
	}

	fn log(&self, record: &Record) {
		if self.enabled(record.metadata()) {
			println!("{} - {}", record.level(), record.args());
		}
	}

	fn flush(&self) {}
}

#[alloc_error_handler]
fn alloc_panic(layout: core::alloc::Layout) -> ! {
	panic!("Bad alloc: {:?}", layout);
}

impl DayOfWeek {
	/// Returns the UK English word for the day of the week
	pub fn day_str(&self) -> &'static str {
		match self {
			DayOfWeek::Monday => "Monday",
			DayOfWeek::Tuesday => "Tuesday",
			DayOfWeek::Wednesday => "Wednesday",
			DayOfWeek::Thursday => "Thursday",
			DayOfWeek::Friday => "Friday",
			DayOfWeek::Saturday => "Saturday",
			DayOfWeek::Sunday => "Sunday",
		}
	}
}

impl Timestamp {
	/// Returns the day of the week for the given timestamp.
	pub fn day_of_week(&self) -> DayOfWeek {
		let zellers_month = ((i32::from(self.month) + 9) % 12) + 1;
		let k = i32::from(self.days);
		let year = i32::from(self.year_from_1970) + if zellers_month >= 11 { 1969 } else { 1970 };
		let d = year % 100;
		let c = year / 100;
		let f = k + (((13 * zellers_month) - 1) / 5) + d + (d / 4) + (c / 4) - (2 * c);
		let day_of_week = f % 7;
		match day_of_week {
			0 => DayOfWeek::Sunday,
			1 => DayOfWeek::Monday,
			2 => DayOfWeek::Tuesday,
			3 => DayOfWeek::Wednesday,
			4 => DayOfWeek::Thursday,
			5 => DayOfWeek::Friday,
			_ => DayOfWeek::Saturday,
		}
	}

	/// Move this timestamp forward by a number of days and seconds.
	pub fn increment(&mut self, days: u32, seconds: u32) {
		let new_seconds = seconds + u32::from(self.seconds);
		self.seconds = (new_seconds % 60) as u8;
		let new_minutes = (new_seconds / 60) + u32::from(self.minutes);
		self.minutes = (new_minutes % 60) as u8;
		let new_hours = (new_minutes / 60) + u32::from(self.hours);
		self.hours = (new_hours % 24) as u8;
		let mut new_days = (new_hours / 24) + u32::from(self.days) + days;
		while new_days > u32::from(self.days_in_month()) {
			new_days -= u32::from(self.days_in_month());
			self.month += 1;
			if self.month > 12 {
				self.month = 1;
				self.year_from_1970 += 1;
			}
		}
		self.days = new_days as u8;
	}

	/// Returns true if this is a leap year, false otherwise.
	pub fn is_leap_year(&self) -> bool {
		let year = u32::from(self.year_from_1970) + 1970;
		(year == 2000) || (((year % 4) == 0) && ((year % 100) != 0))
	}

	/// Returns the number of days in the current month
	pub fn days_in_month(&self) -> u8 {
		match self.month {
			1 => 31,
			2 if self.is_leap_year() => 29,
			2 => 28,
			3 => 31,
			4 => 30,
			5 => 31,
			6 => 30,
			7 => 31,
			8 => 31,
			9 => 30,
			10 => 31,
			11 => 30,
			12 => 31,
			_ => panic!("Bad timestamp {:?}", self),
		}
	}

	/// Returns the current month as a UK English string (e.g. "August").
	pub fn month_str(&self) -> &'static str {
		match self.month {
			1 => "January",
			2 => "February",
			3 => "March",
			4 => "April",
			5 => "May",
			6 => "June",
			7 => "July",
			8 => "August",
			9 => "September",
			10 => "October",
			11 => "November",
			12 => "December",
			_ => "Unknown",
		}
	}
}

impl core::fmt::Display for Timestamp {
	fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
		write!(
			f,
			"{year:04}-{month:02}-{days:02}T{hours:02}:{minutes:02}:{seconds:02}",
			year = u16::from(self.year_from_1970) + 1970u16,
			month = self.month,
			days = self.days,
			hours = self.hours,
			minutes = self.minutes,
			seconds = self.seconds,
		)
	}
}

impl TimeContext {
	/// Get the current date/time. Uses a spin-lock so do not call concurrently.
	pub fn get_timestamp(&self) -> Timestamp {
		let mut inner = self.inner.lock();
		inner.get_timestamp()
	}

	/// Get the current date/time. Uses a spin-lock so do not call concurrently.
	pub fn get_posix_time(&self) -> i64 {
		let mut inner = self.inner.lock();
		inner.get_posix_time()
	}

	/// Set the current date/time to the given value and notes the current tick
	/// count from the video system. Uses a spin-lock so do not call
	/// concurrently.
	pub fn set_timestamp(&self, timestamp: Timestamp) {
		let mut inner = self.inner.lock();
		inner.timestamp = timestamp;
		inner.timestamp_tick_count = get_time_ticks();
	}
}

impl TimeContextInner {
	/// Get the current calendar date/time.
	///
	/// Grabs the tick count from the video system, and advances the calendar
	/// date/time stored in this object by the appropriate amount before
	/// returning a copy of that date/time.
	pub fn get_timestamp(&mut self) -> Timestamp {
		let num_ticks = get_time_ticks();
		if num_ticks != self.timestamp_tick_count {
			let delta = num_ticks.wrapping_sub(self.timestamp_tick_count) / RTC_TICK_RATE;
			let days = delta / (3600 * 24);
			let seconds = delta % (3600 * 24);
			self.timestamp.increment(days as u32, seconds as u32);
			self.timestamp_tick_count += delta * RTC_TICK_RATE;
		}
		self.timestamp.clone()
	}

	pub fn get_posix_time(&mut self) -> i64 {
		let ts = self.get_timestamp();

		let (month, year) = if ts.month <= 2 {
			(ts.month + 12, i16::from(ts.year_from_1970) + 1969)
		} else {
			(ts.month, i16::from(ts.year_from_1970) + 1970)
		};

		let mut t: i64 = (365 * i64::from(year)) + i64::from(year / 4) - i64::from(year / 100)
			+ i64::from(year / 400);
		t += (30 * i64::from(month)) + (3 * (i64::from(month) + 1) / 5) + i64::from(ts.days);
		t -= 719561;
		t *= 86400;
		t += (3600 * i64::from(ts.hours)) + (60 * i64::from(ts.minutes)) + i64::from(ts.seconds);

		t
	}
}

/// Get the time in 32768 Hz units.
fn get_time_ticks() -> u64 {
	let rtc = unsafe { bsp::hal::pac::Peripherals::steal().RTC1_NS };
	let overflows = TIMER_OVERFLOWS.load(core::sync::atomic::Ordering::Acquire);
	let mut ticks = rtc.counter.read().bits();
	let overflows2 = TIMER_OVERFLOWS.load(core::sync::atomic::Ordering::Acquire);
	if overflows != overflows2 {
		ticks = rtc.counter.read().bits();
	}
	u64::from(ticks) + (u64::from(overflows) << 24)
}

impl From<nrfxlib_sys::nrf_gnss_datetime_t> for Timestamp {
	fn from(fix: nrfxlib_sys::nrf_gnss_datetime_t) -> Self {
		Self {
			year_from_1970: (fix.year - 1970) as u8,
			month: fix.month,
			days: fix.day,
			hours: fix.hour,
			minutes: fix.minute,
			seconds: fix.seconds,
		}
	}
}

// ==========================================================================
//
// End of file
//
// ==========================================================================
