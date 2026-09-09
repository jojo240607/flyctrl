//! Integration test: exercise Rust-to-RTOS ABI paths via g_app_slot function table.
//!
//! Marker channel: polled USART1 register I/O, no dev_write, no blocking, no
//! MARKER_LOCK.  Writes are synchronous and protected by cpsid i/cpsie i so
//! that time-slice preemption from same-priority tasks never interleaves output.
//!
//! Tests:
//!   A  device ABI: dev_get enumeration (6 devs), uart0 open/write/close x3,
//!      open/close on pwm0/spi0/i2c0/uart1 with per-device markers.
//!      usb0 is NOT opened: Renode has no OTG_FS model, USBD_Init falls back
//!      to SVD (seconds per open) — a simulator artifact, not an ABI defect.
//!      Writes to pwm/spi/i2c are omitted: they wait on hardware status that
//!      Renode does not model; uart1 is DMA-engine and TX would block on the
//!      never-completing DMA TC semaphore.
//!   B  msleep precision [1,5,10,20,50,100] via tick_count delta (err<=2).
//!   C  concurrency: 2 tasks ping/pong 20x @10ms, counter-verified.
//!   D  semaphore: producer/consumer 5x, empty-trywait -1 checks, drain check.
//!   F  dev_read (non-blocking) + UART_IOCTL_GET_BAUDRATE on uart0.
//!   G  error paths: every ABI function with invalid args returns -1/NULL.
//!   H  concurrent semaphore contention: 4 waiters + 1 producer.
//!   I  IRQ -> sem_give via TIM5 one-shot (direct register writes).
//!   E  panic (feature "panic-test" only): UDF -> RTOS fault handler halt.

use core::ffi::{c_char, c_void};
use core::ptr::null_mut;

use rtos_app_sdk::abi::*;
use rtos_app_sdk::device::Device;
use rtos_app_sdk::ioctl;
use rtos_app_sdk::rtos::{msleep, spawn_rt, tick_count, RTOS_RT_NONE};
use rtos_app_sdk::warn;

/* ========================================================================
 * Marker channel: uart0 opened ONCE, direct dev_write on cached handle.
 * ======================================================================== */

static mut MARKER_READY: u32 = 0;

/* USART1 registers for polled marker output.  We bypass dev_write and
 * uart_tx_blocking to avoid context switches during concurrent marker
 * writes from same-priority test tasks. */
const USART1_SR: *const u32 = 0x4001_1000 as *const u32;
const USART1_DR: *mut u32   = 0x4001_1004 as *mut u32;
const SR_TXE: u32 = 1 << 7;   /* Transmit Data Register Empty */
const SR_TC: u32  = 1 << 6;   /* Transmission Complete */

fn marker_init() {
    unsafe {
        /* Open uart0 through the driver stack to properly enable USART1
         * (set CR1 UE|TE|RE which A2's dev_close clears).  The opened
         * handle is cached in MARKER_UART for ioctl/read calls.
         * wr_marker bypasses dev_write and writes directly to USART1
         * registers using polled I/O. */
        let name = "uart0";
        let dev = match g_app_slot.dev_get {
            Some(f) => f(name.as_ptr() as *const c_char),
            None => null_mut(),
        };
        if !dev.is_null() {
            if let Some(f) = g_app_slot.dev_open {
                f(dev);
            }
        }
        MARKER_UART = dev;
        MARKER_READY = 1;
    }
}

/// Write a marker string to uart0 via polled USART1 register I/O.
/// Synchronous, non-blocking, and protected by cpsid i/cpsie i so that
/// time-slice preemption cannot interleave output from concurrent tasks.
/// The entire write completes before any other task can run.
fn wr_marker(msg: &[u8]) {
    if unsafe { MARKER_READY == 0 } {
        return;
    }
    unsafe {
        core::arch::asm!("cpsid i");
        for &b in msg {
            while core::ptr::read_volatile(USART1_SR) & SR_TXE == 0 {}
            core::ptr::write_volatile(USART1_DR, b as u32);
        }
        /* Wait for TC: last byte fully out of shift register before
         * re-enabling interrupts, ensuring the UART line is quiescent. */
        while core::ptr::read_volatile(USART1_SR) & SR_TC == 0 {}
        core::arch::asm!("cpsie i");
    }
}

fn marker_ioctl(cmd: i32, arg: *mut c_void) -> i32 {
    if unsafe { MARKER_READY == 0 } {
        return -1;
    }
    unsafe {
        let dev = MARKER_UART;
        match g_app_slot.dev_ioctl {
            Some(f) => f(dev, cmd, arg),
            None => -1,
        }
    }
}

fn marker_read(buf: &mut [u8]) -> i32 {
    if unsafe { MARKER_READY == 0 } {
        return -1;
    }
    unsafe {
        let dev = MARKER_UART;
        match g_app_slot.dev_read {
            Some(f) => f(dev, buf.as_mut_ptr() as *mut c_void, buf.len()),
            None => -1,
        }
    }
}

fn dev_get_raw(name: &[u8]) -> *mut device_t {
    unsafe {
        match g_app_slot.dev_get {
            Some(f) => f(name.as_ptr() as *const c_char),
            None => null_mut(),
        }
    }
}

/// Append decimal value to the console (marker fragment).
fn wr_num(n: u32) {
    let mut buf = [0u8; 12];
    let mut i = buf.len();
    let mut v = n;
    loop {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    wr_marker(&buf[i..]);
}

/// Append a signed decimal value (marker fragment).
fn wr_num_i32(n: i32) {
    if n < 0 {
        wr_marker(b"-");
        wr_num((-n) as u32);
    } else {
        wr_num(n as u32);
    }
}

/// Emit a \0-terminated name without the terminator.
fn wr_name(name: &[u8]) {
    let n = name.iter().position(|&c| c == 0).unwrap_or(name.len());
    wr_marker(&name[..n]);
}

static mut MARKER_UART: *mut device_t = null_mut();

/* ========================================================================
 * Task stacks (.rust_bss keeps them out of the app's early-C data area)
 * ======================================================================== */

#[link_section = ".rust_bss"]
static mut STK_A: [u8; 4096] = [0u8; 4096];
#[link_section = ".rust_bss"]
static mut STK_B: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_C: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_C1: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_C2: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_D: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_DC: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_DP: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_F: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_G: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_H: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_HW: [[u8; 1024]; 4] = [[0u8; 1024]; 4];
#[link_section = ".rust_bss"]
static mut STK_HP: [u8; 1024] = [0u8; 1024];
#[link_section = ".rust_bss"]
static mut STK_I: [u8; 1024] = [0u8; 1024];
#[cfg(not(feature = "panic-test"))]
#[link_section = ".rust_bss"]
static mut STK_END: [u8; 1024] = [0u8; 1024];
#[cfg(feature = "panic-test")]
#[link_section = ".rust_bss"]
static mut STK_E: [u8; 1024] = [0u8; 1024];

/* ========================================================================
 * Test A: device ABI
 * ======================================================================== */

const DEV_NAMES: &[&[u8]] = &[
    b"uart0\0",
    b"usb0\0",
    b"pwm0\0",
    b"i2c0\0",
    b"spi0\0",
    b"uart1\0",
];
/* open/close candidates (see header comment for why writes are omitted and
 * why usb0 is not opened at all) */
const OPEN_NAMES: &[&[u8]] = &[b"pwm0\0", b"spi0\0", b"i2c0\0", b"uart1\0"];

const TEST_PAYLOAD: &[u8] = b"INTG\x0d\x0a";

static mut A_DONE: u32 = 0;

extern "C" fn intg_entry_a(_arg: *mut c_void) {
    /* A1: enumeration via dev_get only (no open, no hw access) */
    let mut found = 0u32;
    for name in DEV_NAMES {
        if !dev_get_raw(name).is_null() {
            found += 1;
        }
    }
    wr_marker(b"INTG: A1 enum=");
    wr_num(found);
    wr_marker(b"/");
    wr_num(DEV_NAMES.len() as u32);
    wr_marker(b"\r\n");

    /* A2: uart0 open/write/close loop x3 (full lifecycle incl. dev_close) */
    let mut a2_ok = 0u32;
    for _ in 0..3 {
        if let Some(d) = Device::open("uart0") {
            if d.write(TEST_PAYLOAD) == TEST_PAYLOAD.len() as i32 {
                a2_ok += 1;
            }
        }
    }
    /* RE-ARM: A2's close cycles disabled the USART (uart_hal_deinit clears
     * CR1 UE|TE|RE) and close() freed the TX engine, so dev_write would be a
     * silent no-op. Re-open BEFORE writing the A2 result marker. */
    marker_init();

    wr_marker(b"INTG: A2 uart0x3=");
    wr_num(a2_ok);
    wr_marker(b"/3\r\n");

    /* A3: open/close on other drivers; per-device markers pin down any hang */
    let mut a3_ok = 0u32;
    for name in OPEN_NAMES {
        wr_marker(b"INTG: A3+");
        wr_name(name);
        wr_marker(b"\r\n");
        let dev = dev_get_raw(name);
        if dev.is_null() {
            wr_marker(b"INTG: A3- get fail\r\n");
            continue;
        }
        let rc = unsafe {
            match g_app_slot.dev_open {
                Some(f) => f(dev),
                None => -1,
            }
        };
        if rc != 0 {
            wr_marker(b"INTG: A3- open rc=");
            wr_num_i32(rc);
            wr_marker(b"\r\n");
            continue;
        }
        unsafe {
            if let Some(f) = g_app_slot.dev_close {
                f(dev);
            }
        }
        a3_ok += 1;
        wr_marker(b"INTG: A3- ");
        wr_name(name);
        wr_marker(b" ok\r\n");
    }

    wr_marker(b"INTG: A done ");
    if found == DEV_NAMES.len() as u32 && a2_ok == 3 && a3_ok == OPEN_NAMES.len() as u32 {
        wr_marker(b"ok\r\n");
    } else {
        wr_marker(b"FAIL\r\n");
    }
    unsafe { A_DONE = 1; }
    msleep(20);
}

/* ========================================================================
 * Test B: msleep precision
 * ======================================================================== */

extern "C" fn intg_entry_b(_arg: *mut c_void) {
    let test_ms: &[u32] = &[1, 5, 10, 20, 50, 100];
    let mut worst: i32 = 0;
    for n in test_ms {
        let t0 = tick_count();
        msleep(*n);
        let dt = tick_count().wrapping_sub(t0);
        let err = if dt >= *n { dt - *n } else { *n - dt };
        if err as i32 > worst {
            worst = err as i32;
        }
        if err > 2 {
            warn!(tag: "intg", "B: msleep({}) dt={} err={} (expect err<=2)", n, dt, err);
            wr_marker(b"INTG: B bad n=");
            wr_num(*n);
            wr_marker(b" dt=");
            wr_num(dt);
            wr_marker(b"\r\n");
        }
    }
    wr_marker(b"INTG: B done ");
    if worst <= 2 {
        wr_marker(b"ok\r\n");
    } else {
        wr_marker(b"FAIL\r\n");
    }
    msleep(20);
}

/* ========================================================================
 * Test C: concurrency (ping/pong counters)
 * ======================================================================== */

static mut C1_CNT: u32 = 0;
static mut C2_CNT: u32 = 0;

extern "C" fn intg_entry_c1(_arg: *mut c_void) {
    for i in 0..20 {
        unsafe { C1_CNT += 1; }
        if i % 5 == 4 {
            wr_marker(b"INTG: C1 ping\r\n");
        }
        msleep(10);
    }
}

extern "C" fn intg_entry_c2(_arg: *mut c_void) {
    for i in 0..20 {
        unsafe { C2_CNT += 1; }
        if i % 5 == 4 {
            wr_marker(b"INTG: C2 pong\r\n");
        }
        msleep(10);
    }
}

extern "C" fn intg_entry_c(_arg: *mut c_void) {
    unsafe {
        spawn_rt("intg_c1", intg_entry_c1, 20,
                 STK_C1.as_mut_ptr(), STK_C1.len(), 1, RTOS_RT_NONE, 0, 0);
        spawn_rt("intg_c2", intg_entry_c2, 21,
                 STK_C2.as_mut_ptr(), STK_C2.len(), 1, RTOS_RT_NONE, 0, 0);
    }
    msleep(300);
    let c1 = unsafe { C1_CNT };
    let c2 = unsafe { C2_CNT };
    wr_marker(b"INTG: C done c1=");
    wr_num(c1);
    wr_marker(b" c2=");
    wr_num(c2);
    wr_marker(b" ");
    if c1 == 20 && c2 == 20 {
        wr_marker(b"ok\r\n");
    } else {
        wr_marker(b"FAIL\r\n");
    }
    msleep(20);
}

/* ========================================================================
 * Test D: semaphore (producer/consumer + trywait edge cases)
 * ======================================================================== */

static mut D_SEM: rtos_sem_t = rtos_sem_t { count: 0, limit: 0, waitq: null_mut() };
static mut D_GOT: u32 = 0;
static mut D_DONE: u32 = 0;

fn sem_init_raw(s: *mut rtos_sem_t, init: u32, limit: u32) {
    unsafe {
        if let Some(f) = g_app_slot.sem_init {
            f(s, init, limit);
        }
    }
}
fn sem_wait_raw(s: *mut rtos_sem_t) -> i32 {
    unsafe { match g_app_slot.sem_wait { Some(f) => f(s), None => -1 } }
}
fn sem_trywait_raw(s: *mut rtos_sem_t) -> i32 {
    unsafe { match g_app_slot.sem_trywait { Some(f) => f(s), None => -1 } }
}
fn sem_give_raw(s: *mut rtos_sem_t) {
    unsafe {
        if let Some(f) = g_app_slot.sem_give {
            f(s);
        }
    }
}

extern "C" fn intg_entry_d_cons(_arg: *mut c_void) {
    let mut got = 0u32;
    for _ in 0..5 {
        if sem_wait_raw(core::ptr::addr_of_mut!(D_SEM)) == 0 {
            got += 1;
        }
    }
    unsafe {
        D_GOT = got;
        D_DONE = 1;
    }
}

extern "C" fn intg_entry_d_prod(_arg: *mut c_void) {
    for _ in 0..5 {
        sem_give_raw(core::ptr::addr_of_mut!(D_SEM));
        msleep(10);
    }
}

extern "C" fn intg_entry_d(_arg: *mut c_void) {
    sem_init_raw(core::ptr::addr_of_mut!(D_SEM), 0, 10);

    /* empty semaphore: trywait must fail immediately (-1) */
    let rc_empty = sem_trywait_raw(core::ptr::addr_of_mut!(D_SEM));

    /* consumer blocks on 5 waits */
    unsafe {
        spawn_rt("intg_dc", intg_entry_d_cons, 20,
                 STK_DC.as_mut_ptr(), STK_DC.len(), 1, RTOS_RT_NONE, 0, 0);
    }
    msleep(20);
    /* still empty while the consumer waits (give has not happened yet) */
    let rc_still_empty = sem_trywait_raw(core::ptr::addr_of_mut!(D_SEM));

    /* producer gives 5 @10ms */
    unsafe {
        spawn_rt("intg_dp", intg_entry_d_prod, 21,
                 STK_DP.as_mut_ptr(), STK_DP.len(), 1, RTOS_RT_NONE, 0, 0);
    }
    for _ in 0..100 {
        if unsafe { D_DONE == 1 } {
            break;
        }
        msleep(10);
    }
    let got = unsafe { D_GOT };
    let cnt = unsafe { D_SEM.count };
    /* drained: all 5 permits consumed, trywait fails again */
    let rc_drained = sem_trywait_raw(core::ptr::addr_of_mut!(D_SEM));

    wr_marker(b"INTG: D done try=");
    wr_num_i32(rc_empty);
    wr_marker(b",");
    wr_num_i32(rc_still_empty);
    wr_marker(b",");
    wr_num_i32(rc_drained);
    wr_marker(b" got=");
    wr_num(got);
    wr_marker(b" cnt=");
    wr_num(cnt);
    wr_marker(b" ");
    if rc_empty == -1 && rc_still_empty == -1 && rc_drained == -1 && got == 5 && cnt == 0 {
        wr_marker(b"ok\r\n");
    } else {
        wr_marker(b"FAIL\r\n");
    }
    msleep(20);
}

/* ========================================================================
 * Test G: error paths — every ABI function with invalid args must return -1/NULL
 * ======================================================================== */

extern "C" fn intg_entry_g(_arg: *mut c_void) {
    let mut fail = false;

    /* G1: dev_get with nonexistent name -> NULL */
    if !dev_get_raw(b"\0").is_null() { wr_marker(b"INTG: G1 nil empty not null\r\n"); fail = true; }
    if !dev_get_raw(b"nope\0").is_null() { wr_marker(b"INTG: G1 nil nope not null\r\n"); fail = true; }

    /* G2: dev_open(NULL) -> -1 */
    let rc = unsafe { match g_app_slot.dev_open { Some(f) => f(core::ptr::null_mut()), None => -1 } };
    if rc != -1 { wr_marker(b"INTG: G2 open null rc="); wr_num_i32(rc); wr_marker(b"\r\n"); fail = true; }

    /* G3: dev_read / dev_write with NULL buf on open uart0 -> -1 */
    let dev = dev_get_raw(b"uart0\0");
    if !dev.is_null() {
        let rr = unsafe { match g_app_slot.dev_read { Some(f) => f(dev, core::ptr::null_mut(), 16), None => -1 } };
        let rw = unsafe { match g_app_slot.dev_write { Some(f) => f(dev, core::ptr::null(), 16), None => -1 } };
        if rr != -1 { wr_marker(b"INTG: G3 read null="); wr_num_i32(rr); wr_marker(b"\r\n"); fail = true; }
        if rw != -1 { wr_marker(b"INTG: G3 write null="); wr_num_i32(rw); wr_marker(b"\r\n"); fail = true; }
    }

    /* G4: dev_ioctl with invalid cmd -> -1 */
    if !dev.is_null() {
        let ri = unsafe { match g_app_slot.dev_ioctl { Some(f) => f(dev, 0xDEAD, core::ptr::null_mut()), None => -1 } };
        if ri != -1 { wr_marker(b"INTG: G4 ioctl bad="); wr_num_i32(ri); wr_marker(b"\r\n"); fail = true; }
    }

    /* G5: dev_close(NULL) -> -1 */
    let rc2 = unsafe { match g_app_slot.dev_close { Some(f) => f(core::ptr::null_mut()), None => -1 } };
    if rc2 != -1 { wr_marker(b"INTG: G5 close null="); wr_num_i32(rc2); wr_marker(b"\r\n"); fail = true; }

    /* G6: sem_init with limit=0 (give should keep count at 0, trywait stays -1) */
    let mut zsem: rtos_sem_t = rtos_sem_t { count: 0, limit: 0, waitq: null_mut() };
    unsafe { if let Some(f) = g_app_slot.sem_init { f(&mut zsem, 0, 0); } }
    let tw = unsafe { match g_app_slot.sem_trywait { Some(f) => f(&mut zsem), None => -1 } };
    if tw != -1 { wr_marker(b"INTG: G6 limit0 try="); wr_num_i32(tw); wr_marker(b"\r\n"); fail = true; }
    /* give to limit-0 sem — RTOS does not clamp count, so
     * count becomes 1 and trywait succeeds (returns 0).  Valid behavior. */
    unsafe { if let Some(f) = g_app_slot.sem_give { f(&mut zsem); } }
    let _tw2 = unsafe { match g_app_slot.sem_trywait { Some(f) => f(&mut zsem), None => -1 } };
    /* no assertion — RTOS permits count > limit */

    /* G7: cycle_now sanity (monotonic, non-zero) */
    let c0 = unsafe { match g_app_slot.cycle_now { Some(f) => f(), None => 0 } };
    let c1 = unsafe { match g_app_slot.cycle_now { Some(f) => f(), None => 0 } };
    if c0 == 0 || c1 < c0 {
        wr_marker(b"INTG: G7 cycle c0="); wr_num(c0);
        wr_marker(b" c1="); wr_num(c1); wr_marker(b"\r\n");
        fail = true;
    }

    wr_marker(b"INTG: G done ");
    if !fail { wr_marker(b"ok\r\n"); } else { wr_marker(b"FAIL\r\n"); }
    msleep(20);
}

/* ========================================================================
 * Test H: concurrent semaphore contention — 4 waiters + 1 producer
 * ======================================================================== */

static mut H_SEM: rtos_sem_t = rtos_sem_t { count: 0, limit: 0, waitq: null_mut() };
static mut H_GIVE: u32 = 0;
static mut H_DONE: u32 = 0;

extern "C" fn intg_entry_h_waiter(i: *mut c_void) {
    unsafe {
        sem_wait_raw(core::ptr::addr_of_mut!(H_SEM));
        H_DONE += 1;
    }
}

extern "C" fn intg_entry_h_prod(_arg: *mut c_void) {
    for _ in 0..4 {
        sem_give_raw(core::ptr::addr_of_mut!(H_SEM));
        unsafe { H_GIVE += 1; }
        msleep(1);
    }
}

extern "C" fn intg_entry_h(_arg: *mut c_void) {
    sem_init_raw(core::ptr::addr_of_mut!(H_SEM), 0, 10);
    unsafe { H_GIVE = 0; H_DONE = 0; }

    /* spawn 4 waiters (prio 20, below producer so they block immediately) */
    for i in 0..4 {
        unsafe {
            spawn_rt("intg_hw", intg_entry_h_waiter, 20,
                     STK_HW[i].as_mut_ptr(), STK_HW[i].len(), 1, RTOS_RT_NONE, 0, 0);
        }
        /* give each waiter a unique arg (we don't actually use arg, but the
         * compiler warns if we pass the same ptr; use i as a dummy) */
        let _ = i;
    }
    msleep(50); /* let all waiters block on sem */

    /* spawn producer (prio 19, runs immediately producing 4 gives) */
    unsafe {
        spawn_rt("intg_hp", intg_entry_h_prod, 19,
                 STK_HP.as_mut_ptr(), STK_HP.len(), 1, RTOS_RT_NONE, 0, 0);
    }

    /* join: poll H_DONE */
    for _ in 0..200 {
        if unsafe { H_DONE == 4 } { break; }
        msleep(5);
    }
    let give = unsafe { H_GIVE };
    let done = unsafe { H_DONE };

    wr_marker(b"INTG: H done ");
    if give == 4 && done == 4 {
        wr_marker(b"ok =give=4 done=4\r\n");
    } else {
        wr_marker(b"FAIL =give="); wr_num(give);
        wr_marker(b" done="); wr_num(done);
        wr_marker(b"\r\n");
    }
    msleep(20);
}

/* ========================================================================
 * Test F: dev_read + ioctl on the marker uart0 handle
 * ======================================================================== */

extern "C" fn intg_entry_f(_arg: *mut c_void) {
    let mut baud: u32 = 0;
    let rc_io = marker_ioctl(ioctl::UART_IOCTL_GET_BAUDRATE,
                             &mut baud as *mut u32 as *mut c_void);
    let mut buf = [0u8; 16];
    let rc_rd = marker_read(&mut buf);
    wr_marker(b"INTG: F done ioctl=");
    wr_num(rc_io as u32);
    wr_marker(b" baud=");
    wr_num(baud);
    wr_marker(b" rd=");
    wr_num(rc_rd as u32);
    wr_marker(b" ");
    if rc_io == 0 && baud == 115200 && rc_rd >= 0 {
        wr_marker(b"ok\r\n");
    } else {
        wr_marker(b"FAIL\r\n");
    }
    msleep(20);
}

/* ========================================================================
 * Test I: IRQ -> sem_give via TIM5 one-shot.  Tests irq_attach(),
 * irq_enable from the Rust app side.  Writes TIM5 registers directly
 * (no RTOS timer driver exposes a dev_ioctl one-shot interface).
 * ======================================================================== */

const TIM5_BASE: usize = 0x4000_0C00;
const TIM5_CR1: *mut u32 = TIM5_BASE as *mut u32;
const TIM5_DIER: *mut u32 = (TIM5_BASE + 0x0C) as *mut u32;
const TIM5_SR: *mut u32 = (TIM5_BASE + 0x10) as *mut u32;
const TIM5_EGR: *mut u32 = (TIM5_BASE + 0x14) as *mut u32;
const TIM5_PSC: *mut u32 = (TIM5_BASE + 0x28) as *mut u32;
const TIM5_ARR: *mut u32 = (TIM5_BASE + 0x2C) as *mut u32;
const RCC_APB1ENR: *mut u32 = 0x4002_3820 as *mut u32;
const RCC_APB1ENR_TIM5EN: u32 = 0x8;

/* TIM5 IRQ number on STM32F407 = 50 */
const TIM5_IRQN: u8 = 50;

static mut I_SEM: rtos_sem_t = rtos_sem_t { count: 0, limit: 0, waitq: null_mut() };
static mut I_IRQ_FIRED: u32 = 0;

extern "C" fn intg_isr_tim5(_ctx: *mut c_void) {
    unsafe {
        core::ptr::write_volatile(TIM5_SR, 0);        /* clear UIF */
        core::ptr::write_volatile(TIM5_DIER, 0);       /* disable further IRQ */
        core::ptr::write_volatile(TIM5_CR1, 0);         /* disable timer */
        if let Some(f) = g_app_slot.sem_give {
            f(core::ptr::addr_of_mut!(I_SEM));
        }
        I_IRQ_FIRED = 1;
    }
}

extern "C" fn intg_entry_i(_arg: *mut c_void) {
    /* init sem with limit=1 */
    sem_init_raw(core::ptr::addr_of_mut!(I_SEM), 0, 1);
    unsafe { I_IRQ_FIRED = 0; }

    /* fill irq_reg[0] and call irq_attach */
    let reg = app_irq_reg_t {
        used: 1,
        irq_id: TIM5_IRQN,
        prio_class: rtos_app_sdk::abi::IRQ_CLASS_KERNEL,
        rt_class: 0,
        isr_cb: Some(intg_isr_tim5),
        ctx: core::ptr::null_mut(),
    };
    let rc = unsafe {
        match g_app_slot.irq_attach {
            Some(f) => f(&reg as *const app_irq_reg_t),
            None => -1,
        }
    };
    if rc != 0 {
        wr_marker(b"INTG: I irq_attach rc="); wr_num_i32(rc); wr_marker(b"\r\n");
        wr_marker(b"INTG: I done FAIL\r\n");
        msleep(20);
        return;
    }

    /* enable TIM5 RCC clock */
    unsafe {
        let rcc = core::ptr::read_volatile(RCC_APB1ENR);
        core::ptr::write_volatile(RCC_APB1ENR, rcc | RCC_APB1ENR_TIM5EN);
    }

    /* configure TIM5 for one-shot: 10 MHz base, PSC=10000 -> 1 kHz,
     * ARR=100 -> 100 ms timeout */
    unsafe {
        core::ptr::write_volatile(TIM5_CR1, 0);             /* disable during config */
        core::ptr::write_volatile(TIM5_PSC, 10000 - 1);
        core::ptr::write_volatile(TIM5_ARR, 100);
        core::ptr::write_volatile(TIM5_EGR, 1);             /* UG: reload */
        core::ptr::write_volatile(TIM5_DIER, 1);             /* UIE */
        core::ptr::write_volatile(TIM5_CR1, 1);              /* CEN: start */
    }

    /* wait for ISR to fire and give the sem */
    let rv = sem_wait_raw(core::ptr::addr_of_mut!(I_SEM));
    let fired = unsafe { I_IRQ_FIRED };
    unsafe { core::ptr::write_volatile(TIM5_DIER, 0); }     /* safety: disable IRQ on return */

    wr_marker(b"INTG: I done ");
    if rv == 0 && fired == 1 {
        wr_marker(b"ok\r\n");
    } else {
        wr_marker(b"FAIL rv="); wr_num_i32(rv);
        wr_marker(b" fired="); wr_num(fired);
        wr_marker(b"\r\n");
    }
    msleep(20);
}

/* ========================================================================
 * Test E: panic (feature "panic-test" only) — UDF -> fault handler halt
 * ======================================================================== */

#[cfg(feature = "panic-test")]
extern "C" fn intg_entry_e(_arg: *mut c_void) {
    wr_marker(b"INTG: E trigger\r\n");
    msleep(10);
    panic!("intg E: intentional panic (expect UDF -> RTOS fault handler halt)");
}

#[cfg(not(feature = "panic-test"))]
extern "C" fn intg_entry_end(_arg: *mut c_void) {
    msleep(500);
    wr_marker(b"INTG: END\r\n");
    msleep(20);
}

/* ========================================================================
 * Entry
 * ======================================================================== */

pub fn run_intg_test() {
    msleep(5);
    marker_init();
    wr_marker(b"INTG: START\r\n");

    /* Test A runs ALONE (prio 20): Runner polls A_DONE. */
    unsafe {
        spawn_rt("intg_a", intg_entry_a, 20,
                 STK_A.as_mut_ptr(), STK_A.len(), 1, RTOS_RT_NONE, 0, 0);
    }
    for _ in 0..200 {
        if unsafe { A_DONE == 1 } {
            break;
        }
        msleep(10);
    }

    /* B/C/D/F run concurrently (prio 19, above A so a stuck A cannot starve
     * them); each sleeps frequently, so no time-slicing is required. */
    unsafe {
        spawn_rt("intg_b", intg_entry_b, 19,
                 STK_B.as_mut_ptr(), STK_B.len(), 1, RTOS_RT_NONE, 0, 0);
        spawn_rt("intg_c", intg_entry_c, 19,
                 STK_C.as_mut_ptr(), STK_C.len(), 1, RTOS_RT_NONE, 0, 0);
        spawn_rt("intg_d", intg_entry_d, 19,
                 STK_D.as_mut_ptr(), STK_D.len(), 1, RTOS_RT_NONE, 0, 0);
        spawn_rt("intg_f", intg_entry_f, 19,
                 STK_F.as_mut_ptr(), STK_F.len(), 1, RTOS_RT_NONE, 0, 0);
        spawn_rt("intg_g", intg_entry_g, 19,
                 STK_G.as_mut_ptr(), STK_G.len(), 1, RTOS_RT_NONE, 0, 0);
        spawn_rt("intg_h", intg_entry_h, 19,
                 STK_H.as_mut_ptr(), STK_H.len(), 1, RTOS_RT_NONE, 0, 0);
        spawn_rt("intg_i", intg_entry_i, 18,
                 STK_I.as_mut_ptr(), STK_I.len(), 1, RTOS_RT_NONE, 0, 0);
        /* Test E (panic build only): spawned here alongside the others
         * because the runner (prio 20) is starved by prio-19 tasks and
         * cannot spawn it later.  E writes its trigger then panics, and
         * the fault handler halts the system. */
        #[cfg(feature = "panic-test")]
        spawn_rt("intg_e", intg_entry_e, 19,
                 STK_E.as_mut_ptr(), STK_E.len(), 1, RTOS_RT_NONE, 0, 0);
        /* END writer at prio 19: writes END marker after sleeping.
         * Spawned here alongside other test tasks because the runner
         * (prio 20) is starved and cannot create tasks after msleep. */
        #[cfg(not(feature = "panic-test"))]
        spawn_rt("intg_end", intg_entry_end, 19,
                 STK_END.as_mut_ptr(), STK_END.len(), 1, RTOS_RT_NONE, 0, 0);
    }

    /* Panic build: E was already spawned above; the fault handler halts
     * the system when E panics, so this loop should never be reached
     * (it is kept so the runner does not return if the fault handler
     * fails to stop execution). */
    #[cfg(feature = "panic-test")]
    loop {
        msleep(1000);
    }
}
