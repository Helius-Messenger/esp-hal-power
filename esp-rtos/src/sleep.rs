//! Power management utilities.

#[cfg(supports_tagmem_power_down)]
use esp_hal::rtc_cntl::{CacheTagRetentionMemory, CacheTagRetentionMemoryError};
#[cfg(supports_cpu_power_down)]
use esp_hal::rtc_cntl::{CpuRetentionMemory, CpuRetentionMemoryError};
#[cfg(multi_core)]
use esp_hal::system::Cpu;
use esp_hal::{
    peripherals::LPWR,
    rtc_cntl::{
        WakeLock,
        sleep::{LightSleep, LowPower, RtcSleepConfig},
    },
    time::{Duration, Instant},
};
use esp_sync::raw::{RawLock, SingleCoreInterruptLock};

use crate::{SCHEDULER, task::IdleFn};
#[cfg(multi_core)]
use crate::{run_queue::RunSchedulerOn, task};

const LIGHT_SLEEP_MIN_US: u64 =
    esp_config::esp_config_int!(u32, "ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US") as u64;

/// Sleep handles.
pub struct Sleep {
    /// The handle that allows you to enter deep sleep.
    #[cfg(sleep_deep_sleep)]
    pub deep_sleep: DeepSleep,

    /// The idle hook to use for light sleep.
    pub light_sleep_hook: IdleFn,
}

#[cfg(supports_cpu_power_down)]
impl Sleep {
    /// Lets automatic light sleep power the CPU domain down.
    ///
    /// A light sleep keeps the CPU state in `memory` while the CPU domain has no power, which
    /// draws less current than a light sleep that keeps the domain powered. The saving costs the
    /// memory, and it adds the save and the restore to the sleep and the wake.
    ///
    /// Every light sleep after this call powers the CPU domain down. Nothing turns it off again,
    /// because the memory stays with the driver for the life of the program.
    ///
    /// # Errors
    ///
    /// Returns [`CpuRetentionMemoryError`] if a buffer is installed already, or if `memory` is
    /// outside the range that the retention hardware reaches. See
    /// [`LowPower::install_cpu_retention_memory`] for how to place the static.
    pub fn enable_cpu_powerdown(
        &mut self,
        memory: &'static mut CpuRetentionMemory,
    ) -> Result<(), CpuRetentionMemoryError> {
        // `configure` took `LPWR`, so this is the only driver, and the idle hook takes it the same
        // way for every sleep.
        LowPower::new(unsafe { LPWR::steal() }).install_cpu_retention_memory(memory)
    }

    /// Keeps the cache tag memory across a light sleep that powers the CPU domain down.
    ///
    /// This shortens the wake, because the caches keep what they held before the sleep. Without
    /// it, the wake path invalidates both caches, and the code that runs next takes a miss for
    /// every line it needs. The memory it costs buys latency only, never correctness.
    ///
    /// This does nothing on its own: the tag memory is lost only when the CPU domain powers down,
    /// which needs [`enable_cpu_powerdown`][Self::enable_cpu_powerdown].
    ///
    /// # Errors
    ///
    /// Returns [`CacheTagRetentionMemoryError`] if a buffer is installed already, or if `memory`
    /// is outside the range that the retention hardware reaches.
    #[cfg(supports_tagmem_power_down)]
    pub fn keep_cache_tags(
        &mut self,
        memory: &'static mut CacheTagRetentionMemory,
    ) -> Result<(), CacheTagRetentionMemoryError> {
        LowPower::new(unsafe { LPWR::steal() }).install_cache_tag_retention_memory(memory)
    }
}

/// A handle you can use to enter deep sleep.
#[cfg(sleep_deep_sleep)]
pub struct DeepSleep {
    lpwr: LPWR<'static>,
}

#[cfg(sleep_deep_sleep)]
impl DeepSleep {
    /// Puts the system into deep sleep.
    ///
    /// The sleep ends when one of the wakeup sources that the drivers enabled becomes active. The
    /// wake from deep sleep resets the chip, so this function does not return.
    ///
    /// # Panics
    ///
    /// Panics if no wakeup source is enabled, because nothing could end the sleep.
    pub fn deep_sleep(&mut self) -> ! {
        let mut lpwr = LowPower::new(self.lpwr.reborrow());
        lpwr.sleep_deep(RtcSleepConfig::deep())
    }
}

/// Creates resources for managing light/deep sleep with `esp-rtos`.
///
/// The returned [`Sleep`] struct contains the idle hook and a deep sleep handle,
/// if deep sleep is supported.
///
/// Pass the idle hook to [`start_with_idle_hook`] to enable automatic light sleep.
///
/// Each time the scheduler runs out of ready tasks, the hook disables interrupts on this core,
/// takes the scheduler lock, and checks that:
/// - no [`WakeLock`] is held,
/// - all cores are idle,
/// - the next wakeup is at least `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` microseconds away.
///
/// If all hold, it releases the lock and enters light sleep with interrupts still disabled. The
/// sleep path keeps interrupts disabled, and parks every other running core for the duration of
/// the sleep. Then it checks the first two conditions again, and a failed check refuses the sleep.
/// After the sleep, the hook takes the scheduler lock again to rearm the time driver and to wake
/// the scheduler on the other cores, then it enables interrupts and falls back to `WFI`.
///
/// A wakeup source can refuse the sleep. The hook then falls back to `WFI`, as it does when a
/// check above fails. A wakeup source can also bound the sleep. The driver clamps the wake timer
/// inside the sleep call, after this hook has armed the next scheduler wakeup. The sooner of the
/// two wins.
///
/// The minimum-residency threshold is configurable via the
/// `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` build-time option (default `1000`).
#[doc = ""]
#[doc = cfg_select! {
    supports_cpu_power_down => "Light sleep keeps the CPU domain powered, unless the program calls [`Sleep::enable_cpu_powerdown`].",
    _ => "Light sleep keeps the CPU domain powered.",
}]
#[doc = ""]
/// See [`WakeLock`] for the wake-lock contract that governs when sleeping is safe.
///
/// [`start_with_idle_hook`]: crate::start_with_idle_hook
pub fn configure(lpwr: LPWR<'static>) -> Sleep {
    #[cfg(not(sleep_deep_sleep))]
    let _ = lpwr;

    Sleep {
        #[cfg(sleep_deep_sleep)]
        deep_sleep: DeepSleep { lpwr },
        light_sleep_hook: auto_light_sleep_hook,
    }
}

// ── Helius fork: light-sleep / timer-queue diagnostics (bench aids) ──────────
/// Index into [`sleep_diag`]: passes refused because a wake lock was held.
pub const DIAG_GATE_WAKELOCK: usize = 0;
/// Index into [`sleep_diag`]: sleeps refused by a wakeup source or the final check
/// (`LightSleep::Refused`, e.g. the BLE controller awake).
pub const DIAG_GATE_DEADLINE: usize = 1;
/// Index into [`sleep_diag`]: passes refused because a core or the run queue was busy.
pub const DIAG_GATE_NOT_IDLE: usize = 2;
/// Index into [`sleep_diag`]: passes refused because the next wakeup was too near.
pub const DIAG_GATE_TOO_SOON: usize = 3;
/// Index into [`sleep_diag`]: light sleeps entered.
pub const DIAG_SLEPT: usize = 4;
/// Index into [`sleep_diag`]: microseconds spent in light sleep.
pub const DIAG_SLEPT_US: usize = 5;

/// Index into [`sleep_diag`]: first of four microsecond totals, one per gate in the same
/// order as the gate counters, attributing the time between idle passes to the gate that
/// refused the previous pass.
pub const DIAG_GATE_US: usize = 6;

// ⚠️ Every counter in this module is a NATIVE 32-bit atomic, never a 64-bit one.
// Xtensa has no 64-bit atomics, so `portable_atomic::AtomicU64` falls back to a global
// table of 67 spinlocks picked by address hash, taken WITHOUT masking interrupts. These
// counters are bumped inside `SCHEDULER.with` (the idle hook, the embassy timer queue),
// i.e. while holding the scheduler lock. If the other core holds the same hashed lock (any
// unrelated `AtomicU64` that shares the bucket) and its task-switch interrupt fires, that
// interrupt spins for the scheduler lock while this core spins for the fallback lock: both
// cores hang with interrupts masked and no watchdog can run. HW-captured 2026-11-09 on an
// ESP32-S3 (core 0 in `_embassy_time_schedule_wake` on `SCHED_HIST`'s bucket, core 1 in
// `trigger_task_switch` interrupted inside an app `AtomicU64::store`).
use core::sync::atomic::AtomicU32;

/// A microsecond total kept as whole milliseconds plus a sub-millisecond remainder, both
/// native 32-bit atomics (49 days before the ms word wraps, vs 71 minutes for raw µs).
struct UsTotal {
    ms: AtomicU32,
    rem_us: AtomicU32,
}

impl UsTotal {
    const fn new() -> Self {
        Self { ms: AtomicU32::new(0), rem_us: AtomicU32::new(0) }
    }

    fn add(&self, us: u64) {
        use core::sync::atomic::Ordering::Relaxed;
        let r = self.rem_us.fetch_add((us % 1000) as u32, Relaxed) + (us % 1000) as u32;
        let carry = r / 1000;
        if carry > 0 {
            self.rem_us.fetch_sub(carry * 1000, Relaxed);
        }
        self.ms.fetch_add((us / 1000) as u32 + carry, Relaxed);
    }

    fn get(&self) -> u64 {
        use core::sync::atomic::Ordering::Relaxed;
        self.ms.load(Relaxed) as u64 * 1000 + self.rem_us.load(Relaxed) as u64
    }
}

static DIAG_COUNT: [AtomicU32; DIAG_SLEPT_US] = [const { AtomicU32::new(0) }; DIAG_SLEPT_US];
/// `DIAG_SLEPT_US` then the four `DIAG_GATE_US` totals.
static DIAG_US: [UsTotal; 5] = [const { UsTotal::new() }; 5];
/// Low 32 bits of the last idle-pass time (µs); only ever differenced, so wrapping is fine.
static DIAG_LAST_PASS_US: AtomicU32 = AtomicU32::new(0);
static DIAG_LAST_GATE: portable_atomic::AtomicUsize = portable_atomic::AtomicUsize::new(usize::MAX);

fn diag_count(idx: usize) {
    DIAG_COUNT[idx].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    if idx < DIAG_SLEPT {
        let now = crate::now() as u32;
        let last = DIAG_LAST_PASS_US.swap(now, core::sync::atomic::Ordering::Relaxed);
        let prev = DIAG_LAST_GATE.swap(idx, core::sync::atomic::Ordering::Relaxed);
        if prev < DIAG_SLEPT && last != 0 {
            DIAG_US[1 + prev].add(now.wrapping_sub(last) as u64);
        }
    } else {
        DIAG_LAST_GATE.store(usize::MAX, core::sync::atomic::Ordering::Relaxed);
    }
}

/// Snapshot of the light-sleep idle hook's gate/sleep counters, indexed by the `DIAG_*`
/// constants. Diagnostic aid for judging why the chip does or does not sleep.
pub fn sleep_diag() -> [u64; 10] {
    core::array::from_fn(|i| {
        if i < DIAG_SLEPT_US {
            DIAG_COUNT[i].load(core::sync::atomic::Ordering::Relaxed) as u64
        } else {
            DIAG_US[i - DIAG_SLEPT_US].get()
        }
    })
}

/// Histogram of the REQUESTED light-sleep duration (next_wakeup - now) at each committed
/// sleep, so a bench can see the wake cadence (what limits sleep length). Buckets (ms):
/// [<2, 2-4, 4-8, 8-16, 16-32, 32-64, 64-160, >=160].
static SLEEP_HIST: [AtomicU32; 8] = [const { AtomicU32::new(0) }; 8];

fn hist_bump(us: u64) {
    let ms = us / 1000;
    let b = if ms < 2 {
        0
    } else if ms < 4 {
        1
    } else if ms < 8 {
        2
    } else if ms < 16 {
        3
    } else if ms < 32 {
        4
    } else if ms < 64 {
        5
    } else if ms < 160 {
        6
    } else {
        7
    };
    SLEEP_HIST[b].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
}

/// Snapshot of the requested-sleep-duration histogram (see [`SLEEP_HIST`]).
pub fn sleep_hist() -> [u64; 8] {
    core::array::from_fn(|i| SLEEP_HIST[i].load(core::sync::atomic::Ordering::Relaxed) as u64)
}

/// Attributes timer-queue task wakeups by task priority, so a bench can see WHICH tasks are
/// waking the system (the "remaining wakers"). Buckets: [prio 0 = app/thread executor,
/// prio 1-2 = LoRa hp-thread / esp-radio timer thread, prio 3-15, prio 16-31 = BTDM
/// controller task]. Only counts wakeups delivered via the RTOS timer queue (embassy
/// `Timer` expirations / scheduled task wakeups), not direct hardware-IRQ wakeups.
static WAKE_BY_PRIO: [AtomicU32; 4] = [const { AtomicU32::new(0) }; 4];

pub(crate) fn note_timer_wake(prio: usize) {
    let b = if prio == 0 {
        0
    } else if prio <= 2 {
        1
    } else if prio <= 15 {
        2
    } else {
        3
    };
    WAKE_BY_PRIO[b].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
}

/// Snapshot of timer-queue wakeups by priority bucket (see [`WAKE_BY_PRIO`]).
pub fn wake_by_prio() -> [u64; 4] {
    core::array::from_fn(|i| WAKE_BY_PRIO[i].load(core::sync::atomic::Ordering::Relaxed) as u64)
}

/// The HARDWARE wakeup cause after each committed light sleep, read from the RTC/PMU wakeup
/// status. This is the DEFINITIVE "what woke the SoC": buckets [Timer, Bt, Gpio, Uart, Ext1,
/// Other/none]. Timer = a scheduled deadline (embassy or esp-rtos); Bt = the BLE controller;
/// the rest = peripheral pads / UART RX.
static WAKE_CAUSE: [AtomicU32; 6] = [const { AtomicU32::new(0) }; 6];

fn note_wake_cause() {
    use esp_hal::rtc_cntl::WakeupSource as S;
    let r = esp_hal::rtc_cntl::wakeup_cause();
    let b = if r.contains(S::Timer) {
        0
    } else if r.contains(S::Bt) {
        1
    } else if r.contains(S::Gpio) {
        2
    } else if r.contains(S::Uart0) || r.contains(S::Uart1) {
        3
    } else if r.contains(S::Ext1) {
        4
    } else {
        5
    };
    WAKE_CAUSE[b].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
}

/// Snapshot of the hardware wakeup-cause counters (see [`WAKE_CAUSE`]).
pub fn wake_cause() -> [u64; 6] {
    core::array::from_fn(|i| WAKE_CAUSE[i].load(core::sync::atomic::Ordering::Relaxed) as u64)
}

/// Histogram of the delays EMBASSY tasks schedule (`Timer::after` etc.), so a bench can see
/// each task's period (one hot fast timer vs many slow ones). Buckets (ms): [<4, 4-16,
/// 16-64, 64-256, 256-1024, >=1024].
static SCHED_HIST: [AtomicU32; 6] = [const { AtomicU32::new(0) }; 6];

pub(crate) fn note_sched(delay_us: u64) {
    let ms = delay_us / 1000;
    let b = if ms < 4 {
        0
    } else if ms < 16 {
        1
    } else if ms < 64 {
        2
    } else if ms < 256 {
        3
    } else if ms < 1024 {
        4
    } else {
        5
    };
    SCHED_HIST[b].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
}

/// Snapshot of the embassy scheduled-delay histogram (see [`SCHED_HIST`]).
pub fn sched_hist() -> [u64; 6] {
    core::array::from_fn(|i| SCHED_HIST[i].load(core::sync::atomic::Ordering::Relaxed) as u64)
}

/// Answers whether the sleep that the idle hook decided on may start.
///
/// esp-hal calls this after it parked the other cores. Until the sleep ends, no other core can then
/// take a [`WakeLock`] or make a task ready.
fn may_sleep() -> bool {
    cfg_select! {
        multi_core => {
            !WakeLock::is_active()
                && SCHEDULER.with(|scheduler| {
                    !scheduler.run_queue.has_ready_tasks()
                        && Cpu::all().all(|cpu| scheduler.cpu_idle(cpu))
                })
        }
        _ => !WakeLock::is_active(),
    }
}

extern "C" fn auto_light_sleep_hook() -> ! {
    loop {
        // Interrupts stay off from the decision until the sleep is over. A handler on this core
        // can take a `WakeLock`, and a sleep must not start after that.
        let irq_token = unsafe { SingleCoreInterruptLock.enter() };

        let sleep = SCHEDULER.with(|scheduler| {
            if WakeLock::is_active() {
                diag_count(DIAG_GATE_WAKELOCK);
                return None;
            }

            #[cfg(multi_core)]
            {
                if scheduler.run_queue.has_ready_tasks() {
                    diag_count(DIAG_GATE_NOT_IDLE);
                    return None;
                }
                for cpu in Cpu::all() {
                    if !scheduler.cpu_idle(cpu) {
                        diag_count(DIAG_GATE_NOT_IDLE);
                        return None;
                    }
                }
            }

            let Some(time_driver) = scheduler.time_driver.as_mut() else {
                return None;
            };
            let next_wakeup = time_driver.next_wakeup();

            let mut lpwr = LowPower::new(unsafe { LPWR::steal() });

            // The deadline stays until other code clears it, so each pass writes it, also a pass
            // that makes no sleep. A deadline from an earlier pass expires, and the
            // next sleep then returns immediately. A wakeup source can shorten this deadline
            // inside `sleep_light_if`, or arm the timer when this hook cleared it. This call stays
            // the scheduler's request. `sleep_light_if` restores it.
            if next_wakeup == u64::MAX {
                lpwr.clear_wakeup_deadline();
            } else {
                lpwr.set_wakeup_deadline(Instant::EPOCH + Duration::from_micros(next_wakeup));

                if next_wakeup.saturating_sub(crate::now()) < LIGHT_SLEEP_MIN_US {
                    diag_count(DIAG_GATE_TOO_SOON);
                    return None;
                }
            }

            Some(next_wakeup)
        });

        if let Some(next_wakeup) = sleep {
            let before = crate::now();
            // Each driver enables the wakeup source it owns. This hook does not know which
            // sources are enabled. `sleep_light_if` returns immediately when none are enabled.
            // It also returns immediately when a source refuses the sleep. Do not call it
            // again here. The refusal is still true, and a retry would spin. `WFI` below
            // waits for an interrupt.
            let result = LowPower::new(unsafe { LPWR::steal() })
                .sleep_light_if(RtcSleepConfig::default(), may_sleep);
            // Diagnostics: `Ended` also covers a sleep that never started, so the
            // slept-µs total is the more trustworthy number. A refusal (a wakeup
            // source or the final check) is counted in the DEADLINE slot.
            if result == LightSleep::Ended {
                hist_bump(next_wakeup.saturating_sub(before));
                note_wake_cause();
                diag_count(DIAG_SLEPT);
                DIAG_US[0].add(crate::now().saturating_sub(before));
            } else {
                diag_count(DIAG_GATE_DEADLINE);
            }

            SCHEDULER.with(|scheduler| {
                let Some(time_driver) = scheduler.time_driver.as_mut() else {
                    return;
                };

                // The alarm timer was gated during light sleep, so its pre-armed alarm is
                // stale. Force a re-arm against the restored time base so the tick handler
                // fires promptly and drains the timer queue. A refusal leaves the timer
                // running, so its alarm is still valid.
                if result == LightSleep::Ended {
                    time_driver.rearm(crate::now());
                }

                // Trigger the scheduler on the other core to prevent it from putting
                // the system back to sleep immediately.
                #[cfg(multi_core)]
                for cpu in Cpu::other() {
                    if scheduler.active_cores.contains(cpu) {
                        task::trigger_scheduler(RunSchedulerOn::RunOnCore(cpu));
                    }
                }
            });
        }

        unsafe { SingleCoreInterruptLock.exit(irq_token) };

        esp_hal::interrupt::wait_for_interrupt();
    }
}
