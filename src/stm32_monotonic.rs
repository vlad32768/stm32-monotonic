use core::sync::atomic::{AtomicU32, Ordering};
//use defmt::println;
use hal::interrupt;
use hal::pac::TIM5;
use hal::prelude::*;
use hal::rcc::Rcc;
use hal::timer::{CounterUs, Event};

/// ## Monotonic time counter for stm32.
///
/// Uses TIM5 32-bit timer with timeout of 1 second. Use high APB frequency (e. g. 84 MHZ) for higher
/// microseconds precision.
///
/// Sets TIM5 interrupt handler, which handles Update event (aka timer overflow) to count seconds
pub struct Stm32Monotonic {
    counter: CounterUs<TIM5>,
}

impl Stm32Monotonic {
    /// ## Create and start the monotonic clock
    /// * Consumes TIM5
    /// * Uses values from `rcc` for timer setup
    /// * Unmasks TIM5 interrupt, Sets TIM5 interrupt handler
    ///   to handle Update event aka timer overflow.
    /// * Use proper AHB1 frequency (>=1Mhz)
    ///
    /// ## Arguments:
    /// * `tim` -- the TIM5 peripheral (consumed)
    /// * `rcc` -- constrained RCC mutable reference
    ///
    /// ## Panics:
    /// If timer auto reload value (1 secs * actual TIM5 frequency)
    /// doesn't fit into 32bit TIM5_ARR register.\
    ///  This should not happen on STM32F4.
    ///
    pub fn new(tim: TIM5, rcc: &mut Rcc) -> Self {
        //println!("creating counter...");
        let mut counter = tim.counter_us(rcc);

        // In theory, panics if timer auto reload value (1 secs * actual TIM5 frequency)
        // doesn't fit into 32bit TIM5_ARR register.
        // IRL it doesn't panic
        counter.start(1000.millis()).unwrap();
        //println!("Creation res:{:?}",err);

        counter.listen(Event::Update); //enable interrupt generation for Timer expire
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM5);
        }

        Self { counter }
    }

    /// Consumes thr monotomic clock and returns the underlying TIM5 peripheral.
    /// The timer is stopped and its interrupt is disabled before return.
    pub fn into_tim5(self) -> TIM5 {
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM5);
        }
        self.counter.release().release()
    }

    /// ## Reset the counter to zero
    /// also restarts the timer
    pub fn reset(&mut self) {
        cortex_m::interrupt::free(|_| {
            OVERFLOWS.store(0, Ordering::Relaxed);
            // doesn't panic if new() is successful,
            self.counter.start(1000.millis()).unwrap();
        });
    }

    /// Seconds
    pub fn seconds(&self) -> u32 {
        OVERFLOWS.load(Ordering::Relaxed)
    }

    /// Milliseconds as u32. Overflows at ~49 days
    pub fn millis32(&self) -> u32 {
        loop {
            let overflows_before = OVERFLOWS.load(Ordering::Relaxed);
            let timer_millis = self.counter.now().duration_since_epoch().to_millis();
            let overflows_after = OVERFLOWS.load(Ordering::Relaxed);

            if overflows_before == overflows_after {
                // no data race
                return overflows_after * 1000 + timer_millis;
            }
        }

        // cortex_m::interrupt::free(|_| {
        //     OVERFLOWS.load(Ordering::Relaxed) * 1000
        //         + self.counter.now().duration_since_epoch().to_millis()
        // })
    }

    /// Microseconds as u32. Overflows at ~4300 secs
    pub fn micros32(&self) -> u32 {
        loop {
            let overflows_before = OVERFLOWS.load(Ordering::Relaxed);
            let timer_micros = self.counter.now().duration_since_epoch().to_micros();
            let overflows_after = OVERFLOWS.load(Ordering::Relaxed);

            if overflows_before == overflows_after {
                return overflows_after * 1_000_000 + timer_micros;
            }
        }
    }

    /// Milliseconds as u64
    pub fn millis64(&self) -> u64 {
        loop {
            let overflows_before = OVERFLOWS.load(Ordering::Relaxed);
            let timer_millis = self.counter.now().duration_since_epoch().to_millis() as u64;
            let overflows_after = OVERFLOWS.load(Ordering::Relaxed);

            if overflows_before == overflows_after {
                // no data race
                return overflows_after as u64 * 1000 + timer_millis;
            }
        }
    }

    /// Microseconds as u64
    pub fn micros64(&self) -> u64 {
        loop {
            let overflows_before = OVERFLOWS.load(Ordering::Relaxed);
            let timer_micros = self.counter.now().duration_since_epoch().to_micros() as u64;
            let overflows_after = OVERFLOWS.load(Ordering::Relaxed);

            if overflows_before == overflows_after {
                return overflows_after as u64 * 1_000_000 + timer_micros;
            }
        }
    }
}

static OVERFLOWS: AtomicU32 = AtomicU32::new(0);

#[interrupt]
fn TIM5() {
    OVERFLOWS.fetch_add(1, Ordering::Relaxed);
    unsafe {
        // Очищаем флаг обновления (Update Interrupt Flag)
        let tim5 = &*TIM5::ptr();
        tim5.sr().modify(|_, w| w.uif().clear_bit());
    }
}
