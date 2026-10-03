// main.rs - Embassyを使ったADC入力とPWM出力のPI制御
//
// GPIO26のADC値と目標値の誤差および積分値で、GPIO4のPWM出力を調整します。
// PWM出力を増やすとADC値も増える回路を想定しています。
// 制御値とADC値を約100 msごとに更新・ログ出力します。
//
// 実行方法: cargo run

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel, InterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Pull;
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_time::{Instant, Timer};
use panic_probe as _;

const PROPORTIONAL_GAIN: f32 = 0.3;
const INTEGRAL_GAIN: f32 = 0.1;
const PWM_PERIOD_COUNTS: u16 = 1024;

bind_interrupts!(
    struct Irqs {
        ADC_IRQ_FIFO => InterruptHandler;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!(
        "ADC/PWM PI control start! Kp: {}, Ki: {}",
        PROPORTIONAL_GAIN, INTEGRAL_GAIN
    );
    let p = embassy_rp::init(Default::default());
    let mut adc = Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let mut p26 = Channel::new_pin(p.PIN_26, Pull::None);

    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 1u8;

    let mut c = embassy_rp::pwm::Config::default();
    c.top = PWM_PERIOD_COUNTS - 1;
    c.divider = divider.into();

    info!(
        "PWM frequency: {} Hz, period: {} counts",
        clock_freq_hz / (u32::from(divider) * u32::from(PWM_PERIOD_COUNTS)),
        PWM_PERIOD_COUNTS
    );

    let mut pwm = Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, c);
    let target: u16 = 3000;
    let max_duty = pwm.max_duty_cycle();
    let mut integral_term: f32 = 0.0;
    let mut previous_time = Instant::now();

    loop {
        let level = adc.read(&mut p26).await.unwrap();
        let now = Instant::now();
        let elapsed_secs = (now - previous_time).as_micros() as f32 / 1_000_000.0;
        previous_time = now;

        let error = i32::from(target) - i32::from(level);
        let normalized_error = error as f32 / 4095.0;
        let proportional_term = PROPORTIONAL_GAIN * normalized_error;
        let candidate_integral = integral_term + INTEGRAL_GAIN * normalized_error * elapsed_secs;
        let candidate_output = proportional_term + candidate_integral;

        if (0.0..=1.0).contains(&candidate_output)
            || (candidate_output > 1.0 && normalized_error < 0.0)
            || (candidate_output < 0.0 && normalized_error > 0.0)
        {
            integral_term = candidate_integral;
        }

        let duty_ratio = (proportional_term + integral_term).clamp(0.0, 1.0);
        let duty_count = (duty_ratio * max_duty as f32) as u16;

        pwm.set_duty_cycle(duty_count).unwrap();
        info!(
            "ADC: {}, target: {}, error: {}, I: {}, PWM: {}/{}",
            level, target, error, integral_term, duty_count, max_duty
        );
        Timer::after_millis(10).await;
    }
}
