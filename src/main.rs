// main.rs - Embassyを使ったADC入力とPWM出力のP制御
//
// GPIO26のADC値と目標値の誤差に比例して、GPIO4のPWM出力を調整します。
// PWM出力を増やすとADC値も増える回路を想定しています。
// 制御値とADC値を1秒ごとに更新・ログ出力します。
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
use embassy_time::Timer;
use panic_probe as _;

const PROPORTIONAL_GAIN: f32 = 0.5;

bind_interrupts!(
    struct Irqs {
        ADC_IRQ_FIFO => InterruptHandler;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!(
        "ADC/PWM proportional control start! Kp: {}",
        PROPORTIONAL_GAIN
    );
    let p = embassy_rp::init(Default::default());
    let mut adc = Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let mut p26 = Channel::new_pin(p.PIN_26, Pull::None);

    let desired_freq_hz = 25_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut c = embassy_rp::pwm::Config::default();
    c.top = period;
    c.divider = divider.into();

    let mut pwm = Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, c);
    let target: u16 = 1000;
    let max_duty = pwm.max_duty_cycle();

    loop {
        let level = adc.read(&mut p26).await.unwrap();
        let error = i32::from(target) - i32::from(level);
        let duty_ratio = (PROPORTIONAL_GAIN * error as f32 / 4095.0).clamp(0.0, 1.0);
        let duty_count = (duty_ratio * max_duty as f32) as u16;

        pwm.set_duty_cycle(duty_count).unwrap();
        info!(
            "ADC: {}, target: {}, error: {}, PWM: {}/{}",
            level, target, error, duty_count, max_duty
        );
        Timer::after_secs(1).await;
    }
}
