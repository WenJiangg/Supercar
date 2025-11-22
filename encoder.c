/**
 * ==============================================================================
 * ENCODER.C — Global GPIO ISR owned by encoders; ultrasonic echo routed here
 * ==============================================================================
 * Fix summary:
 * - Debounce relaxed: ENC_MIN_PULSE_US = 80 us (accept more pulses).
 * - Ratio gate disabled: we always accept the pulse width once debounced.
 *   (You can re-enable later if needed.)
 * - Moving average smoothing for RPM/speed.
 * - Distance counts ticks (unchanged).
 * - EXACTLY ONE call to gpio_set_irq_enabled_with_callback() — here.
 * ==============================================================================
 */

#include "encoder.h"
#include "ultrasonic_sensor.h"   // ultrasonic_on_gpio_event(), ULTRASONIC_ECHO_PIN
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <math.h>
#include <stdio.h>

// ----- Geometry / encoder params -----
#ifndef ENCODER_TICKS_PER_REV
#define ENCODER_TICKS_PER_REV   20.0f
#endif
#ifndef WHEEL_DIAMETER_MM
#define WHEEL_DIAMETER_MM       65.0f
#endif
#define WHEEL_CIRCUMF_MM       (M_PI * WHEEL_DIAMETER_MM)

// ----- Noise / debounce -----
#ifndef ENC_MIN_PULSE_US
#define ENC_MIN_PULSE_US         80u     // relaxed debounce to accept widths
#endif
#ifndef ENC_MAX_PULSE_US
#define ENC_MAX_PULSE_US     500000u     // 0.5 s → treat as stalled if older
#endif
#ifndef ENC_SPEED_FILTER_SIZE
#define ENC_SPEED_FILTER_SIZE      5
#endif

// ----- State -----
static volatile uint32_t left_ticks  = 0;
static volatile uint32_t right_ticks = 0;

static volatile uint32_t left_last_edge_us  = 0;
static volatile uint32_t right_last_edge_us = 0;

static volatile uint32_t left_pw_us  = 0;   // last accepted pulse width
static volatile uint32_t right_pw_us = 0;

static volatile uint32_t left_noise_rej  = 0;
static volatile uint32_t right_noise_rej = 0;

// Smoothing buffers (store pulse widths)
static uint32_t lbuf[ENC_SPEED_FILTER_SIZE] = {0};
static uint32_t rbuf[ENC_SPEED_FILTER_SIZE] = {0};
static uint8_t  lidx = 0, ridx = 0, lcnt = 0, rcnt = 0;

// Kinematics (if you prefer time-window update)
static float left_rpm_cache=0.0f, right_rpm_cache=0.0f;
static float left_spd_cache=0.0f, right_spd_cache=0.0f;
static float left_dist_m=0.0f, right_dist_m=0.0f;

// ----- Helpers -----
static inline float pulse_to_rpm(uint32_t pw_us){
    if (pw_us == 0 || pw_us > ENC_MAX_PULSE_US) return 0.0f;
    float slots_per_sec = 1e6f / (float)pw_us;
    float rev_per_sec   = slots_per_sec / ENCODER_TICKS_PER_REV;
    return rev_per_sec * 60.0f;
}
static inline float pulse_to_cm_s(uint32_t pw_us){
    if (pw_us == 0 || pw_us > ENC_MAX_PULSE_US) return 0.0f;
    float slots_per_sec = 1e6f / (float)pw_us;
    float rev_per_sec   = slots_per_sec / ENCODER_TICKS_PER_REV;
    return (rev_per_sec * WHEEL_CIRCUMF_MM) / 10.0f;
}
static inline float pulses_to_m(uint32_t pulses){
    float rev = (float)pulses / ENCODER_TICKS_PER_REV;
    return (rev * WHEEL_CIRCUMF_MM) / 1000.0f;
}
static uint32_t movavg(uint32_t newest, uint32_t *buf, uint8_t *idx, uint8_t *cnt){
    buf[*idx] = newest;
    *idx = (uint8_t)((*idx + 1) % ENC_SPEED_FILTER_SIZE);
    if (*cnt < ENC_SPEED_FILTER_SIZE) (*cnt)++;
    uint32_t sum = 0;
    for (uint8_t i=0;i<*cnt;i++) sum += buf[i];
    return (sum / (*cnt));
}

// ----- THE one global ISR -----
static void encoder_gpio_cb(uint gpio, uint32_t events)
{
    uint32_t now = time_us_32();

    // LEFT encoder edges
    if (gpio == ENC_LEFT_PIN) {
        uint32_t pw = now - left_last_edge_us;
        if (pw >= ENC_MIN_PULSE_US) {
            // Accept width (ratio gate disabled for robustness)
            left_pw_us = pw;
            left_ticks++;
            left_last_edge_us = now;
        } else {
            left_noise_rej++;
        }
    }
    // RIGHT encoder edges
    else if (gpio == ENC_RIGHT_PIN) {
        uint32_t pw = now - right_last_edge_us;
        if (pw >= ENC_MIN_PULSE_US) {
            right_pw_us = pw;
            right_ticks++;
            right_last_edge_us = now;
        } else {
            right_noise_rej++;
        }
    }

    // Route ultrasonic echo through the same ISR
    if (gpio == ULTRASONIC_ECHO_PIN) {
        ultrasonic_on_gpio_event(gpio, events);
    }
}

// ----- Public API -----
void encoder_init(void)
{
    // Pins
    gpio_init(ENC_LEFT_PIN);  gpio_set_dir(ENC_LEFT_PIN,  GPIO_IN); gpio_pull_up(ENC_LEFT_PIN);
    gpio_init(ENC_RIGHT_PIN); gpio_set_dir(ENC_RIGHT_PIN, GPIO_IN); gpio_pull_up(ENC_RIGHT_PIN);
    sleep_ms(5);

    // Install the ONE global callback here; enable both encoder IRQs
    gpio_set_irq_enabled_with_callback(ENC_LEFT_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_cb);

    // Do NOT install another _with_callback anywhere else
    gpio_set_irq_enabled(ENC_RIGHT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Reset state
    uint32_t now = time_us_32();
    left_last_edge_us = right_last_edge_us = now;
    left_ticks = right_ticks = 0;
    left_pw_us = right_pw_us = 0;
    left_noise_rej = right_noise_rej = 0;

    lidx=ridx=0; lcnt=rcnt=0;
    for (int i=0;i<ENC_SPEED_FILTER_SIZE;i++){ lbuf[i]=0; rbuf[i]=0; }

    printf("✓ Encoders ready (global ISR owned by encoders; ultrasonic routed)\n");
}

void encoder_reset_glitch_filter(void)
{
    uint32_t irq = save_and_disable_interrupts();
    lidx=ridx=0; lcnt=rcnt=0;
    for (int i=0;i<ENC_SPEED_FILTER_SIZE;i++){ lbuf[i]=0; rbuf[i]=0; }
    left_pw_us = right_pw_us = 0;
    left_noise_rej = right_noise_rej = 0;
    restore_interrupts(irq);
}

// Optional: time-window kinematics (call ~every 10–20 ms if you want cached values)
void encoder_update_kinematics(void)
{
    static uint32_t last_us = 0;
    uint32_t now = time_us_32();
    if (last_us == 0) { last_us = now; return; }

    float dt_s = (now - last_us) * 1e-6f;
    if (dt_s < 0.010f) return; // ~10 ms window

    // Snapshot pulse widths atomically
    uint32_t irq = save_and_disable_interrupts();
    uint32_t lpw = left_pw_us;
    uint32_t rpw = right_pw_us;
    uint32_t lticks = left_ticks;
    uint32_t rticks = right_ticks;
    uint32_t lts = left_last_edge_us;
    uint32_t rts = right_last_edge_us;
    restore_interrupts(irq);

    // Stall protection (no recent edges)
    if ((now - lts) > ENC_MAX_PULSE_US) lpw = 0;
    if ((now - rts) > ENC_MAX_PULSE_US) rpw = 0;

    // Smooth widths
    if (lpw) lpw = movavg(lpw, lbuf, &lidx, &lcnt);
    if (rpw) rpw = movavg(rpw, rbuf, &ridx, &rcnt);

    // Convert
    left_rpm_cache  = pulse_to_rpm(lpw);
    right_rpm_cache = pulse_to_rpm(rpw);
    left_spd_cache  = pulse_to_cm_s(lpw);
    right_spd_cache = pulse_to_cm_s(rpw);

    // Distance from ticks (monotonic)
    left_dist_m  = pulses_to_m(lticks);
    right_dist_m = pulses_to_m(rticks);

    last_us = now;
}

// Getters — if you aren’t calling encoder_update_kinematics(), we compute on demand
static uint32_t snapshot_pw(volatile uint32_t *pw, volatile uint32_t *last_ts){
    uint32_t irq = save_and_disable_interrupts();
    uint32_t v   = *pw;
    uint32_t ts  = *last_ts;
    restore_interrupts(irq);
    if ((time_us_32() - ts) > ENC_MAX_PULSE_US) return 0; // stalled
    return v;
}

float encoder_get_left_rpm(void){
    // If you call encoder_update_kinematics(), prefer left_rpm_cache
    uint32_t pw = snapshot_pw(&left_pw_us, &left_last_edge_us);
    if (pw) pw = movavg(pw, lbuf, &lidx, &lcnt);
    return pulse_to_rpm(pw);
}
float encoder_get_right_rpm(void){
    uint32_t pw = snapshot_pw(&right_pw_us, &right_last_edge_us);
    if (pw) pw = movavg(pw, rbuf, &ridx, &rcnt);
    return pulse_to_rpm(pw);
}
float encoder_get_left_speed_cm_s(void){
    uint32_t pw = snapshot_pw(&left_pw_us, &left_last_edge_us);
    if (pw) pw = movavg(pw, lbuf, &lidx, &lcnt);
    return pulse_to_cm_s(pw);
}
float encoder_get_right_speed_cm_s(void){
    uint32_t pw = snapshot_pw(&right_pw_us, &right_last_edge_us);
    if (pw) pw = movavg(pw, rbuf, &ridx, &rcnt);
    return pulse_to_cm_s(pw);
}
float encoder_get_left_distance_m(void){  return left_dist_m;  }
float encoder_get_right_distance_m(void){ return right_dist_m; }

// Raw pulses
uint32_t encoder_get_left_pulses(void){  return left_ticks;  }
uint32_t encoder_get_right_pulses(void){ return right_ticks; }

// Diagnostics
uint32_t encoder_get_left_noise_count(void){  return left_noise_rej;  }
uint32_t encoder_get_right_noise_count(void){ return right_noise_rej; }
void encoder_reset_noise_counters(void){ left_noise_rej=right_noise_rej=0; }
void encoder_print_diagnostics(void){
    printf("\n=== ENCODER DIAGNOSTICS ===\n");
    printf("Left:  pulses=%lu  noise_rej=%lu  rpm=%.1f  cm/s=%.2f  dist=%.2fm\n",
           (unsigned long)left_ticks, (unsigned long)left_noise_rej,
           encoder_get_left_rpm(), encoder_get_left_speed_cm_s(), left_dist_m);
    printf("Right: pulses=%lu  noise_rej=%lu  rpm=%.1f  cm/s=%.2f  dist=%.2fm\n",
           (unsigned long)right_ticks, (unsigned long)right_noise_rej,
           encoder_get_right_rpm(), encoder_get_right_speed_cm_s(), right_dist_m);
    printf("===========================\n");
}
