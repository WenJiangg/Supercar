#include "ultrasonic_sensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// ---- IRQ state (echo timing) ----
static volatile uint32_t t_rise=0, t_fall=0;
static volatile bool echo_ready=false;
static volatile int echo_err=0;

// ---- measurement state ----
static bool in_progress=false;
static uint32_t meas_start_us=0, last_ping_us=0, last_servo_move_us=0;
static float last_cm=-1.0f;
static us_status_t last_status=US_NOT_READY;

// ---- median filter ----
#if (US_FILTER_COUNT != 3) && (US_FILTER_COUNT != 5)
#error "US_FILTER_COUNT must be 3 or 5"
#endif
static float  filt[US_FILTER_COUNT]={0};
static uint8_t fidx=0; static bool ffull=false;

static float median3(float a,float b,float c){
    if(a>b){float t=a;a=b;b=t;} if(b>c){float t=b;b=c;c=t;} if(a>b){float t=a;a=b;b=t;} return b;
}
static float median5(float a,float b,float c,float d,float e){
    if(a>b){float t=a;a=b;b=t;} if(c>d){float t=c;c=d;d=t;}
    if(a<c){ } else {float t=a;a=c;c=t;}
    if(b>d){float t=b;b=d;d=t;}
    if(b>c){float t=b;b=c;c=t;}
    if(c>e){float t=c;c=e;e=t;}
    if(b>c){float t=b;b=c;c=t;}
    if(c>d){float t=c;c=d;d=t;}
    return c;
}

static inline void trig_10us(void){
    gpio_put(ULTRASONIC_TRIG_PIN,1);
    sleep_us(12);
    gpio_put(ULTRASONIC_TRIG_PIN,0);
}

void ultrasonic_init(void){
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);

    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    // IMPORTANT: level-shift ECHO to 3.3V if your module outputs 5V.

    // Enable ECHO edges — BUT DO NOT install a global callback here.
    gpio_set_irq_enabled(ULTRASONIC_ECHO_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    last_servo_move_us = time_us_32();
}

// Called by the single global GPIO ISR (in encoder.c)
void ultrasonic_on_gpio_event(uint gpio, uint32_t events){
    if (gpio != ULTRASONIC_ECHO_PIN) return;
    uint32_t now = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE) {
        t_rise = now; echo_ready=false; echo_err=0;
    }
    if (events & GPIO_IRQ_EDGE_FALL) {
        if(now<=t_rise){ echo_err=US_INVALID_TOO_CLOSE; echo_ready=true; return; }
        uint32_t pulse = now - t_rise;
        if      (pulse < US_MIN_VALID_US) echo_err=US_INVALID_TOO_CLOSE;
        else if (pulse > US_MAX_VALID_US) echo_err=US_TIMEOUT;
        else                              echo_err=0;
        t_fall = now; echo_ready=true;
    }
}

void ultrasonic_start_read(void){
    echo_ready=false; echo_err=0;
    meas_start_us = time_us_32();
    trig_10us();
    in_progress=true;
    last_ping_us = meas_start_us;
    last_status  = US_NOT_READY;
}

bool ultrasonic_update(void){
    if(!in_progress) return true;

    if(echo_ready){
        if(echo_err==0){
            uint32_t pulse = t_fall - t_rise;
            float m  = (pulse * 0.000343f) * 0.5f;
            float cm = m * 100.0f;
#if (US_FILTER_COUNT==3)
            filt[fidx]=cm; fidx=(fidx+1)%3; if(fidx==0) ffull=true;
            last_cm = ffull ? median3(filt[0],filt[1],filt[2]) : cm;
#else
            filt[fidx]=cm; fidx=(fidx+1)%5; if(fidx==0) ffull=true;
            last_cm = ffull ? median5(filt[0],filt[1],filt[2],filt[3],filt[4]) : cm;
#endif
            last_status=US_OK;
        }else if(echo_err==US_TIMEOUT){
            last_status=US_TIMEOUT; last_cm=-1.0f;
        }else{
            last_status=US_INVALID_TOO_CLOSE; last_cm=0.0f;
        }
        in_progress=false;
        return true;
    }

    // safety timeout
    if(time_us_32() - meas_start_us > (US_MAX_VALID_US + 2000u)){
        last_status=US_TIMEOUT; last_cm=-1.0f; in_progress=false; return true;
    }
    return false;
}

us_status_t ultrasonic_get_last_reading_cm(float *cm_out){
    if(last_status==US_OK && cm_out) *cm_out = last_cm;
    return last_status;
}

// helpers for servo/spacing gates
void ultrasonic_mark_servo_moved(uint32_t now_us){ last_servo_move_us = now_us; }
bool ultrasonic_servo_settled(uint32_t now_us){ return (now_us - last_servo_move_us) >= US_SERVO_SETTLE_US; }
bool ultrasonic_ping_spacing_ok(uint32_t now_us){ return (now_us - last_ping_us) >= US_PING_SPACING_MS*1000u; }
