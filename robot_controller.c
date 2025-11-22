/**
 * ==============================================================================
 * ROBOT_CONTROLLER.C — Original-feel Line Following + Always-Sweeping Ultrasonic
 * Flow: FOLLOW → DETECT → STOP → WIDTH SCAN → WAIT-CLEAR → FOLLOW
 * ==============================================================================
 * - PID/IMU line following runs every loop (never blocked).
 * - Ultrasonic sweep ALWAYS runs (no gating), but pings only when servo is settled
 *   and spacing is OK; echo window is handled quickly (non-blocking).
 * - On detection (< 24 cm): stop, scan right→left to estimate width, then ping
 *   forward until clear N times, then resume following. No maneuver/turning.
 * ==============================================================================
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "robot_controller.h"
#include "motor_calibration.h"
#include "ir_sensor.h"
#include "encoder.h"
#include "imu.h"
#include "servo_control.h"
#include "ultrasonic_sensor.h"
#include "mqtt_handler.h"
#include "barcode_direction.h"

// ================== PID / LINE FOLLOWING (matches your original) ==================
#define STRAIGHT_SPEED              43
#define CORNER_SPEED                23
#define MOTOR_MAX_ABS              100
#define MAX_ADJUSTMENT              35

#define KP_GAIN                   24.0f
#define KI_GAIN                    0.05f
#define KD_GAIN                   65.0f

#define INTEGRAL_MAX              100.0f
#define DEAD_BAND                  0.10f
#define LINE_TARGET                0.50f

#define MAX_SPEED_CHANGE              5
#define MAX_ERROR_FOR_SPEED_ADJUST  0.40f
#define SPEED_SMOOTHING_FACTOR      0.10f

// IMU assists (lightweight)
#define IMU_HEADING_GAIN           0.25f
#define IMU_STABILITY_GAIN         0.40f
#define IMU_ACCEL_GAIN             0.15f
#define WOBBLE_THRESHOLD            0.12f
#define HEADING_LOCK_THRESH         0.08f
#define MIN_SPEED_FOR_IMU              25
#define HEADING_LOCK_TIMEOUT_S       0.50f

// Cadences (µs)
#define DT_PID_US                   5000u
#define DT_IMU_US                  20000u
#define DT_NET_US                   5000u
#define DT_MQTT_US                150000u

// ================== OBSTACLE / RADAR / CLEAR-WAIT ==================
#define OBSTACLE_THRESHOLD_CM       24.0f   // detection
#define CLEAR_HYSTERESIS_CM          4.0f   // resume when ≥ 28 cm
#define CLEAR_CONSECUTIVE_COUNT         3
#define CLEAR_RECHECK_MS             120

// Sweep limits (deg)
#define ROBOT_RIGHT_DEG             35.0f
#define ROBOT_CENTER_DEG            85.0f
#define ROBOT_LEFT_DEG             120.0f

// Sweep behaviour
#define RADAR_SWEEP_STEP_DEG        14.0f   // bigger hop → faster sweep feel
#define RADAR_POLL_MS               10u     // how often we *try* to progress sweep

// Width scan (stopped)
#define WIDTH_SCAN_STEP_DEG          3.0f
#define SCAN_SETTLE_MS               50     // UI dwell; US driver enforces settle/spacing

// Telemetry / debug
#define TELEMETRY_PAYLOAD_SIZE     768
#define MQTT_PUB_PERIOD_MS         200
#define DEBUG_PRINT_DIVISOR         25
#define DT_ENC_US  20000u

// Barcode scanning
#define DT_BARCODE_US   2000u   // Update barcode scanner every 2ms   


// ================== States ==================
typedef enum { STATE_LINE_FOLLOWING=0, STATE_WIDTH_MEASURE } RobotState;
typedef enum { WS_PREPARE=0, WS_SCANNING, WS_PROCESS, WS_WAIT_CLEAR } WidthScanState;

// ================== State vars ==================
static RobotState     g_master_state;
static WidthScanState g_ws_state;

// PID / speeds
static float pid_integral=0.0f, pid_last_error=0.0f;
static int   cur_left_speed=0, cur_right_speed=0;
static float smoothed_target_speed=STRAIGHT_SPEED;

// IMU assists
static float target_heading=0.0f; static bool heading_locked=false;
static float heading_lock_timer=0.0f, prev_lateral_accel=0.0f;

// Debug stash
static float last_p_term=0.0f, last_i_term=0.0f, last_d_term=0.0f, last_steer=0.0f;

// Radar sweep (FOLLOW mode, non-blocking)
static float radar_angle=ROBOT_CENTER_DEG; static int radar_dir=+1;
static absolute_time_t next_radar_tick;
static bool  radar_waiting_echo=false;
static float radar_pending_next_angle=-1.0f;
static float last_scan_distance=-1.0f;

// Width scan (stopped)
static float ws_angle=ROBOT_RIGHT_DEG;
static float ws_start_angle=-1.0f, ws_end_angle=-1.0f;
static float ws_dist_start=-1.0f, ws_dist_end=-1.0f;
static bool  ws_seeing=false;
static absolute_time_t ws_next_move;

// Wait-clear
static uint8_t ws_clear_ok_count=0;
static absolute_time_t ws_next_clear_check;

// Telemetry
static char telemetry_payload[TELEMETRY_PAYLOAD_SIZE];

// ================== Small helpers ==================
static inline int clamp_i(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
static inline float clamp_f(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline bool time_due(absolute_time_t t){ return absolute_time_diff_us(get_absolute_time(), t) <= 0; }
static inline int smooth_speed_transition(int current,int target){
    int d=target-current;
    if(d> MAX_SPEED_CHANGE) return current+MAX_SPEED_CHANGE;
    if(d<-MAX_SPEED_CHANGE) return current-MAX_SPEED_CHANGE;
    return target;
}

// ----- IMU assists -----
static void update_heading_lock(float ir_err,float heading){
    if(fabsf(ir_err)<HEADING_LOCK_THRESH && cur_left_speed>MIN_SPEED_FOR_IMU){
        if(!heading_locked){ target_heading=heading; heading_locked=true; heading_lock_timer=0.0f; }
        heading_lock_timer=0.0f;
    }else{
        heading_lock_timer+=0.02f;
        if(heading_lock_timer>HEADING_LOCK_TIMEOUT_S) heading_locked=false;
    }
}
static float get_heading_correction(float heading){
    if(!heading_locked || cur_left_speed<MIN_SPEED_FOR_IMU) return 0.0f;
    float e=target_heading-heading; while(e>180)e-=360; while(e<-180)e+=360; return e*IMU_HEADING_GAIN;
}
static float get_stability_correction(float lat_g){
    if(cur_left_speed<MIN_SPEED_FOR_IMU) return 0.0f;
    return (fabsf(lat_g)>WOBBLE_THRESHOLD)? lat_g*IMU_STABILITY_GAIN*20.0f : 0.0f;
}
static float get_accel_correction(float lat_g){
    if(cur_left_speed<MIN_SPEED_FOR_IMU){ prev_lateral_accel=lat_g; return 0.0f; }
    float d = lat_g - prev_lateral_accel; prev_lateral_accel=lat_g; return d*IMU_ACCEL_GAIN*30.0f;
}

static float calc_pid_with_imu(float line_norm, float *p_out,float *i_out,float *d_out){
    const imu_data_t *imu = imu_get_data();
    bool imu_ok = imu_is_available();

    float ir_err = LINE_TARGET - line_norm;
    float P = KP_GAIN*ir_err;

    if(fabsf(ir_err)>DEAD_BAND){
        if(fabsf(ir_err)<0.4f){ pid_integral += ir_err; pid_integral = clamp_f(pid_integral,-INTEGRAL_MAX,INTEGRAL_MAX); }
    }else{
        pid_integral *= 0.95f;
    }
    float I = KI_GAIN*pid_integral;
    float D = KD_GAIN*(ir_err - pid_last_error);
    pid_last_error = ir_err;

    *p_out=P; *i_out=I; *d_out=D;

    float imu_corr=0.0f;
    if(imu_ok){
        float head=get_heading_correction(imu->heading_deg);
        float stab=get_stability_correction(imu->accel_y_g);
        float acc =get_accel_correction(imu->accel_y_g);
        update_heading_lock(ir_err, imu->heading_deg);
        imu_corr = head + stab + acc;
    }

    float out = clamp_f(P+I+D + imu_corr, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
    last_p_term=P; last_i_term=I; last_d_term=D; last_steer=out;
    return out;
}

static float width_from_edges_cm(float a1,float d1,float a2,float d2){
    if(d1<=0||d2<=0) return 0.0f;
    float r1=a1*(float)M_PI/180.0f, r2=a2*(float)M_PI/180.0f, span=fabsf(r2-r1);
    float w2 = d1*d1 + d2*d2 - 2.0f*d1*d2*cosf(span);
    return (w2>0)? sqrtf(w2) : 0.0f;
}

// ================== LINE FOLLOWING (always-sweeping radar) ==================
static void handle_line_following(uint32_t *loop_counter, absolute_time_t *next_pub)
{
    // 1) PID line follow (runs EVERY frame)
    line_detection_t line = ir_read_line();
    float p,i,d; float steer = calc_pid_with_imu(line.normalized_value, &p,&i,&d);

    float abs_err=fabsf(LINE_TARGET - line.normalized_value);
    float factor=clamp_f(abs_err / MAX_ERROR_FOR_SPEED_ADJUST, 0.0f,1.0f);
    int raw_target=(int)(STRAIGHT_SPEED - (STRAIGHT_SPEED - CORNER_SPEED)*factor);
    smoothed_target_speed = smoothed_target_speed*(1.0f - SPEED_SMOOTHING_FACTOR) + raw_target*SPEED_SMOOTHING_FACTOR;

    int tgtL=clamp_i((int)(smoothed_target_speed + steer), -MOTOR_MAX_ABS, MOTOR_MAX_ABS);
    int tgtR=clamp_i((int)(smoothed_target_speed - steer), -MOTOR_MAX_ABS, MOTOR_MAX_ABS);
    cur_left_speed  = smooth_speed_transition(cur_left_speed,  tgtL);
    cur_right_speed = smooth_speed_transition(cur_right_speed, tgtR);
    motor_set_differential(cur_left_speed, cur_right_speed);

    // 2) NON-BLOCKING radar sweep — ALWAYS attempt to sweep
    static bool waiting_echo=false;
    static float pending_next_angle=-1.0f;

    if (time_due(next_radar_tick)) {
        next_radar_tick = make_timeout_time_ms(RADAR_POLL_MS);

        // a) If a ping is in flight, finish its tiny echo window
        if (waiting_echo) {
            if (!ultrasonic_update()) {
                // echo still in flight (short); PID already ran this frame
            } else {
                float cm=-1.0f; us_status_t rc=ultrasonic_get_last_reading_cm(&cm);
                last_scan_distance = (rc==US_OK)? cm : (rc==US_INVALID_TOO_CLOSE? 0.0f : -1.0f);

                if (last_scan_distance > 0 && last_scan_distance < OBSTACLE_THRESHOLD_CM) {
                    robot_log("\nOBSTACLE @ %.1f cm → STOP & WIDTH SCAN\n\n", last_scan_distance);
                    motor_stop();
                    cur_left_speed = cur_right_speed = 0;
                    encoder_reset_glitch_filter();
                    g_master_state = STATE_WIDTH_MEASURE;
                    g_ws_state     = WS_PREPARE;
                    robot_mqtt_publish_qos(MQTT_TOPIC_STATE, "{\"status\":\"width_scan\"}", 1, 1);
                    waiting_echo=false;
                    return;
                }

                // move servo to next angle immediately so it can settle while PID keeps running
                if (pending_next_angle >= 0.0f) {
                    servo_set_angle_instant(pending_next_angle);
                    radar_angle = pending_next_angle;
                    ultrasonic_mark_servo_moved(time_us_32());
                    pending_next_angle = -1.0f;
                }
                waiting_echo=false;
            }
        }
        // b) If idle, start a ping whenever servo is settled & spacing OK (NO GATE)
        else {
            uint32_t now = time_us_32();
            if (ultrasonic_servo_settled(now) && ultrasonic_ping_spacing_ok(now)) {
                ultrasonic_start_read();
                waiting_echo = true;

                // queue next angle (commanded right after this ping completes)
                float next = radar_angle + radar_dir*RADAR_SWEEP_STEP_DEG;
                if (next >= ROBOT_LEFT_DEG)  { next=ROBOT_LEFT_DEG;  radar_dir=-1; }
                if (next <= ROBOT_RIGHT_DEG) { next=ROBOT_RIGHT_DEG; radar_dir=+1; }
                pending_next_angle = next;
            }
        }
    }

    // 3) Telemetry (periodic; non-blocking)
    if (time_due(*next_pub)) {
        *next_pub = make_timeout_time_ms(MQTT_PUB_PERIOD_MS);
        if (mqtt_is_connected()) {
            float l_rpm=encoder_get_left_rpm(), r_rpm=encoder_get_right_rpm();
            float l_cs =encoder_get_left_speed_cm_s(), r_cs=encoder_get_right_speed_cm_s();
            float avg_cs=(l_cs+r_cs)*0.5f, avg_kmh=avg_cs*0.036f;
            const imu_data_t *imu=imu_get_data();
            float ir_err=LINE_TARGET - line.normalized_value;

            snprintf(telemetry_payload,sizeof(telemetry_payload),
                "{"
                "\"avg_cm_s\":%.2f,\"avg_km_h\":%.3f,"
                "\"left_rpm\":%.2f,\"right_rpm\":%.2f,"
                "\"left_dist_m\":%.2f,\"right_dist_m\":%.2f,"
                "\"line_raw\":%u,\"line_norm\":%.3f,\"on_line\":%d,"
                "\"pid_error\":%.3f,\"pid_p\":%.2f,\"pid_i\":%.2f,\"pid_d\":%.2f,\"pid_output\":%.2f,"
                "\"heading\":%.1f,"
                "\"left_speed\":%d,\"right_speed\":%d,\"target_speed\":%d,"
                "\"distance_cm\":%.2f,\"scan_angle\":%.1f"
                "}",
                avg_cs, avg_kmh,
                l_rpm, r_rpm,
                encoder_get_left_distance_m(), encoder_get_right_distance_m(),
                line.raw_value, line.normalized_value, line.is_on_line,
                ir_err, last_p_term,last_i_term,last_d_term,last_steer,
                imu->heading_deg,
                cur_left_speed,cur_right_speed,(int)smoothed_target_speed,
                last_scan_distance>0?last_scan_distance:0.0f,
                radar_angle
            );
            robot_mqtt_publish(MQTT_TOPIC_TELEM, telemetry_payload);
        }
    }

    // 4) Occasional debug
    if ((*loop_counter)%DEBUG_PRINT_DIVISOR==0) {
        const imu_data_t *imu=imu_get_data();
        float ir_err=LINE_TARGET - line.normalized_value;
        printf("[#%06lu] IR=%u err=%+.3f | PID=%+.1f (P=%+.1f I=%+.1f D=%+.1f) "
               "| H=%.1f° | L=%d R=%d | @%.0f°=%.1fcm\n",
            *loop_counter, line.raw_value, ir_err,
            last_steer,last_p_term,last_i_term,last_d_term,
            imu->heading_deg, cur_left_speed,cur_right_speed,
            radar_angle, last_scan_distance);
    }
}

// ================== WIDTH MEASURE (stopped) ==================
static void handle_width_measure(absolute_time_t *next_pub)
{
    switch (g_ws_state) {
        case WS_PREPARE: {
            robot_log("--- Width Scan (Right → Left) ---\n");
            encoder_reset_glitch_filter();

            ws_angle=ROBOT_RIGHT_DEG; ws_start_angle=ws_end_angle=-1.0f;
            ws_dist_start=ws_dist_end=-1.0f; ws_seeing=false; last_scan_distance=-1.0f;

            servo_set_angle_instant(ws_angle);
            ultrasonic_mark_servo_moved(time_us_32());
            ws_next_move = make_timeout_time_ms(300);

            g_ws_state = WS_SCANNING;
        } break;

        case WS_SCANNING: {
            if (!time_due(ws_next_move)) break;

            // Settle + spacing (OK to wait; motors stopped)
            while (!(ultrasonic_servo_settled(time_us_32()) && ultrasonic_ping_spacing_ok(time_us_32()))) {
                cyw43_arch_poll(); tight_loop_contents();
            }

            ultrasonic_start_read();
            while (!ultrasonic_update()) tight_loop_contents();

            float cm=-1.0f; us_status_t rc=ultrasonic_get_last_reading_cm(&cm);
            last_scan_distance = (rc==US_OK)? cm : (rc==US_INVALID_TOO_CLOSE? 0.0f : -1.0f);

            float ang = ws_angle;
            bool is_obs = (last_scan_distance >= 0 && last_scan_distance < OBSTACLE_THRESHOLD_CM);
            if (is_obs) {
                if (!ws_seeing) { ws_start_angle=ang; ws_dist_start=last_scan_distance; ws_seeing=true; }
                ws_end_angle=ang; ws_dist_end=last_scan_distance;
            } else {
                ws_seeing=false;
            }

            // Telemetry snapshot
            if (time_due(*next_pub)) {
                *next_pub = make_timeout_time_ms(MQTT_PUB_PERIOD_MS);
                if (mqtt_is_connected()) {
                    float width_now=0.0f;
                    if (ws_start_angle>=0 && ws_end_angle>=0) {
                        width_now = width_from_edges_cm(ws_start_angle, ws_dist_start, ws_end_angle, ws_dist_end);
                    }
                    snprintf(telemetry_payload,sizeof(telemetry_payload),
                        "{\"distance_cm\":%.2f,\"width_cm\":%.2f,\"scan_angle\":%.1f}",
                        last_scan_distance>0?last_scan_distance:0.0f, width_now, ws_angle);
                    robot_mqtt_publish(MQTT_TOPIC_TELEM, telemetry_payload);
                }
            }

            // step & command next immediately (overlap settle with other work)
            ws_angle += WIDTH_SCAN_STEP_DEG;
            if (ws_angle > ROBOT_LEFT_DEG) {
                g_ws_state = WS_PROCESS;
            } else {
                servo_set_angle_instant(ws_angle);
                ultrasonic_mark_servo_moved(time_us_32());
                ws_next_move = make_timeout_time_ms(SCAN_SETTLE_MS);
            }
        } break;

        case WS_PROCESS: {
            robot_log("--- Width Scan Complete ---\n");
            float width_cm=0.0f;
            if (ws_start_angle>=0 && ws_end_angle>=0) {
                width_cm = width_from_edges_cm(ws_start_angle, ws_dist_start, ws_end_angle, ws_dist_end);
            } else {
                robot_log("No consistent obstacle during width scan.\n");
            }

            if (mqtt_is_connected()) {
                snprintf(telemetry_payload,sizeof(telemetry_payload),
                    "{\"width_scan_done\":true,\"width_cm\":%.2f,\"start_deg\":%.1f,\"end_deg\":%.1f}",
                    width_cm,
                    ws_start_angle>=0?ws_start_angle:0.0f,
                    ws_end_angle  >=0?ws_end_angle  :0.0f
                );
                robot_mqtt_publish(MQTT_TOPIC_TELEM, telemetry_payload);
            }

            // center + wait-clear
            servo_set_angle_instant(ROBOT_CENTER_DEG);
            ultrasonic_mark_servo_moved(time_us_32());
            ws_clear_ok_count=0;
            ws_next_clear_check = make_timeout_time_ms(CLEAR_RECHECK_MS);
            robot_log("Waiting for obstacle to clear (≥ %.0f cm)...\n",
                      OBSTACLE_THRESHOLD_CM + CLEAR_HYSTERESIS_CM);
            g_ws_state = WS_WAIT_CLEAR;
        } break;

        case WS_WAIT_CLEAR: {
            motor_stop();

            if (!time_due(ws_next_clear_check)) break;

            while (!(ultrasonic_servo_settled(time_us_32()) && ultrasonic_ping_spacing_ok(time_us_32()))) {
                cyw43_arch_poll(); tight_loop_contents();
            }

            ultrasonic_start_read();
            while (!ultrasonic_update()) tight_loop_contents();

            float cm=-1.0f; us_status_t rc=ultrasonic_get_last_reading_cm(&cm);
            last_scan_distance = (rc==US_OK)? cm : (rc==US_INVALID_TOO_CLOSE? 0.0f : -1.0f);

            float clear_thresh = OBSTACLE_THRESHOLD_CM + CLEAR_HYSTERESIS_CM; // 28 cm
            bool is_clear = (last_scan_distance >= clear_thresh);
            ws_clear_ok_count = is_clear ? (uint8_t)(ws_clear_ok_count+1) : 0;

            if (mqtt_is_connected()) {
                snprintf(telemetry_payload,sizeof(telemetry_payload),
                    "{\"wait_clear\":true,\"distance_cm\":%.2f,\"ok_count\":%u}",
                    last_scan_distance>0?last_scan_distance:0.0f, ws_clear_ok_count);
                robot_mqtt_publish(MQTT_TOPIC_TELEM, telemetry_payload);
            }

            if (ws_clear_ok_count >= CLEAR_CONSECUTIVE_COUNT) {
                robot_log("Path clear — resuming follow.\n");
                g_master_state = STATE_LINE_FOLLOWING;
                robot_mqtt_publish_qos(MQTT_TOPIC_STATE, "{\"status\":\"following\"}", 1, 1);

                // kick background sweep immediately from current radar_angle
                servo_set_angle_instant(radar_angle);
                ultrasonic_mark_servo_moved(time_us_32());
                break;
            }

            ws_next_clear_check = make_timeout_time_ms(CLEAR_RECHECK_MS);
        } break;
    }
}

// ================== PUBLIC API ==================
void robot_controller_init(void)
{
    g_master_state = STATE_LINE_FOLLOWING;
    g_ws_state     = WS_PREPARE;

    // center radar
    servo_set_angle_instant(ROBOT_CENTER_DEG);
    radar_angle = ROBOT_CENTER_DEG; radar_dir = +1;

    next_radar_tick = get_absolute_time();
    radar_waiting_echo=false; radar_pending_next_angle=-1.0f;

    // PID/IMU init
    pid_integral=0.0f; pid_last_error=0.0f;
    cur_left_speed=cur_right_speed=0; smoothed_target_speed=STRAIGHT_SPEED;
    heading_locked=false; target_heading=0.0f; prev_lateral_accel=0.0f;

    // Initialize and start continuous barcode scanning
    barcode_direction_init();
    barcode_direction_start_continuous();
    robot_log("✓ Continuous barcode scanning enabled on GPIO28\n");

    robot_log("\n=== Controller Ready (Always-sweeping ultrasonic) — Thr=%.0f cm ===\n",
              OBSTACLE_THRESHOLD_CM);
    robot_mqtt_publish_qos(MQTT_TOPIC_STATE, "{\"status\":\"following\"}", 1, 1);

    // prime settle gate so first ping after centering is clean
    ultrasonic_mark_servo_moved(time_us_32());
}

void robot_controller_run_loop(void)
{
    absolute_time_t next_pub = get_absolute_time();
    uint32_t loop_counter=0;

    uint32_t t_pid=time_us_32(), t_imu=t_pid, t_net=t_pid, t_mqtt=t_pid;
    uint32_t t_enc=t_pid, t_barcode=t_pid;

    while(true){
        loop_counter++;

        // keep Wi-Fi/MQTT & IMU alive (short & frequent, outside echo windows)
        if(time_us_32()-t_net >= DT_NET_US){ mqtt_maintain_connection(); cyw43_arch_poll(); t_net+=DT_NET_US; }
        if(time_us_32()-t_imu >= DT_IMU_US){ imu_read(); t_imu+=DT_IMU_US; }
        if (time_us_32()-t_enc >= DT_ENC_US)  { encoder_update_kinematics();                    t_enc  += DT_ENC_US; }

        // Update barcode scanner (non-blocking)
        if(time_us_32()-t_barcode >= DT_BARCODE_US){
            barcode_result_t* bc_result = barcode_direction_update();
            if(bc_result != NULL && bc_result->status == BC_STATUS_OK){
                // Barcode detected and decoded!
                const char* dir_str = barcode_direction_string(bc_result->move_dir);
                const char* scan_str = barcode_scan_direction_string(bc_result->scan_dir);

                robot_log("\n🔍 BARCODE DETECTED: \"%s\" | %s | Scan: %s\n",
                         bc_result->data, dir_str, scan_str);

                // Publish to MQTT
                if(mqtt_is_connected()){
                    char bc_payload[256];
                    snprintf(bc_payload, sizeof(bc_payload),
                        "{\"barcode\":\"%s\",\"direction\":\"%s\",\"scan_dir\":\"%s\",\"move_dir\":%d,\"bar_count\":%d}",
                        bc_result->data,
                        dir_str,
                        scan_str,
                        (int)bc_result->move_dir,
                        bc_result->bar_count
                    );
                    robot_mqtt_publish_qos("robocar/barcode", bc_payload, 1, 1);
                }
            }
            t_barcode += DT_BARCODE_US;
        }

        switch(g_master_state){
            case STATE_LINE_FOLLOWING:  handle_line_following(&loop_counter,&next_pub); break;
            case STATE_WIDTH_MEASURE:   handle_width_measure(&next_pub);                break;
        }

        if(time_us_32()-t_mqtt >= DT_MQTT_US){ t_mqtt += DT_MQTT_US; }
        if(time_us_32()-t_pid  >= DT_PID_US ){ t_pid  += DT_PID_US;  }

        tight_loop_contents();
    }
}
