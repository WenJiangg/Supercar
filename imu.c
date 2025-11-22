/**
 * ==============================================================================
 * IMU.C - Implementation for IMU Module (LIS3DH + LSM303)
 * ==============================================================================
 */

#include "imu.h"
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ===== I2C config =====
#define IMU_I2C_PORT     i2c0
#define IMU_SDA_PIN      0
#define IMU_SCL_PIN      1
#define IMU_I2C_BAUD     (400 * 1000)

// ===== LIS3DH (accelerometer) =====
#define LIS3DH_ADDR      0x19
#define LIS3DH_WHO_AM_I  0x0F   // expect 0x33
#define LIS3DH_CTRL1     0x20   // ODR, axis enable
#define LIS3DH_CTRL4     0x23   // scale, HR
#define LIS3DH_OUT_X_L   0x28   // auto-inc OK

// ===== LSM303DLHC (magnetometer) =====
#define DLHC_ADDR        0x1E
#define DLHC_CRA         0x00   // data rate
#define DLHC_CRB         0x01   // gain
#define DLHC_MR          0x02   // mode
#define DLHC_OUT_X_H     0x03   // X_H, X_L, Z_H, Z_L, Y_H, Y_L
#define DLHC_ID_A        0x0A   // 'H'
#define DLHC_ID_B        0x0B   // '4'
#define DLHC_ID_C        0x0C   // '3'

// ===== LSM303D (magnetometer in combo chip) =====
#define LSM303D_ADDR_1D  0x1D
#define LSM303D_ADDR_1E  0x1E
#define LSM303D_WHO_AM_I 0x0F   // expect 0x49
#define LSM303D_CTRL5    0x24   // mag ODR
#define LSM303D_CTRL7    0x26   // mag mode
#define LSM303D_OUT_X_LM 0x08   // X_L, X_H, Y_L, Y_H, Z_L, Z_H

typedef enum { MAG_NONE = 0, MAG_LSM303D, MAG_LSM303DLHC } mag_type_t;

// ===== Module state (private) =====
// This is now the *single* copy for the whole project
static bool       imu_available = false;
static mag_type_t mag_type      = MAG_NONE;
static uint8_t    mag_addr      = DLHC_ADDR;
static char       imu_model[48] = "UNINITIALIZED";
static imu_data_t imu_data; // The single data struct

// ===== I2C helpers (private) =====
static bool imu_i2c_write_u8(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(IMU_I2C_PORT, addr, buf, 2, false) == 2;
}
static bool imu_i2c_read_u8(uint8_t addr, uint8_t reg, uint8_t *val) {
    if (i2c_write_blocking(IMU_I2C_PORT, addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(IMU_I2C_PORT, addr, val, 1, false) == 1;
}
static bool imu_i2c_read(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len, bool autoinc) {
    uint8_t r = autoinc ? (reg | 0x80) : reg;
    if (i2c_write_blocking(IMU_I2C_PORT, addr, &r, 1, true) != 1) return false;
    return i2c_read_blocking(IMU_I2C_PORT, addr, dst, len, false) == (int)len;
}

// ===== Public API Functions (Implementations) =====
bool imu_is_available(void)   { return imu_available; }
const char* imu_get_model(void)   { return imu_model; }
const imu_data_t* imu_get_data(void)  { return &imu_data; }
float imu_get_heading(void)    { return imu_data.heading_deg; }
float imu_get_tilt(void)       { return imu_data.tilt_deg; }

bool imu_init(void) {
    i2c_init(IMU_I2C_PORT, IMU_I2C_BAUD);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    // LIS3DH check
    uint8_t who = 0;
    if (!imu_i2c_read_u8(LIS3DH_ADDR, LIS3DH_WHO_AM_I, &who) || who != 0x33) {
        printf("IMU: LIS3DH not found (WHO=0x%02X)\n", who);
        imu_available = false;
        snprintf(imu_model, sizeof(imu_model), "LIS3DH? + MAG?");
        return false;
    }
    // LIS3DH config: 100Hz, XYZ on; +/-2g, high-res
    imu_i2c_write_u8(LIS3DH_ADDR, LIS3DH_CTRL1, 0x57);
    imu_i2c_write_u8(LIS3DH_ADDR, LIS3DH_CTRL4, 0x08);

    // Try LSM303D (0x1D then 0x1E)
    uint8_t who_d = 0;
    if (imu_i2c_read_u8(LSM303D_ADDR_1D, LSM303D_WHO_AM_I, &who_d) && who_d == 0x49) {
        mag_type = MAG_LSM303D; mag_addr = LSM303D_ADDR_1D;
    } else if (imu_i2c_read_u8(LSM303D_ADDR_1E, LSM303D_WHO_AM_I, &who_d) && who_d == 0x49) {
        mag_type = MAG_LSM303D; mag_addr = LSM303D_ADDR_1E;
    } else {
        // Try LSM303DLHC IDs ('H','4','3')
        uint8_t ida=0, idb=0, idc=0;
        bool okA = imu_i2c_read_u8(DLHC_ADDR, DLHC_ID_A, &ida);
        bool okB = imu_i2c_read_u8(DLHC_ADDR, DLHC_ID_B, &idb);
        bool okC = imu_i2c_read_u8(DLHC_ADDR, DLHC_ID_C, &idc);
        if (okA && okB && okC && ida=='H' && idb=='4' && idc=='3') {
            mag_type = MAG_LSM303DLHC; mag_addr = DLHC_ADDR;
        } else {
            printf("IMU: No magnetometer found (LSM303D/DLHC)\n");
            imu_available = false;
            snprintf(imu_model, sizeof(imu_model), "LIS3DH (OK) + MAG?");
            return false;
        }
    }

    // Configure magnetometer
    if (mag_type == MAG_LSM303D) {
        // CTRL5: 0x64 => mag ODR=12.5Hz; CTRL7: 0x00 => continuous-conversion
        imu_i2c_write_u8(mag_addr, LSM303D_CTRL5, 0x64);
        imu_i2c_write_u8(mag_addr, LSM303D_CTRL7, 0x00);
        snprintf(imu_model, sizeof(imu_model), "LIS3DH + LSM303D (0x%02X)", mag_addr);
    } else { // DLHC
        // CRA=0x18 (75Hz), CRB=0x20 (gain), MR=0x00 (continuous)
        imu_i2c_write_u8(mag_addr, DLHC_CRA, 0x18);
        imu_i2c_write_u8(mag_addr, DLHC_CRB, 0x20);
        imu_i2c_write_u8(mag_addr, DLHC_MR,  0x00);
        snprintf(imu_model, sizeof(imu_model), "LIS3DH + LSM303DLHC");
    }

    imu_available = true;
    printf("IMU: initialized — %s | I2C0 SDA=GP%d SCL=GP%d\n",
           imu_model, IMU_SDA_PIN, IMU_SCL_PIN);
    return true;
}

void imu_read(void) {
    if (!imu_available) return;

    // Accelerometer (6 bytes, little-endian)
    uint8_t a[6];
    if (imu_i2c_read(LIS3DH_ADDR, LIS3DH_OUT_X_L, a, 6, true)) {
        int16_t ax = (int16_t)((a[1] << 8) | a[0]);
        int16_t ay = (int16_t)((a[3] << 8) | a[2]);
        int16_t az = (int16_t)((a[5] << 8) | a[4]);
        imu_data.accel_x = ax; imu_data.accel_y = ay; imu_data.accel_z = az;
        // rough scale: ~16384 LSB/g
        imu_data.accel_x_g = ax / 16384.0f;
        imu_data.accel_y_g = ay / 16384.0f;
        imu_data.accel_z_g = az / 16384.0f;
    }

    // Magnetometer
    uint8_t m[6];
    if (mag_type == MAG_LSM303D) {
        if (imu_i2c_read(mag_addr, LSM303D_OUT_X_LM, m, 6, true)) {
            int16_t mx = (int16_t)((m[1] << 8) | m[0]);
            int16_t my = (int16_t)((m[3] << 8) | m[2]);
            int16_t mz = (int16_t)((m[5] << 8) | m[4]);
            imu_data.mag_x = mx; imu_data.mag_y = my; imu_data.mag_z = mz;
            imu_data.mag_x_u = (float)mx; imu_data.mag_y_u = (float)my; imu_data.mag_z_u = (float)mz;
        }
    } else { // DLHC: X_H,X_L,Z_H,Z_L,Y_H,Y_L
        uint8_t reg = DLHC_OUT_X_H;
        if (i2c_write_blocking(IMU_I2C_PORT, mag_addr, &reg, 1, true) == 1 &&
            i2c_read_blocking (IMU_I2C_PORT, mag_addr, m, 6, false) == 6) {
            int16_t mx = (int16_t)((m[0] << 8) | m[1]);
            int16_t mz = (int16_t)((m[2] << 8) | m[3]);
            int16_t my = (int16_t)((m[4] << 8) | m[5]);
            imu_data.mag_x = mx; imu_data.mag_y = my; imu_data.mag_z = mz;
            imu_data.mag_x_u = (float)mx; imu_data.mag_y_u = (float)my; imu_data.mag_z_u = (float)mz;
        }
    }

    // Heading (no calibration)
    float hdg = atan2f(imu_data.mag_y_u, imu_data.mag_x_u) * 180.0f / (float)M_PI;
    if (hdg < 0) hdg += 360.0f;
    imu_data.heading_deg = hdg;

    // Simple tilt from accel (0° ~ flat)
    float ax = imu_data.accel_x_g, ay = imu_data.accel_y_g, az = imu_data.accel_z_g;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 1e-6f) { ax /= norm; ay /= norm; az /= norm; }
    imu_data.tilt_deg = acosf(fminf(fmaxf(az, -1.0f), 1.0f)) * 180.0f / (float)M_PI;
}