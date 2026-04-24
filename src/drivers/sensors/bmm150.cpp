#include "bmm150.h"
#include <Wire.h>

namespace atabey {
    namespace drivers {

        // Datasheet: BMM150 — Bosch geomagnetic sensor, I2C addr 0x10 (SDO low)
        static const uint8_t BMM_ADDR        = 0x10;

        static const uint8_t REG_CHIP_ID     = 0x40;
        static const uint8_t REG_DATA_X_LSB  = 0x42;
        static const uint8_t REG_POWER_CTRL  = 0x4B;
        static const uint8_t REG_OP_MODE     = 0x4C;
        static const uint8_t REG_INT_EN      = 0x4D;
        static const uint8_t REG_AXES_EN     = 0x4E;
        static const uint8_t REG_REP_XY      = 0x51;
        static const uint8_t REG_REP_Z       = 0x52;

        // Trim NVM addresses (per Bosch reference driver)
        static const uint8_t TRIM_X1         = 0x5D;
        static const uint8_t TRIM_Y1         = 0x5E;
        static const uint8_t TRIM_Z4_LSB     = 0x62;
        static const uint8_t TRIM_Z2_LSB     = 0x68;
        static const uint8_t TRIM_XYZ1_LSB   = 0x6A;

        static const uint8_t CHIP_ID_VAL     = 0x32;

        // Overflow sentinels (per Bosch)
        static const int16_t BMM150_OVERFLOW_XY = -4096;
        static const int16_t BMM150_OVERFLOW_Z  = -16384;
        static const float   BMM150_OVERFLOW_OUTPUT = 0.0f;

        BMM150::BMM150()
            : mx(0), my(0), mz(0), healthy_(false),
              dig_x1(0), dig_y1(0), dig_x2(0), dig_y2(0),
              dig_z1(0), dig_z2(0), dig_z3(0), dig_z4(0),
              dig_xy1(0), dig_xy2(0), dig_xyz1(0) {}

        bool BMM150::writeReg(uint8_t reg, uint8_t data) {
            Wire.beginTransmission(BMM_ADDR);
            Wire.write(reg);
            Wire.write(data);
            return (Wire.endTransmission() == 0);
        }

        bool BMM150::readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
            Wire.beginTransmission(BMM_ADDR);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0) return false;

            Wire.requestFrom((uint8_t)BMM_ADDR, (uint8_t)len, (uint8_t)1);
            if (Wire.available() < len) return false;

            for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
            return true;
        }

        bool BMM150::readReg(uint8_t reg, uint8_t& out) {
            uint8_t b;
            if (!readBytes(reg, &b, 1)) return false;
            out = b;
            return true;
        }

        bool BMM150::readTrim() {
            uint8_t b[2];

            if (!readBytes(TRIM_X1, b, 1)) return false;
            dig_x1 = (int8_t)b[0];
            if (!readBytes(TRIM_Y1, b, 1)) return false;
            dig_y1 = (int8_t)b[0];

            // dig_x2, dig_y2 at 0x64, 0x65
            if (!readBytes(0x64, b, 1)) return false;
            dig_x2 = (int8_t)b[0];
            if (!readBytes(0x65, b, 1)) return false;
            dig_y2 = (int8_t)b[0];

            // dig_z1 (u16) at 0x6A,0x6B   -- overlaps with XYZ1; re-check layout
            // Correct Bosch layout:
            //   dig_z2 (s16) 0x68,0x69
            //   dig_z1 (u16) 0x6A,0x6B
            //   dig_z3 (s16) 0x6E,0x6F
            //   dig_z4 (s16) 0x62,0x63
            //   dig_xy1 (u8) 0x71
            //   dig_xy2 (s8) 0x70
            //   dig_xyz1 (u16) 0x6D,0x6E? ... use reference layout below
            if (!readBytes(0x68, b, 2)) return false;
            dig_z2 = (int16_t)(((uint16_t)b[1] << 8) | b[0]);

            if (!readBytes(0x6A, b, 2)) return false;
            dig_z1 = (uint16_t)(((uint16_t)b[1] << 8) | b[0]);

            if (!readBytes(0x6E, b, 2)) return false;
            dig_z3 = (int16_t)(((uint16_t)b[1] << 8) | b[0]);

            if (!readBytes(0x62, b, 2)) return false;
            dig_z4 = (int16_t)(((uint16_t)b[1] << 8) | b[0]);

            if (!readBytes(0x70, b, 1)) return false;
            dig_xy2 = (int8_t)b[0];
            if (!readBytes(0x71, b, 1)) return false;
            dig_xy1 = b[0];

            // dig_xyz1 is u16 at 0x6D..0x6E in some variants; Bosch ref uses 0x6D/0x6E
            if (!readBytes(0x6D, b, 2)) return false;
            dig_xyz1 = (uint16_t)((((uint16_t)b[1] & 0x7F) << 8) | b[0]);

            return true;
        }

        bool BMM150::init() {
            Wire.begin();
            Wire.setClock(400000);

            // Suspend -> sleep: set power control bit
            writeReg(REG_POWER_CTRL, 0x01);
            delay(5);

            uint8_t id = 0;
            if (!readReg(REG_CHIP_ID, id) || id != CHIP_ID_VAL) {
                healthy_ = false;
                return false;
            }

            // Presets: high-accuracy (XY=47, Z=83 rep), 20 Hz ODR, normal mode
            writeReg(REG_REP_XY, 0x17); // 47 = (0x17 * 2) + 1
            writeReg(REG_REP_Z,  0x52); // 83 = 0x52 + 1
            // OP mode: normal (bits 2:1 = 00), ODR 20Hz (bits 5:3 = 100)
            writeReg(REG_OP_MODE, (0x04 << 3) | 0x00);
            delay(10);

            if (!readTrim()) {
                healthy_ = false;
                return false;
            }

            healthy_ = true;
            return true;
        }

        // Bosch-reference compensation, reduced to float math.
        float BMM150::compensateX(int16_t raw, uint16_t rhall) {
            if (raw == BMM150_OVERFLOW_XY || rhall == 0 || dig_xyz1 == 0)
                return BMM150_OVERFLOW_OUTPUT;

            float process_comp_x0 = (float)dig_xyz1 * 16384.0f / (float)rhall;
            float process_comp_x1 = process_comp_x0 - 16384.0f;
            float process_comp_x2 = ((float)dig_xy2) * (process_comp_x1 * process_comp_x1 / 268435456.0f);
            process_comp_x2 += process_comp_x1 * ((float)dig_xy1) / 16384.0f;
            float process_comp_x3 = ((float)dig_x2) + 160.0f;
            float process_comp_x4 = ((float)raw) * ((process_comp_x2 + 256.0f) * process_comp_x3);
            return (process_comp_x4 / 8192.0f + ((float)dig_x1) * 8.0f) / 16.0f;
        }

        float BMM150::compensateY(int16_t raw, uint16_t rhall) {
            if (raw == BMM150_OVERFLOW_XY || rhall == 0 || dig_xyz1 == 0)
                return BMM150_OVERFLOW_OUTPUT;

            float process_comp_y0 = (float)dig_xyz1 * 16384.0f / (float)rhall;
            float process_comp_y1 = process_comp_y0 - 16384.0f;
            float process_comp_y2 = ((float)dig_xy2) * (process_comp_y1 * process_comp_y1 / 268435456.0f);
            process_comp_y2 += process_comp_y1 * ((float)dig_xy1) / 16384.0f;
            float process_comp_y3 = ((float)dig_y2) + 160.0f;
            float process_comp_y4 = ((float)raw) * ((process_comp_y2 + 256.0f) * process_comp_y3);
            return (process_comp_y4 / 8192.0f + ((float)dig_y1) * 8.0f) / 16.0f;
        }

        float BMM150::compensateZ(int16_t raw, uint16_t rhall) {
            if (raw == BMM150_OVERFLOW_Z || rhall == 0 || dig_z2 == 0 || dig_z1 == 0)
                return BMM150_OVERFLOW_OUTPUT;

            float num   = ((float)raw - (float)dig_z4) * 131072.0f
                        - ((float)dig_z3) * ((float)rhall - (float)dig_xyz1);
            float denom = ((float)dig_z2 + ((float)dig_z1 * (float)rhall / 32768.0f)) * 4.0f;
            if (denom == 0.0f) return BMM150_OVERFLOW_OUTPUT;
            return (num / denom) / 16.0f;
        }

        bool BMM150::read() {
            uint8_t buf[8];
            if (!readBytes(REG_DATA_X_LSB, buf, 8)) {
                healthy_ = false;
                return false;
            }

            // Data-ready bit check (LSB bit 0 of RHALL LSB)
            if ((buf[6] & 0x01) == 0) {
                // data not yet ready — skip this cycle, keep last values
                return false;
            }

            // X/Y: 13-bit signed, shift right 3 on LSB (bits 7:3)
            int16_t rawX = (int16_t)(((int16_t)((int8_t)buf[1]) << 5) | (buf[0] >> 3));
            int16_t rawY = (int16_t)(((int16_t)((int8_t)buf[3]) << 5) | (buf[2] >> 3));
            // Z: 15-bit signed, shift right 1 on LSB (bits 7:1)
            int16_t rawZ = (int16_t)(((int16_t)((int8_t)buf[5]) << 7) | (buf[4] >> 1));
            // RHALL: 14-bit unsigned, bits 7:2 of LSB
            uint16_t rhall = (uint16_t)(((uint16_t)buf[7] << 6) | (buf[6] >> 2));

            mx = compensateX(rawX, rhall);
            my = compensateY(rawY, rhall);
            mz = compensateZ(rawZ, rhall);

            healthy_ = true;
            return true;
        }

    }
}
