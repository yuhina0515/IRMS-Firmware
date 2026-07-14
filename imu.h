#pragma once
// ─────────────────────────────────────────────────────────────
// MPU6050 驅動 + 互補濾波(header-only,供 IRMS_Sensor.ino 使用)
// 職責:I2C 讀寫、量程設定、陀螺儀零點校準、Pitch/Roll 姿態融合。
// 不含任何 BLE / 判定邏輯(依 ROADMAP D1,判定屬 App 端)。
// ─────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"

/** 單次取樣:加速度計推得的角度 + 已扣零點偏移的角速度 */
struct ImuSample {
  float accPitch;       // 矢狀面角(deg,atan2(ay, az))
  float accRoll;        // 冠狀面角(deg,atan2(ax, az))
  float gyroPitchRate;  // 矢狀面角速度(deg/s)
  float gyroRollRate;   // 冠狀面角速度(deg/s)
};

class Mpu6050 {
public:
  explicit Mpu6050(uint8_t addr) : addr_(addr) {}

  /** 喚醒 + 設定量程(gyro ±500dps / accel ±4g)。每步檢查 I2C 回應。 */
  bool begin() {
    Wire.beginTransmission(addr_);
    if (Wire.endTransmission(true) != 0) return false;
    if (!writeReg(0x6B, 0x00)) return false;  // PWR_MGMT_1:喚醒
    if (!writeReg(0x1B, 0x08)) return false;  // GYRO_CONFIG:±500 dps
    if (!writeReg(0x1C, 0x08)) return false;  // ACCEL_CONFIG:±4 g
    ready_ = true;
    return true;
  }

  bool isReady() const { return ready_; }

  /**
   * 讀取一筆樣本(14-byte burst)。失敗回 false 且不動 out。
   * 角速度已扣除 calibrate() 求得的零點偏移。
   */
  bool read(ImuSample &out) {
    Wire.beginTransmission(addr_);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr_, 14, true) != 14) return false;

    const int16_t ax = (Wire.read() << 8) | Wire.read();
    const int16_t ay = (Wire.read() << 8) | Wire.read();
    const int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // 溫度,略過
    const int16_t gx = (Wire.read() << 8) | Wire.read();
    const int16_t gy = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // gz 不使用

    out.accPitch      = atan2f((float)ay, (float)az) * 180.0f / PI;
    out.accRoll       = atan2f((float)ax, (float)az) * 180.0f / PI;
    // 軸向對應(2026-06-11 審計修正):Pitch 繞 Y 軸→gy、Roll 繞 X 軸→gx
    out.gyroPitchRate = (float)gy / GYRO_LSB_500 - offPitch_;
    out.gyroRollRate  = (float)gx / GYRO_LSB_500 - offRoll_;
    return true;
  }

  /**
   * 靜態零點校準:取樣平均求陀螺儀零漂。
   * 以 float 直接累加,避免整數截斷精度損失。裝置未就緒時直接跳過。
   */
  void calibrate(int samples) {
    if (!ready_) return;
    float sumPitch = 0.0f, sumRoll = 0.0f;
    int valid = 0;
    offPitch_ = offRoll_ = 0.0f;  // 校準期間讀原始值
    for (int i = 0; i < samples; i++) {
      ImuSample s;
      if (read(s)) {
        sumPitch += s.gyroPitchRate;
        sumRoll  += s.gyroRollRate;
        valid++;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (valid > 0) {
      offPitch_ = sumPitch / valid;
      offRoll_  = sumRoll / valid;
    }
  }

private:
  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission(true) == 0;
  }

  uint8_t addr_;
  bool ready_ = false;
  float offPitch_ = 0.0f;
  float offRoll_ = 0.0f;
};

/** 單一肢段的 Pitch/Roll 互補濾波器 */
class ComplementaryFilter {
public:
  /** 以加速度計靜態角重設濾波狀態(開機與 I2C 復原後呼叫,防跳變) */
  void reset(const ImuSample &s) {
    pitch_ = s.accPitch;
    roll_  = s.accRoll;
  }

  /** 融合一筆樣本。dt 以 DT_CLAMP_S 夾限,任何停頓後都不會產生積分尖峰。 */
  void update(const ImuSample &s, float dt) {
    if (dt < 0.0f) dt = 0.0f;
    if (dt > DT_CLAMP_S) dt = DT_CLAMP_S;
    pitch_ = FILTER_ALPHA * (pitch_ + s.gyroPitchRate * dt) + (1.0f - FILTER_ALPHA) * s.accPitch;
    roll_  = FILTER_ALPHA * (roll_  + s.gyroRollRate  * dt) + (1.0f - FILTER_ALPHA) * s.accRoll;
  }

  float pitch() const { return pitch_; }
  float roll() const { return roll_; }

private:
  float pitch_ = 0.0f;
  float roll_ = 0.0f;
};
