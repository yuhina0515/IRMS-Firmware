#pragma once
// ─────────────────────────────────────────────────────────────
// IRMS 韌體組態(單一來源)
// 腳位 / I2C 位址 / BLE 協定 / 濾波與時序參數全部集中於此。
// ⚠ BLE 區塊為與 App (shared/protocol.ts) 的協定契約,不可單方變更。
// ─────────────────────────────────────────────────────────────
#include <stdint.h>

// ── 腳位 ──
constexpr int PIN_SDA        = 21;  // I2C 資料
constexpr int PIN_SCL        = 22;  // I2C 時脈
constexpr int PIN_EXT_LED    = 25;  // 目標區間回饋 LED(App CMD 直控)
constexpr int PIN_BUZZER     = 26;  // 有源蜂鳴器(App CMD 直控)
constexpr int PIN_STATUS_LED = 2;   // 內建狀態指示燈

// ── IMU ──
constexpr uint8_t ADDR_THIGH = 0x68;  // 大腿 MPU6050(AD0 → GND)
constexpr uint8_t ADDR_SHIN  = 0x69;  // 小腿 MPU6050(AD0 → 3.3V)

// ── BLE 協定(與 App 契約,勿改)──
#define IRMS_DEVICE_NAME      "IRMS_Device"
#define IRMS_SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define IRMS_CHAR_ANGLE_TX    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define IRMS_CHAR_PROFILE_RX  "beb5483f-36e1-4688-b7f5-ea07361b26a8"
constexpr uint16_t BLE_MTU = 128;  // 6 軸封包最長約 62 bytes,預設 23 會截斷

// ── 時序 ──
constexpr uint32_t SENSOR_PERIOD_MS = 20;    // 感測迴圈 50Hz
constexpr uint32_t COMM_PERIOD_MS   = 40;    // BLE 推播 25Hz
constexpr uint32_t I2C_TIMEOUT_MS   = 1000;  // 匯流排鎖死保護
constexpr uint32_t RECOVER_WAIT_MS  = 1000;  // I2C 復原後的穩定等待

// ── 濾波與校準 ──
constexpr float FILTER_ALPHA   = 0.85f;  // 互補濾波:陀螺儀積分權重
constexpr float GYRO_LSB_500   = 65.5f;  // ±500 dps 靈敏度 (LSB/dps)
constexpr int   CALIB_SAMPLES  = 200;    // 開機零點校準取樣數
constexpr float DT_CLAMP_S     = 0.2f;   // dt 上限:任何停頓後防積分尖峰
constexpr int   I2C_FAIL_LIMIT = 5;      // 連續讀取失敗即進入錯誤/復原流程
