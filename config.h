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

// 6 軸封包滿刻度長度 "T:-180.0,S:-180.0,K:360.0,TR:-180.0,SR:-180.0,KR:360.0" = 54 bytes。
// ATT notify 承載 = MTU − 3,故 MTU 至少要 57 才不會把 Roll 切掉。setMTU() 只是「請求」,
// 真正生效的值由對端協商決定,所以韌體在 onMtuChanged 印出實際結果而不是假設它成功。
constexpr uint16_t PACKET_MAX_BYTES = 54;
constexpr uint16_t MTU_MIN_REQUIRED = PACKET_MAX_BYTES + 3;

// ── 序列遙測(issue #2:桌上 ±180° 旋轉記錄)──
// BLE notify 只在 App 連上時才送,所以「拿兩塊板子在桌上轉、把資料記下來」這件事
// 原本做不到——必須先有 App 連線,而那正是要用這份記錄去驗證的東西。開啟後
// Task_Comm 會把同一個封包字串同時輸出到 Serial,前面加上 millis() 時間戳:
//
//   <millis>	 T:12.5,S:-45.2,K:57.7,TR:1.2,SR:-0.8,KR:2.0
//
// 封包本體與 BLE 完全一致,擷取下來的記錄因此可以直接餵進 App 的 parseAnglePacket
// 當測試 fixture;時間戳是分開的一欄,用 tab 切開即可。診斷訊息一律以 '[' 開頭,
// grep '^[0-9]' 就能濾出純資料列。
//
// 預設開啟:UART0 在 115200 baud 下即使沒有人讀也會照常排空(classic ESP32 的
// Serial 是真 UART,不是 USB CDC,不會因為沒有讀取端而阻塞),25Hz × 約 65 bytes
// ≈ 16 kbps,佔用可忽略。
constexpr bool SERIAL_TELEMETRY = true;

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
