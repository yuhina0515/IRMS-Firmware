// ─────────────────────────────────────────────────────────────
// IRMS 穿戴式感測節點(v3 重寫)
// 架構依 ROADMAP 決策 D1(App-Driven):韌體只負責
//   感測(50Hz 雙 MPU6050)→ 濾波(互補)→ 傳輸(BLE 25Hz)→ 執行 App 的 CMD
// 達標/超限「判定」完全在 App 端 TriggerEngine,本檔不含判定邏輯。
//
// BLE 協定與 v2 完全相容:UUID、封包 T,S,K,TR,SR,KR / ERR:1、CMD 字串均不變。
//
// 任務配置:
//   Task_Sensor (Core1, P3) 取樣+濾波+I2C 自復原
//   Task_Comm   (Core0, P1) BLE 推播與重新廣播
//   Task_LED    (Core1, P1) 狀態指示燈 + 達標雙響(旗標驅動)
// ─────────────────────────────────────────────────────────────
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>
#include "config.h"
#include "imu.h"

// ── 系統狀態(僅供狀態燈與 ERR 推播;達標音另以旗標處理,不混入此處)──
enum SystemState { STATE_INIT, STATE_ERROR, STATE_ADVERTISING, STATE_CONNECTED };
volatile SystemState currentState = STATE_INIT;

// ── 共享角度資料 ──
struct LegAngles {
  float thigh, shin, knee;
  float thighRoll, shinRoll, kneeRoll;
};
static LegAngles latestAngles = {};
static SemaphoreHandle_t xAngleMutex;

// ── BLE ──
static BLEServer *pServer = nullptr;
static BLECharacteristic *pCharAngleTx = nullptr;
static BLECharacteristic *pCharProfileRx = nullptr;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;

// ── OTA(獨立 service,見 config.h)──
static BLECharacteristic *pCharOtaControl = nullptr;
static BLECharacteristic *pCharOtaData    = nullptr;
static BLECharacteristic *pCharOtaStatus  = nullptr;
static BLECharacteristic *pCharFwVersion  = nullptr;

enum OtaState { OTA_IDLE, OTA_RECEIVING, OTA_ERROR };
static volatile OtaState otaState = OTA_IDLE;
static size_t otaExpectedSize     = 0;
static volatile size_t otaReceivedSize = 0;
static size_t otaLastNotifiedSize = 0;

static void sendOtaStatus(const char *msg) {
  if (pCharOtaStatus == nullptr) return;
  pCharOtaStatus->setValue((uint8_t *)msg, strlen(msg));
  pCharOtaStatus->notify();
}

// 斷線或中止都要回到乾淨狀態,否則下一次 OTA:START 會被舊的殘留狀態擋住,
// 或更糟——continue 寫入一個其實已經沒有對應 begin() 的 Update 物件。
static void otaReset() {
  if (otaState == OTA_RECEIVING) Update.abort();
  otaState = OTA_IDLE;
  otaExpectedSize = 0;
  otaReceivedSize = 0;
  otaLastNotifiedSize = 0;
}

// ── 回饋輸出旗標(由 BLE onWrite 設定,由對應執行緒消費)──
volatile bool goalBeepRequest = false;  // 達標雙響請求(Task_LED 消費)
volatile bool alarmActive = false;      // 超限警報狀態(雙響結束後據此恢復蜂鳴器)

// ── 感測物件 ──
static Mpu6050 imuThigh(ADDR_THIGH);
static Mpu6050 imuShin(ADDR_SHIN);
static ComplementaryFilter filtThigh;
static ComplementaryFilter filtShin;

// ── 安全:關閉所有回饋輸出(斷線 / 開機時)──
static void feedbackAllOff() {
  digitalWrite(PIN_EXT_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  alarmActive = false;
  goalBeepRequest = false;
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    deviceConnected = true;
    if (currentState != STATE_ERROR) currentState = STATE_CONNECTED;
    Serial.println("[BLE] client connected");
  }
  void onDisconnect(BLEServer *) override {
    deviceConnected = false;
    if (currentState != STATE_ERROR) currentState = STATE_ADVERTISING;
    // 安全:App 已無法下指令,強制靜音熄燈,避免蜂鳴器卡在長鳴
    feedbackAllOff();
    // 安全:斷線發生在 OTA 傳輸中途時中止 Update——otadata 只有在 end() 成功時才會
    // 切換開機目標,所以這裡就算什麼都不做也不會變磚,但不 abort() 會讓 Update
    // 物件卡在半寫入狀態,擋住重新連線後的下一次 OTA:START。
    otaReset();
    Serial.println("[BLE] client disconnected — feedback outputs forced off");
  }
  // setMTU() 只是請求;真正生效的值由對端協商。App 端在 MTU 停在預設 23 時只會看到
  // Roll 恆為 0(封包被切在 20 bytes),那是最難從症狀反推回成因的一種故障,
  // 所以這裡把協商結果直接印出來,不留給人猜。
  void onMtuChanged(BLEServer *, esp_ble_gatts_cb_param_t *param) override {
    const uint16_t mtu = param->mtu.mtu;
    Serial.print("[BLE] MTU negotiated: ");
    Serial.print(mtu);
    if (mtu < MTU_MIN_REQUIRED) {
      Serial.print(" — TOO SMALL, need >= ");
      Serial.print(MTU_MIN_REQUIRED);
      Serial.println(" for the 6-axis packet; roll fields will be truncated");
    } else {
      Serial.println(" — OK for the 6-axis packet");
    }
  }
};

class ProfileCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    String data = ch->getValue();
    if (data.length() == 0) return;
    data.trim();  // 防禦:剝除 \r\n 空白,確保精確比對
    Serial.print("[CMD] ");
    Serial.println(data);

    if (!data.startsWith("CMD:")) return;  // Profile 參數依 D1 不再解析
    if (data == "CMD:GOAL") {
      goalBeepRequest = true;
    } else if (data == "CMD:LED_ON") {
      digitalWrite(PIN_EXT_LED, HIGH);
    } else if (data == "CMD:LED_OFF") {
      digitalWrite(PIN_EXT_LED, LOW);
    } else if (data == "CMD:ALARM_ON") {
      alarmActive = true;
      digitalWrite(PIN_BUZZER, HIGH);
    } else if (data == "CMD:ALARM_OFF") {
      alarmActive = false;
      digitalWrite(PIN_BUZZER, LOW);
    }
  }
};

// OTA 控制指令,獨立於上面的 CMD: 解析器之外(不同 characteristic、不同 service)——
// 就算 OTA 狀態機寫錯,也不可能誤吃或卡住既有的感測器指令路徑。
class OtaControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    String data = ch->getValue();
    data.trim();
    if (data.length() == 0) return;
    Serial.print("[OTA:CMD] ");
    Serial.println(data);

    if (data.startsWith("OTA:START:")) {
      // 格式:OTA:START:<totalBytes>:<md5hex32> — md5 由 App 端在傳送前算好,
      // Update.end() 會拿實際寫入內容重算一次比對,詳見 setMD5() 的用途。
      const int sepAt = data.indexOf(':', 10);
      if (sepAt < 0) { sendOtaStatus("OTA:ERROR:BAD_START"); return; }
      const size_t total = (size_t)data.substring(10, sepAt).toInt();
      String md5 = data.substring(sepAt + 1);
      md5.trim();

      if (otaState == OTA_RECEIVING) { sendOtaStatus("OTA:ERROR:ALREADY_RUNNING"); return; }
      if (total == 0 || md5.length() != 32) { sendOtaStatus("OTA:ERROR:BAD_START"); return; }

      if (!Update.begin(total)) {
        sendOtaStatus("OTA:ERROR:NO_SPACE");
        Update.printError(Serial);
        return;
      }
      Update.setMD5(md5.c_str());
      otaExpectedSize = total;
      otaReceivedSize = 0;
      otaLastNotifiedSize = 0;
      otaState = OTA_RECEIVING;
      sendOtaStatus("OTA:READY");
      Serial.print("[OTA] start, expecting ");
      Serial.print(total);
      Serial.println(" bytes");

    } else if (data == "OTA:END") {
      if (otaState != OTA_RECEIVING) { sendOtaStatus("OTA:ERROR:NOT_STARTED"); return; }
      if (otaReceivedSize != otaExpectedSize) {
        otaReset();
        sendOtaStatus("OTA:ERROR:SIZE_MISMATCH");
        return;
      }
      // end() 不傳 true——保留「必須恰好收滿宣告的 size」這道檢查,不允許提早結束。
      if (Update.end()) {
        sendOtaStatus("OTA:DONE");
        Serial.println("[OTA] verified OK, rebooting into new firmware");
        delay(300);  // 讓上面這則 notify 真的來得及送出去,不然 App 端永遠等不到 DONE
        ESP.restart();
      } else {
        char errBuf[80];
        snprintf(errBuf, sizeof(errBuf), "OTA:ERROR:%s", Update.errorString());
        sendOtaStatus(errBuf);
        Serial.print("[OTA] end() failed: ");
        Serial.println(Update.errorString());
        otaReset();
      }

    } else if (data == "OTA:ABORT") {
      otaReset();
      sendOtaStatus("OTA:ABORTED");
      Serial.println("[OTA] aborted by App");
    }
  }
};

class OtaDataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    if (otaState != OTA_RECEIVING) return;  // OTA:START 之前或結束/中止之後的殘留分塊——忽略

    // 用 getData()/getLength() 而非 getValue():後者回傳 Arduino String,韌體二進位
    // 分塊裡的 0x00 位元組沒有理由不出現,長度必須直接來自 BLE 層記的位元組數。
    const size_t chunkLen = ch->getLength();
    uint8_t *chunkData = ch->getData();
    if (chunkLen == 0 || chunkData == nullptr) return;

    const size_t written = Update.write(chunkData, chunkLen);
    if (written != chunkLen) {
      sendOtaStatus("OTA:ERROR:WRITE_FAIL");
      Serial.println("[OTA] write() short-write, aborting");
      otaReset();
      return;
    }

    otaReceivedSize += written;
    if (otaReceivedSize - otaLastNotifiedSize >= OTA_PROGRESS_STEP_BYTES ||
        otaReceivedSize == otaExpectedSize) {
      otaLastNotifiedSize = otaReceivedSize;
      char progBuf[32];
      snprintf(progBuf, sizeof(progBuf), "OTA:PROGRESS:%u", (unsigned)otaReceivedSize);
      sendOtaStatus(progBuf);
    }
  }
};

// ─────────────────────────────────────────────────────────────
// Task_Sensor:50Hz 取樣 → 互補濾波 → 發佈;I2C 失敗自動復原
// ─────────────────────────────────────────────────────────────
static void Task_Sensor(void *) {
  const bool ok1 = imuThigh.begin();
  const bool ok2 = imuShin.begin();
  if (!ok1 || !ok2) {
    currentState = STATE_ERROR;
    Serial.println("[IMU] CRITICAL: init failed at startup");
  }

  // 校準與初始化(裝置未就緒時 calibrate/read 自動略過)
  imuThigh.calibrate(CALIB_SAMPLES);
  imuShin.calibrate(CALIB_SAMPLES);
  ImuSample s1, s2;
  if (imuThigh.read(s1)) filtThigh.reset(s1);
  if (imuShin.read(s2)) filtShin.reset(s2);

  uint32_t lastMs = millis();
  int failCount = 0;

  for (;;) {
    const uint32_t nowMs = millis();
    const float dt = (nowMs - lastMs) / 1000.0f;  // 濾波器內部另有 DT_CLAMP_S 夾限
    lastMs = nowMs;

    const bool r1 = imuThigh.read(s1);
    const bool r2 = imuShin.read(s2);

    if (r1 && r2) {
      failCount = 0;
      filtThigh.update(s1, dt);
      filtShin.update(s2, dt);

      LegAngles a;
      a.thigh     = filtThigh.pitch();
      a.shin      = filtShin.pitch();
      a.knee      = fabsf(a.thigh - a.shin);
      a.thighRoll = filtThigh.roll();
      a.shinRoll  = filtShin.roll();
      a.kneeRoll  = fabsf(a.thighRoll - a.shinRoll);

      if (xSemaphoreTake(xAngleMutex, portMAX_DELAY) == pdTRUE) {
        latestAngles = a;
        xSemaphoreGive(xAngleMutex);
      }
      if (currentState == STATE_ERROR) {
        currentState = deviceConnected ? STATE_CONNECTED : STATE_ADVERTISING;
        Serial.println("[IMU] recovered");
      }
    } else if (++failCount > I2C_FAIL_LIMIT) {
      currentState = STATE_ERROR;
      // 自動復原:重啟匯流排 → 重新初始化 → 穩定等待 → 以靜態角重設濾波
      Wire.begin(PIN_SDA, PIN_SCL);
      Wire.setTimeOut(I2C_TIMEOUT_MS);
      const bool re1 = imuThigh.begin();
      const bool re2 = imuShin.begin();
      vTaskDelay(pdMS_TO_TICKS(RECOVER_WAIT_MS));
      if (re1 && re2 && imuThigh.read(s1) && imuShin.read(s2)) {
        filtThigh.reset(s1);
        filtShin.reset(s2);
        failCount = 0;
      }
      lastMs = millis();  // 復原耗時不得計入下一筆 dt
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// ─────────────────────────────────────────────────────────────
// Task_Comm:25Hz 推播角度封包(或 ERR:1);斷線後重新廣播
// ─────────────────────────────────────────────────────────────
static void Task_Comm(void *) {
  char buf[72];
  for (;;) {
    // 封包在「已連線」或「開了序列遙測」時才需要組。後者讓桌上旋轉記錄不必先有
    // App 連線就能擷取(issue #2)——原本 notify 與封包組裝綁在同一個 if 裡,
    // 導致唯一的資料出口是 BLE,而 BLE 正是那份記錄要拿來驗證的對象。
    if (deviceConnected || SERIAL_TELEMETRY) {
      int len;
      if (currentState == STATE_ERROR) {
        len = snprintf(buf, sizeof(buf), "ERR:1");
      } else {
        LegAngles a;
        if (xSemaphoreTake(xAngleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          a = latestAngles;
          xSemaphoreGive(xAngleMutex);
          len = snprintf(buf, sizeof(buf), "T:%.1f,S:%.1f,K:%.1f,TR:%.1f,SR:%.1f,KR:%.1f",
                         a.thigh, a.shin, a.knee, a.thighRoll, a.shinRoll, a.kneeRoll);
        } else {
          len = 0;  // 本輪取不到資料,跳過
        }
      }
      if (len > 0) {
        if (deviceConnected) {
          pCharAngleTx->setValue((uint8_t *)buf, len);
          pCharAngleTx->notify();
        }
        // 與 BLE 送出的完全同一個字串,前面補 millis() 時間戳(tab 分隔)。
        // 送出的是組裝後的完整封包,不是 notify 之後被 MTU 切過的結果——
        // 這正是要的:序列記錄代表感測器算出什麼,BLE 那端代表 App 收到什麼。
        if (SERIAL_TELEMETRY) {
          Serial.print(millis());
          Serial.print('	');
          Serial.println(buf);
        }
      }
    }

    // 斷線 → 延遲後重新廣播
    if (!deviceConnected && oldDeviceConnected) {
      vTaskDelay(pdMS_TO_TICKS(500));
      pServer->startAdvertising();
      Serial.println("[BLE] advertising restarted");
    }
    oldDeviceConnected = deviceConnected;

    vTaskDelay(pdMS_TO_TICKS(COMM_PERIOD_MS));
  }
}

// ─────────────────────────────────────────────────────────────
// Task_LED:狀態指示 + 達標雙響(旗標驅動,不動系統狀態)
// ─────────────────────────────────────────────────────────────
static void serveGoalBeep() {
  // 雙短響 + 狀態燈雙閃,約 800ms;結束後蜂鳴器恢復 alarmActive 對應狀態
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    digitalWrite(PIN_STATUS_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_STATUS_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(i == 0 ? 150 : 250));
  }
  digitalWrite(PIN_BUZZER, alarmActive ? HIGH : LOW);
}

static void Task_LED(void *) {
  pinMode(PIN_STATUS_LED, OUTPUT);
  for (;;) {
    if (goalBeepRequest) {
      goalBeepRequest = false;
      serveGoalBeep();
      continue;
    }
    switch (currentState) {
      case STATE_CONNECTED:  // 恆亮
        digitalWrite(PIN_STATUS_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      case STATE_ERROR:  // 快閃 10Hz
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      default:  // INIT / ADVERTISING:慢閃
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
    }
  }
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  pinMode(PIN_EXT_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  feedbackAllOff();

  // 開機測試音(200ms)
  digitalWrite(PIN_BUZZER, HIGH);
  delay(200);
  digitalWrite(PIN_BUZZER, LOW);

  xAngleMutex = xSemaphoreCreateMutex();

  BLEDevice::init(IRMS_DEVICE_NAME);
  BLEDevice::setMTU(BLE_MTU);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(IRMS_SERVICE_UUID);
  pCharAngleTx = service->createCharacteristic(IRMS_CHAR_ANGLE_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pCharAngleTx->addDescriptor(new BLE2902());
  pCharProfileRx = service->createCharacteristic(IRMS_CHAR_PROFILE_RX, BLECharacteristic::PROPERTY_WRITE);
  pCharProfileRx->setCallbacks(new ProfileCallbacks());
  service->start();

  // OTA:獨立 service,不動到上面既有的感測器契約(見 config.h 註解)。
  BLEService *otaService = pServer->createService(IRMS_OTA_SERVICE_UUID);
  pCharOtaControl = otaService->createCharacteristic(IRMS_CHAR_OTA_CONTROL, BLECharacteristic::PROPERTY_WRITE);
  pCharOtaControl->setCallbacks(new OtaControlCallbacks());
  pCharOtaData = otaService->createCharacteristic(IRMS_CHAR_OTA_DATA, BLECharacteristic::PROPERTY_WRITE_NR);
  pCharOtaData->setCallbacks(new OtaDataCallbacks());
  pCharOtaStatus = otaService->createCharacteristic(IRMS_CHAR_OTA_STATUS, BLECharacteristic::PROPERTY_NOTIFY);
  pCharOtaStatus->addDescriptor(new BLE2902());
  pCharFwVersion = otaService->createCharacteristic(IRMS_CHAR_FW_VERSION, BLECharacteristic::PROPERTY_READ);
  pCharFwVersion->setValue(IRMS_FW_VERSION);
  otaService->start();

  pServer->getAdvertising()->addServiceUUID(IRMS_OTA_SERVICE_UUID);
  pServer->getAdvertising()->start();
  currentState = STATE_ADVERTISING;
  Serial.println("Waiting for BLE connection...");

  xTaskCreatePinnedToCore(Task_Sensor, "Sensor", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Comm, "Comm", 4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(Task_LED, "LED", 2048, nullptr, 1, nullptr, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // FreeRTOS 任務接管,loop 閒置
}
