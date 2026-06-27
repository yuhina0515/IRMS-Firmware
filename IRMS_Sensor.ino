#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

const int addrThigh = 0x68;
const int addrShin = 0x69;
const int pinExtLED = 25; // Target Zone Visual Feedback
const int pinBuzzer = 26;
const int pinLED = 2; // Default ESP32 internal LED

// System States for LED Indication
enum SystemState {
  STATE_INIT,
  STATE_ERROR,
  STATE_ADVERTISING,
  STATE_CONNECTED,
  STATE_GOAL_REACHED
};
volatile SystemState currentState = STATE_INIT;

// I2C Pins for ESP32
const int sdaPin = 21;
const int sclPin = 22;

// Sensor Data Structure
struct SensorAngles {
  float thigh;
  float shin;
  float knee;
  float thighRoll;
  float shinRoll;
  float kneeRoll;
};
SensorAngles latestAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
SemaphoreHandle_t xAngleMutex;

// BLE Constants
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID_ANGLE_TX     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_UUID_PROFILE_RX   "beb5483f-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharAngleTx = NULL;
BLECharacteristic* pCharProfileRx = NULL;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      currentState = STATE_CONNECTED;
      Serial.println("BLE Client Connected");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      currentState = STATE_ADVERTISING;
      Serial.println("BLE Client Disconnected");
    }
};

class MyProfileCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        String data = rxValue;
        Serial.print("Received Command: ");
        Serial.println(data);
        
        // App Command Channel Parser
        if (data.startsWith("CMD:")) {
          if (data == "CMD:GOAL") {
            currentState = STATE_GOAL_REACHED;
          } else if (data == "CMD:LED_ON") {
            digitalWrite(pinExtLED, HIGH);
          } else if (data == "CMD:LED_OFF") {
            digitalWrite(pinExtLED, LOW);
          } else if (data == "CMD:ALARM_ON") {
            digitalWrite(pinBuzzer, HIGH);
          } else if (data == "CMD:ALARM_OFF") {
            digitalWrite(pinBuzzer, LOW);
          }
        }
      }
    }
};

bool setupMPU6050(int addr) {
  Wire.beginTransmission(addr);
  if (Wire.endTransmission(true) != 0) {
    Serial.print("MPU Error: 0x");
    Serial.println(addr, HEX);
    return false;
  }
  
  // 喚醒 MPU6050 (PWR_MGMT_1)
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);    // Wake up
  if (Wire.endTransmission(true) != 0) {
    Serial.println("MPU Config Error: PWR_MGMT_1");
    return false;
  }
  
  // 設定陀螺儀量程 +/- 500 deg/s (GYRO_CONFIG)
  Wire.beginTransmission(addr);
  Wire.write(0x1B);
  Wire.write(0x08);
  if (Wire.endTransmission(true) != 0) {
    Serial.println("MPU Config Error: GYRO_CONFIG");
    return false;
  }
  
  // 設定加速度計量程 +/- 4g (ACCEL_CONFIG)
  Wire.beginTransmission(addr);
  Wire.write(0x1C);
  Wire.write(0x08);
  if (Wire.endTransmission(true) != 0) {
    Serial.println("MPU Config Error: ACCEL_CONFIG");
    return false;
  }
  
  return true;
}

bool readMPU6050(int addr, float &accelAngle, float &accelRoll, float &gyroRate, float &gyroRateRoll) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  uint8_t bytes = Wire.requestFrom(addr, 14, true);
  
  if (bytes != 14) return false;
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temp
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // Pitch (sagittal flexion)
  accelAngle = atan2(ay, az) * 180.0 / PI;
  gyroRate = gy / 65.5;

  // Roll (coronal lateral bend)
  accelRoll = atan2(ax, az) * 180.0 / PI;
  gyroRateRoll = gx / 65.5;
  return true;
}

float gyroOffsetThigh = 0, gyroOffsetThighRoll = 0;
float gyroOffsetShin = 0, gyroOffsetShinRoll = 0;

void calibrateMPU6050(int addr, float &gyroOffset, float &gyroOffsetRoll) {
  Serial.print("Calibrating 0x"); Serial.println(addr, HEX);
  float sumF = 0.0f;
  float sumRollF = 0.0f;
  int valid = 0;
  for (int i = 0; i < 200; i++) {
    float a, r, g, gr;
    if (readMPU6050(addr, a, r, g, gr)) {
      sumF += g;        // 直接累加 float 角速度，避免 float→long→float 精度損失
      sumRollF += gr;
      valid++;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (valid > 0) {
    gyroOffset = sumF / valid;
    gyroOffsetRoll = sumRollF / valid;
  }
}

// Tasks
void Task_Sensor(void *pvParameters) {
  (void) pvParameters;
  bool ok1 = setupMPU6050(addrThigh);
  bool ok2 = setupMPU6050(addrShin);

  if (!ok1 || !ok2) {
    currentState = STATE_ERROR;
    Serial.println("CRITICAL: MPU6050 connection failed at startup!");
  }

  calibrateMPU6050(addrThigh, gyroOffsetThigh, gyroOffsetThighRoll);
  calibrateMPU6050(addrShin, gyroOffsetShin, gyroOffsetShinRoll);

  float angleThigh = 0, angleThighRoll = 0;
  float angleShin = 0, angleShinRoll = 0;
  
  // Read initial angles
  float a1, r1, g1, gr1, a2, r2, g2, gr2;
  if (readMPU6050(addrThigh, a1, r1, g1, gr1)) {
    angleThigh = a1;
    angleThighRoll = r1;
  }
  if (readMPU6050(addrShin, a2, r2, g2, gr2)) {
    angleShin = a2;
    angleShinRoll = r2;
  }

  unsigned long lastTime = millis();
  int i2cFailCount = 0;
  
  for (;;) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    float accAngleThigh, accRollThigh, gyroRateThigh, gyroRateRollThigh;
    bool read1 = readMPU6050(addrThigh, accAngleThigh, accRollThigh, gyroRateThigh, gyroRateRollThigh);
    
    float accAngleShin, accRollShin, gyroRateShin, gyroRateRollShin;
    bool read2 = readMPU6050(addrShin, accAngleShin, accRollShin, gyroRateShin, gyroRateRollShin);

    if (!read1 || !read2) {
      i2cFailCount++;
      if (i2cFailCount > 5) {
        currentState = STATE_ERROR;
        SensorAngles errData;
        errData.thigh = -100.0;
        errData.shin = -100.0;
        errData.knee = -100.0;
        errData.thighRoll = -100.0;
        errData.shinRoll = -100.0;
        errData.kneeRoll = -100.0;
        
        if (xSemaphoreTake(xAngleMutex, portMAX_DELAY) == pdTRUE) {
          latestAngles = errData;
          xSemaphoreGive(xAngleMutex);
        }
        
        // Auto-recovery attempt
        Wire.begin(sdaPin, sclPin);
        Wire.setTimeOut(1000); // 復原後重新設定 I2C 超時
        bool ok1 = setupMPU6050(addrThigh);
        bool ok2 = setupMPU6050(addrShin);
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (ok1 && ok2) {
          float a1, r1, g1, gr1, a2, r2, g2, gr2;
          if (readMPU6050(addrThigh, a1, r1, g1, gr1) && readMPU6050(addrShin, a2, r2, g2, gr2)) {
            angleThigh = a1;
            angleThighRoll = r1;
            angleShin = a2;
            angleShinRoll = r2;
            i2cFailCount = 0;
            currentState = deviceConnected ? STATE_CONNECTED : STATE_ADVERTISING;
          }
        }
        lastTime = millis();
        continue;
      }
    } else {
      i2cFailCount = 0;
      
      gyroRateThigh -= gyroOffsetThigh;
      angleThigh = 0.85 * (angleThigh + gyroRateThigh * dt) + 0.15 * accAngleThigh;

      gyroRateRollThigh -= gyroOffsetThighRoll;
      angleThighRoll = 0.85 * (angleThighRoll + gyroRateRollThigh * dt) + 0.15 * accRollThigh;

      gyroRateShin -= gyroOffsetShin;
      angleShin = 0.85 * (angleShin + gyroRateShin * dt) + 0.15 * accAngleShin;

      gyroRateRollShin -= gyroOffsetShinRoll;
      angleShinRoll = 0.85 * (angleShinRoll + gyroRateRollShin * dt) + 0.15 * accRollShin;

      float kneeAngle = fabs(angleThigh - angleShin);
      float kneeAngleRoll = fabs(angleThighRoll - angleShinRoll);
      
      if (currentState == STATE_ERROR) {
        currentState = deviceConnected ? STATE_CONNECTED : STATE_ADVERTISING; // 依據 BLE 連線狀態設定正確狀態
      }
      SensorAngles angles;
      angles.thigh = angleThigh;
      angles.shin = angleShin;
      angles.knee = kneeAngle;
      angles.thighRoll = angleThighRoll;
      angles.shinRoll = angleShinRoll;
      angles.kneeRoll = kneeAngleRoll;
      
      if (xSemaphoreTake(xAngleMutex, portMAX_DELAY) == pdTRUE) {
        latestAngles = angles;
        xSemaphoreGive(xAngleMutex);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
  }
}

void Task_Comm(void *pvParameters) {
  (void) pvParameters;
  SensorAngles currentData;
  for (;;) {
    if (deviceConnected) {
      // Read latest angles from the global variable using the mutex
      bool hasData = false;
      if (xSemaphoreTake(xAngleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentData = latestAngles;
        hasData = true;
        xSemaphoreGive(xAngleMutex);
      }
      
      if (hasData) {
        String msg;
        if (currentState == STATE_ERROR) {
          msg = "ERR:1"; // I2C Disconnected
        } else {
          // 使用 snprintf 避免 String 串接產生的堆積碎片化
          char buf[72];
          snprintf(buf, sizeof(buf), "T:%.1f,S:%.1f,K:%.1f,TR:%.1f,SR:%.1f,KR:%.1f",
                   currentData.thigh, currentData.shin, currentData.knee,
                   currentData.thighRoll, currentData.shinRoll, currentData.kneeRoll);
          msg = buf;
        }
        pCharAngleTx->setValue((uint8_t*)msg.c_str(), msg.length());
        pCharAngleTx->notify();
      }
    }
    
    // Handle disconnects
    if (!deviceConnected && oldDeviceConnected) {
        vTaskDelay(pdMS_TO_TICKS(500)); 
        pServer->startAdvertising();
        Serial.println("Restarted Advertising");
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    vTaskDelay(pdMS_TO_TICKS(40)); // Update BLE every 40ms (25Hz) for real-time feel
  }
}

void Task_LED(void *pvParameters) {
  (void) pvParameters;
  pinMode(pinLED, OUTPUT);
  for (;;) {
    switch (currentState) {
      case STATE_INIT:
      case STATE_ADVERTISING:
        // Slow Blink (Wait for connection)
        digitalWrite(pinLED, !digitalRead(pinLED));
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
      case STATE_CONNECTED:
        // Solid ON (Connected)
        digitalWrite(pinLED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      case STATE_ERROR:
        // Fast Blink (Hardware Error)
        digitalWrite(pinLED, !digitalRead(pinLED));
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      case STATE_GOAL_REACHED:
        // Rapid double blink on pinLED and double beep on pinBuzzer asynchronously
        digitalWrite(pinLED, LOW);
        digitalWrite(pinBuzzer, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        
        digitalWrite(pinBuzzer, LOW);
        digitalWrite(pinLED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(150));
        
        digitalWrite(pinBuzzer, HIGH);
        digitalWrite(pinLED, LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
        
        digitalWrite(pinBuzzer, LOW);
        digitalWrite(pinLED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(250)); // Total 800ms
        
        currentState = STATE_CONNECTED;
        break;
      default:
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(sdaPin, sclPin);
  Wire.setTimeOut(1000); // 設定 I2C 超時 1 秒，防止匯流排鎖死導致 Task_Sensor 卡死
  
  pinMode(pinExtLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT); // Configure buzzer pin as digital output
  digitalWrite(pinBuzzer, LOW); // Start silent
  
  // Startup Test Beep (200ms) for Active Buzzer
  digitalWrite(pinBuzzer, HIGH);
  delay(200);
  digitalWrite(pinBuzzer, LOW);

  // RTOS Init
  xAngleMutex = xSemaphoreCreateMutex();

  // BLE Setup
  BLEDevice::init("IRMS_Device");
  BLEDevice::setMTU(128); // 確保 6 軸數據包（最大 62 bytes）可完整傳送
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharAngleTx = pService->createCharacteristic(
                      CHAR_UUID_ANGLE_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharAngleTx->addDescriptor(new BLE2902());

  pCharProfileRx = pService->createCharacteristic(
                                         CHAR_UUID_PROFILE_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharProfileRx->setCallbacks(new MyProfileCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  currentState = STATE_ADVERTISING; // 明確設定初始狀態為廣播模式
  Serial.println("Waiting for BLE connection...");

  // Start Tasks
  xTaskCreatePinnedToCore(Task_Sensor, "Sensor", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_Comm, "Comm", 4096, NULL, 1, NULL, 0); // Network tasks typically run on core 0
  xTaskCreatePinnedToCore(Task_LED, "LED", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // FreeRTOS will manage tasks, loop is not used
  vTaskDelay(portMAX_DELAY);
}
