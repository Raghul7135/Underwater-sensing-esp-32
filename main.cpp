// ============================================
// ESP32-CAM Underwater Organism Detection System
// Uses: DS18B20, MPU6050, A02YYUW Ultrasonic, OV2640 Camera, ThingSpeak
// ============================================

#include <WiFi.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NewPing.h>
#include <esp_camera.h>
#include <base64.h>
#include <ThingSpeak.h>

// === Wi-Fi & ThingSpeak ===
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
unsigned long sensorChannelID = YOUR_SENSOR_CHANNEL_ID;
const char* sensorWriteAPIKey = "YOUR_SENSOR_WRITE_API_KEY";
unsigned long imageChannelID = YOUR_IMAGE_CHANNEL_ID; // Optional, for metadata
const char* imageWriteAPIKey = "YOUR_IMAGE_WRITE_API_KEY";

// === Pins ===
#define TEMP_PIN            4
#define MPU_SDA             14
#define MPU_SCL             2
#define ULTRASONIC_TRIG     12
#define ULTRASONIC_ECHO     13
#define LED_PIN             0   // Built-in flash or external LED

// === Ultrasonic ===
#define MAX_DISTANCE_CM     450 // 4.5m
NewPing sonar(ULTRASONIC_TRIG, ULTRASONIC_ECHO, MAX_DISTANCE_CM);

// === Temperature ===
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);

// === Gyro ===
Adafruit_MPU6050 mpu;

// === WiFi Client ===
WiFiClient client;
unsigned long lastUpload = 0;
const long uploadInterval = 10000; // 10 sec

// === Detection Thresholds ===
const float TEMP_VARIANCE_THRESHOLD = 2.0; // °C
const float GYRO_STABILITY_THRESHOLD = 5.0; // deg/s
const int PROXIMITY_THRESHOLD_CM = 50;
bool organismDetected = false;
float lastTemp = 0;
float baselineTemp = 20.0; // Will calibrate early

// ============================================
// CAMERA CONFIG FOR OV2640 (AI-Thinker ESP32-CAM)
// ============================================
camera_config_t config;
void initCamera() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; // 320x240 - good for bandwidth
  config.jpeg_quality = 12; // 0-63, lower = better quality, larger file
  config.fb_count = 1;

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize sensors
  sensors.begin();
  Wire.begin(MPU_SDA, MPU_SCL);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  initCamera();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  ThingSpeak.begin(client);
}

// ============================================
// READ SENSORS
// ============================================
float readTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

sensors_event_t a, g, temp;
void readGyro(float &gx, float &gy, float &gz) {
  mpu.getEvent(&a, &g, &temp);
  gx = g.gyro.x * 180 / PI; // Convert rad/s to deg/s
  gy = g.gyro.y * 180 / PI;
  gz = g.gyro.z * 180 / PI;
}

float readUltrasonic() {
  unsigned int uS = sonar.ping(); // Microseconds
  float soundSpeed = 1480.0 + (0.0046 * lastTemp); // m/s in water
  float distance = (uS / 1000000.0) * soundSpeed * 100.0 / 2.0; // cm
  return distance;
}

// ============================================
// CAPTURE IMAGE
// ============================================
String captureImage() {
  digitalWrite(LED_PIN, HIGH); // Flash ON
  delay(100); // Let LED stabilize
  camera_fb_t * fb = esp_camera_fb_get();
  digitalWrite(LED_PIN, LOW); // Flash OFF

  if (!fb) {
    Serial.println("Camera capture failed");
    return "";
  }

  // Encode to Base64 (for ThingSpeak image field is tricky;
  // alternatively, send to custom server or use ThingSpeak metadata field)
  String imageStr = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return imageStr;
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Read sensors
  float temp = readTemperature();
  float gyro_x, gyro_y, gyro_z;
  readGyro(gyro_x, gyro_y, gyro_z);
  float distance = readUltrasonic();

  // Update baseline temp (first 10 readings)
  static int calibCount = 0;
  if (calibCount < 10) {
    baselineTemp = (baselineTemp * calibCount + temp) / (calibCount + 1);
    calibCount++;
  }

  // Stability check
  bool stable = (abs(gyro_x) < GYRO_STABILITY_THRESHOLD) &&
                (abs(gyro_y) < GYRO_STABILITY_THRESHOLD);
  
  organismDetected = false;
  if (stable && distance > 0 && distance < PROXIMITY_THRESHOLD_CM) {
    float tempDiff = abs(temp - baselineTemp);
    if (tempDiff > TEMP_VARIANCE_THRESHOLD) {
      organismDetected = true;
      Serial.println(">>> Organism detected! Capturing image...");
      String img = captureImage();
      if (img.length() > 0) {
        // Optional: Store or transmit img
        // ThingSpeak doesn't support direct image upload,
        // so we'll just log detection flag + metadata
      }
    }
  }

  // Upload to ThingSpeak every 10 sec
  if (millis() - lastUpload > uploadInterval) {
    if (ThingSpeak.writeField(sensorChannelID, 1, temp, sensorWriteAPIKey) == 200 &&
        ThingSpeak.writeField(sensorChannelID, 2, distance, sensorWriteAPIKey) == 200 &&
        ThingSpeak.writeField(sensorChannelID, 3, gyro_x, sensorWriteAPIKey) == 200 &&
        ThingSpeak.writeField(sensorChannelID, 4, organismDetected ? 1 : 0, sensorWriteAPIKey) == 200) {
      Serial.println("Data uploaded to ThingSpeak");
    } else {
      Serial.println("ThingSpeak upload failed");
    }
    lastUpload = millis();
  }

  delay(100); // Main loop delay
}
