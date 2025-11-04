# Underwater-sensing-esp-32
# 🌊 Underwater Organism Detection System with ESP32-CAM

**IoT-based smart monitoring for aquatic life using multi-sensor fusion and visual verification**

---

## 📌 Overview

This project enhances an IoT underwater environment monitoring system by integrating **visual imaging (ESP32-CAM)** with **temperature sensing**, **device orientation tracking**, and **ultrasonic echo analysis** to detect and verify the presence of aquatic organisms (e.g., fish schools, marine debris). The system is designed for submersion up to **10 meters**, featuring **corrosion-resistant, waterproof hardware**, and transmits real-time sensor data and triggered images to **ThingSpeak** for remote analysis and visualization.

---

## 🔬 Key Features

- 📸 **Visual Verification**: ESP32-CAM (OV2640) captures 2MP JPEG images on anomaly detection.  
- 🌡️ **Temperature Monitoring**: DS18B20 waterproof sensor (±0.5°C accuracy, -55°C to +125°C).  
- 🧭 **Orientation Stabilization**: MPU6050 6-axis gyroscope & accelerometer compensates for water currents.  
- 🔊 **Ultrasonic Detection**: A02YYUW waterproof sensor (5cm–4.5m range, 1cm resolution) with temperature-compensated speed of sound.  
- 📶 **IoT Integration**: Wi-Fi uploads sensor data and images to **ThingSpeak** (dual-channel: telemetry + images).  
- 💡 **Low-Light Imaging**: Integrated waterproof LED flash for murky or deep-water conditions.  
- 🔋 **Power-Efficient**: Supports deep-sleep mode for battery deployments in remote locations.  
- 💰 **Low-Cost**: Full system under **$60**, including enclosure and storage.  

---

## 🛠️ Hardware Components

| Component | Purpose |
|----------|--------|
| **ESP32-CAM** (AI Thinker) | Main controller: Wi-Fi, camera, processing |
| **DS18B20** (Waterproof) | Temperature sensing in water |
| **MPU6050** | 6-axis orientation & motion stabilization |
| **A02YYUW** | Waterproof ultrasonic distance sensor (200kHz) |
| **OV2640 Camera** | Image capture for organism verification |
| **Waterproof High-Brightness LED** | Illumination for low-visibility conditions |
| **4.7kΩ Pull-up Resistor** | Required for DS18B20 1-Wire communication |
| **IP68 Acrylic Enclosure** | Pressure-resistant (up to 10m), transparent lens window |
| **MicroSD Card Slot** | Local image buffering before upload |
| **FTDI Programmer** | For flashing and serial debug |

> **Power**: 3.3V–5V supply (use 5V with regulator for ultrasonic sensor)

---

## 📐 Pin Connections

| Sensor | ESP32-CAM Pin | Notes |
|--------|---------------|------|
| DS18B20 (Data) | GPIO4 | Pull-up 4.7kΩ to 3.3V |
| MPU6050 (SDA) | GPIO14 | I²C |
| MPU6050 (SCL) | GPIO2 | I²C |
| A02YYUW (Trig) | GPIO12 | |
| A02YYUW (Echo) | GPIO13 | |
| LED Flash | GPIO0 | Active-high |
| Camera | Built-in | OV2640 via native interface |
| Power | 5V (regulated for ultrasonic), 3.3V for logic | |

---

## 🧑‍💻 Software Setup

### Required Libraries (Arduino IDE)
- `ESP32` board support (AI Thinker ESP32-CAM)
- `OneWire` + `DallasTemperature` → DS18B20
- `Adafruit MPU6050` + `Adafruit Unified Sensor` → Gyro/Accel
- `NewPing` → Ultrasonic (modified for water)
- `esp_camera` → Native camera driver
- `WiFi`, `HTTPClient`, `ThingSpeak` → IoT upload

### Installation Steps
1. Install **Arduino IDE** (v2.0+ recommended)  
2. Add ESP32 board support via **Preferences → Additional Boards Manager URLs**:
3. 3. Install **ESP32** package → Select **AI Thinker ESP32-CAM**  
4. Install libraries via **Library Manager**  
5. Configure Wi-Fi SSID, password, and ThingSpeak API keys in `config.h`

---

## 🔄 System Workflow

1. **Boot & Initialization**:  
- Connect to Wi-Fi  
- Initialize DS18B20, MPU6050, A02YYUW, and camera  
- Calibrate ultrasonic speed using real-time temperature  

2. **Continuous Monitoring Loop**:  
- Check device stability (MPU6050: gyro < 5°/s)  
- Fire ultrasonic pulse → measure distance  
- Read water temperature  
- If **distance < 50cm** AND **temp variance > 2°C**, flag anomaly  

3. **Trigger & Capture**:  
- Turn on LED flash  
- Capture image (320×240 JPEG for bandwidth efficiency)  
- Store image in buffer + log sensor context  

4. **Upload to Cloud**:  
- Every 10s: POST sensor data to ThingSpeak (Temp, Distance, Gyro, Flag)  
- On detection: POST base64-encoded image to image channel  

5. **Power Management**:  
- Enter deep sleep between scan cycles (optional for battery operation)

---

## 📤 ThingSpeak Integration

- **Channel 1**: Sensor telemetry (4 fields)  
  - Field 1: Temperature (°C)  
  - Field 2: Ultrasonic distance (cm)  
  - Field 3: Gyro X (°/s)  
  - Field 4: Detection flag (1 = organism detected)  

- **Channel 2**: Image alerts  
  - Upload JPEG as base64-encoded string via **Write API**  

> ⚠️ Ensure image size < 5 MB (use compression or lower resolution if needed)

---

## 🧪 Testing Protocol

### 1. **Bench Test (Aquarium)**
- Simulate organisms with moving objects  
- Verify detection accuracy >85%  
- Confirm image clarity with LED in low light  

### 2. **Field Test (Shallow Water: 1–2m)**
- Tune gyro thresholds for natural currents  
- Adjust camera exposure for turbidity  
- Validate Wi-Fi signal range and upload reliability  

### 3. **Debugging Tips**
- Use UART0 (115200 baud) for real-time serial logs  
- Shorten sensor cables to reduce noise  
- Add signal amplifier if ultrasonic echo weakens  
- Log all data to microSD for offline MATLAB analysis  

---

## 📁 Project Structure

---

## 📜 License

This project is open-source for educational and research purposes.  
Hardware design and software are provided **"as is"** — use at your own risk in aquatic environments.

> **Note**: Always pressure-test enclosures before deep deployment.  

---

## 🙌 Acknowledgments

- Based on prior ESP32 IoT and ThingSpeak experience  
- Inspired by marine biology monitoring needs  
- Community libraries: Adafruit, PaulStoffregen (NewPing), Miles Burton (DallasTemp)

---

**Deploy. Detect. Discover.** 🐠📡  

*Made for the ocean, by engineers who care.*
