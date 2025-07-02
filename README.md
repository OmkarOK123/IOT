## **1. IoT Fundamentals**

### Definition:

**Internet of Things (IoT)** refers to the interconnection of physical devices (things) through the internet, enabling them to collect, exchange, and act on data.

### Key Components:

* **Devices/Things:** Sensors, actuators, embedded systems
* **Connectivity:** Wi-Fi, Bluetooth, ZigBee, etc.
* **Data Processing:** Microcontrollers, edge computing, cloud
* **User Interface:** Web, mobile apps, dashboards

### Characteristics:

* Interoperability
* Automation and control
* Data-driven insights
* Scalability and remote access

---

## **2. IoT Security**

### Importance:

IoT devices are often vulnerable due to limited resources and poor security practices. IoT security ensures confidentiality, integrity, and availability of data.

### Key Areas:

* **Device Security:** Secure boot, tamper detection, firmware updates
* **Communication Security:** TLS/SSL, VPN, encrypted data
* **Authentication:** OAuth, JWT, two-factor authentication
* **Data Security:** Encryption at rest and in transit
* **Network Security:** Firewalls, IDS/IPS
* **Privacy:** GDPR compliance, data anonymization

---

## **3. Internet Protocols in IoT**

### Key Protocols:

* **IPv4/IPv6:** IP addressing; IPv6 preferred for scalability
* **TCP/UDP:** Reliable (TCP) and fast (UDP) transport layer protocols
* **HTTP/HTTPS:** Application layer protocols for communication
* **DNS:** Resolves domain names
* **NAT/PAT:** Network Address Translation for local IPs
* **6LoWPAN:** IPv6 over Low power Wireless Personal Area Networks

---

## **4. IoT Technologies**

### Categories:

* **Connectivity:** Wi-Fi, ZigBee, LoRa, NB-IoT, Bluetooth, Z-Wave
* **Processing:** Microcontrollers (e.g., Arduino, STM32), edge computing, cloud computing (AWS IoT, Azure IoT)
* **Storage:** Cloud, on-device memory, databases like InfluxDB, MongoDB
* **Analytics:** AI/ML for predictive maintenance, anomaly detection
* **Visualization:** Dashboards (e.g., Grafana, ThingsBoard)

---

## **5. IoT Protocols**

### Network/Transport/Application Layer:

* **MQTT (Message Queuing Telemetry Transport):** Lightweight, pub-sub model, ideal for constrained devices
* **CoAP (Constrained Application Protocol):** RESTful, UDP-based
* **AMQP (Advanced Message Queuing Protocol):** Reliable but heavier than MQTT
* **DDS (Data Distribution Service):** Real-time, distributed systems
* **XMPP (Extensible Messaging and Presence Protocol):** Real-time messaging
* **HTTP/HTTPS:** Web-based RESTful APIs

---

## **6. Serial Communication Protocols**

### Common Protocols in Embedded IoT Devices:

* **UART (Universal Asynchronous Receiver/Transmitter):** Simple, point-to-point communication
* **SPI (Serial Peripheral Interface):** Fast, used for short-distance device communication (master-slave)
* **I2C (Inter-Integrated Circuit):** Two-wire protocol for communication between ICs
* **RS-232/RS-485:** Legacy but still used in industrial IoT systems
* **CAN (Controller Area Network):** Robust protocol in automotive and industrial IoT

---

## **7. Understanding of IoT Technology**

### Technical Stack:

* **Perception Layer:** Sensors, actuators
* **Network Layer:** Gateways, communication protocols
* **Processing Layer:** Edge devices, fog computing, cloud servers
* **Application Layer:** User interfaces, services (smart home, industrial automation)

### Enablers:

* Miniaturization of sensors
* Cloud and edge computing
* AI/ML and big data
* Mobile and wireless communication
* Open-source development platforms

---

## **8. IoT Applications**

### Key Domains:

* **Smart Home:** Automation, security, lighting, energy monitoring
* **Industrial IoT (IIoT):** Predictive maintenance, factory automation
* **Healthcare:** Remote monitoring, smart wearables
* **Agriculture:** Soil moisture monitoring, smart irrigation
* **Transport:** Fleet management, smart logistics, connected vehicles
* **Environment:** Pollution monitoring, weather stations

---

## **9. Sensors and Actuators in IoT**

### **Sensors** (Input Devices):

They collect data from the physical environment.

**Types:**

* Temperature (e.g., LM35)
* Humidity (e.g., DHT11)
* Pressure (e.g., BMP180)
* Proximity (e.g., IR sensor, ultrasonic)
* Motion (e.g., PIR, accelerometer)
* Light (e.g., LDR)
* Gas (e.g., MQ series)

### **Actuators** (Output Devices):

They convert electrical signals into physical action.

**Types:**

* Motors (DC, stepper, servo)
* Relays
* Buzzers
* LED displays
* Valves

### Integration:

Microcontrollers read sensor data, process it, and control actuators to perform actions (like turning on a fan when temperature is high).

---

## 🔌 **1. What is Arduino?**

**Arduino** is an open-source electronics platform based on easy-to-use hardware and software. It's widely used for prototyping and learning in embedded systems, robotics, IoT, and automation.

### 🧠 At its Core:

* **Microcontroller-based development board**
* **Arduino IDE** for writing, compiling, and uploading code (C/C++ based)
* **Digital and analog I/O pins** to connect sensors, actuators, and other devices

---

## ⚙️ **2. Common Arduino Boards**

| Board     | Microcontroller | Flash  | Operating Voltage | I/O Pins              |
| --------- | --------------- | ------ | ----------------- | --------------------- |
| Uno R3    | ATmega328P      | 32 KB  | 5V                | 14 Digital, 6 Analog  |
| Nano      | ATmega328P      | 32 KB  | 5V                | 22 I/O                |
| Mega 2560 | ATmega2560      | 256 KB | 5V                | 54 Digital, 16 Analog |
| Leonardo  | ATmega32u4      | 32 KB  | 5V                | 20 I/O                |
| MKR1000   | SAMD21 + WiFi   | 256 KB | 3.3V              | IoT Ready             |

---

## 🔧 **3. Arduino IDE Basics**

* **Sketch (.ino file)** = Arduino program
* Structure:

  ```cpp
  void setup() {
    // runs once
  }

  void loop() {
    // runs repeatedly
  }
  ```
* **Upload via USB** to the board using the "Upload" button.

---

## 🔌 **4. Pinout Overview (Arduino Uno)**

| Pin Type    | Description                             |
| ----------- | --------------------------------------- |
| Digital I/O | Pins 0–13 for HIGH/LOW (on/off) signals |
| PWM         | Pins \~3, \~5, \~6, \~9, \~10, \~11     |
| Analog In   | Pins A0–A5 for sensor inputs (0-1023)   |
| Power       | 3.3V, 5V, GND, Vin                      |
| Serial      | Pin 0 (RX), Pin 1 (TX)                  |

---

## 🧪 **5. Sample Arduino Code**

### Blink LED:

```cpp
void setup() {
  pinMode(13, OUTPUT); // Set pin 13 as output
}

void loop() {
  digitalWrite(13, HIGH); // Turn LED on
  delay(1000);            // Wait 1 sec
  digitalWrite(13, LOW);  // Turn LED off
  delay(1000);            // Wait 1 sec
}
```

---

## 📡 **6. Connecting Sensors and Actuators**

### Examples:

* **DHT11**: Temperature & Humidity Sensor (Digital Pin)
* **LDR**: Light Sensor (Analog Pin)
* **Ultrasonic**: Distance Sensor (Digital Pins - Trigger & Echo)
* **Servo Motor**: Control using PWM
* **Relay Module**: Switch AC loads

---

## 📡 **7. Serial Communication in Arduino**

Used for debugging, logging, or communication with PC/other devices.

```cpp
void setup() {
  Serial.begin(9600); // Start serial at 9600 baud
}

void loop() {
  Serial.println("Hello from Arduino!");
  delay(1000);
}
```

---

## 🌐 **8. Arduino in IoT**

With additional modules:

* **ESP8266 / ESP32**: Wi-Fi-enabled boards for IoT
* **Ethernet Shield**: For wired connections
* **Bluetooth Modules**: HC-05, HC-06
* **LoRa Modules**: Long-range communication
* **GSM/GPRS Modules**: Cellular-based communication

---

### ✅ Components You Mentioned:

1. **Ultrasonic Sensor (HC-SR04 - 4 pins)**
2. **LDR (Light Dependent Resistor)**
3. **IR Sensor (3-pin type)**
4. **Push Button**
5. **DHT11 (Temperature and Humidity Sensor)**

---

## 📌 1. **Ultrasonic Sensor (HC-SR04)**

### **Pins**:

* VCC → 5V
* GND → GND
* TRIG → Digital pin 9
* ECHO → Digital pin 10

### **Code**:

```cpp
#define TRIG 9
#define ECHO 10

void setup() {
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  long duration;
  float distance;

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = (duration * 0.034) / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500);
}
```

---

## 📌 2. **LDR Sensor**

### **Circuit**:

Use a voltage divider:

* LDR one side → 5V
* LDR other side → A0 + 10kΩ resistor to GND

### **Code**:

```cpp
void setup() {
  Serial.begin(9600);
}

void loop() {
  int ldrValue = analogRead(A0);
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  delay(500);
}
```

---

## 📌 3. **IR Sensor (3-pin)**

### **Pins**:

* VCC → 5V
* GND → GND
* OUT → Digital pin 2

### **Code**:

```cpp
#define IR_PIN 2

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  int val = digitalRead(IR_PIN);
  if (val == LOW) { // Depends on module, some give LOW on detection
    Serial.println("Object Detected!");
  } else {
    Serial.println("No Object.");
  }
  delay(500);
}
```

---

## 📌 4. **Push Button**

### **Circuit**:

* One side → GND
* Other side → Digital pin 7 + pull-up resistor or use internal pull-up

### **Code (Internal Pull-up)**:

```cpp
#define BUTTON_PIN 7

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Button Pressed");
  } else {
    Serial.println("Button Released");
  }
  delay(300);
}
```

---

## 📌 5. **DHT11 (Temperature & Humidity Sensor)**

### **Pins**:

* VCC → 5V
* GND → GND
* DATA → Digital pin 8

> 📦 You need the **DHT library**:

* Go to Arduino IDE → Tools → Manage Libraries → Search for **“DHT sensor library”** by Adafruit and install it.

### **Code**:

```cpp
#include "DHT.h"
#define DHTPIN 8
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" °C, Humidity: ");
  Serial.print(hum);
  Serial.println(" %");
  delay(2000);
}
```

---

## 🔌 Combined Connection Summary:

| Component     | Arduino Pin |
| ------------- | ----------- |
| HC-SR04 TRIG  | D9          |
| HC-SR04 ECHO  | D10         |
| LDR Out       | A0          |
| IR Sensor OUT | D2          |
| Button        | D7          |
| DHT11 Data    | D8          |

---


