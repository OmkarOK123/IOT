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

## ✅ 1. **Blink LED**

### 🔌 Components:

* Arduino Uno
* LED
* 220Ω resistor
* Breadboard
* Jumper wires

### 🔗 Circuit Connections:

* LED anode (+) → Digital Pin 13 (via resistor)
* LED cathode (–) → GND

### 💡 Code:

```cpp
void setup() {
  pinMode(13, OUTPUT); // Set pin 13 as output
}

void loop() {
  digitalWrite(13, HIGH); // Turn LED ON
  delay(1000);            // Wait 1 second
  digitalWrite(13, LOW);  // Turn LED OFF
  delay(1000);            // Wait 1 second
}
```

---

## ✅ 2. **Control LED with Push Button**

### 🔌 Components:

* Arduino Uno
* LED
* 220Ω resistor
* Push button
* 10kΩ pull-down resistor
* Breadboard
* Jumper wires

### 🔗 Circuit Connections:

* LED anode → Pin 13 (via 220Ω)
* LED cathode → GND
* Button one side → Pin 2
* Button other side → GND
* Pull-down resistor: 10kΩ between pin 2 and GND

### 💡 Code:

```cpp
void setup() {
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);
}

void loop() {
  int buttonState = digitalRead(2);
  digitalWrite(13, buttonState);
}
```

---

## ✅ 3. **Temperature Sensor (LM35)**

### 🔌 Components:

* Arduino Uno
* LM35 Temperature sensor
* Breadboard
* Jumper wires

### 🔗 Circuit Connections:

* LM35 VCC → 5V
* LM35 GND → GND
* LM35 OUT → A0

### 💡 Code:

```cpp
void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  float temperatureC = voltage * 100;
  Serial.print("Temp: ");
  Serial.print(temperatureC);
  Serial.println(" °C");
  delay(1000);
}
```

---

## ✅ 4. **LED Fading with PWM**

### 🔌 Components:

* Arduino Uno
* LED
* 220Ω resistor
* Breadboard
* Jumper wires

### 🔗 Circuit Connections:

* LED anode → Pin 9 (via 220Ω)
* LED cathode → GND

### 💡 Code:

```cpp
void setup() {
  pinMode(9, OUTPUT);
}

void loop() {
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(9, brightness);
    delay(10);
  }
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(9, brightness);
    delay(10);
  }
}
```

---

## ✅ 5. **Ultrasonic Sensor (HC-SR04) Distance Display**

### 🔌 Components:

* Arduino Uno
* HC-SR04 Ultrasonic Sensor
* Breadboard
* Jumper wires

### 🔗 Circuit Connections:

* VCC → 5V
* GND → GND
* Trig → Pin 9
* Echo → Pin 10

### 💡 Code:

```cpp
#define trigPin 9
#define echoPin 10

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500);
}
```

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
---

## ✅ **PWM (Pulse Width Modulation) – Theory for IoT Students**

### 🔹 **What is PWM?**

PWM stands for **Pulse Width Modulation**. It is a technique used to simulate **analog output using digital signals** by rapidly turning a digital pin ON and OFF.

Even though microcontrollers can only output HIGH or LOW (digital), PWM lets us **control power delivered to devices like LEDs, motors, buzzers**, etc.

---

### 🔹 **Why is PWM needed in IoT?**

IoT devices often need to:

* Dim an LED (e.g., smart lights)
* Control the speed of a motor (e.g., smart fans)
* Generate audio tones
* Regulate voltage or simulate analog control using digital pins

Since microcontrollers (like Arduino or NodeMCU) have limited DAC (Digital to Analog Converter) channels or none, **PWM is used as a substitute for analog output**.

---

### 🔹 **How PWM Works**

PWM signal is a **digital square wave** with:

* **ON time** (signal is HIGH)
* **OFF time** (signal is LOW)
* Repeats in a fixed period.

#### 💡 Key Terms:

* **Duty Cycle (%)**: It tells how much time the signal is HIGH in one cycle.

  $$
  \text{Duty Cycle} = \left( \frac{T_{ON}}{T_{ON} + T_{OFF}} \right) \times 100
  $$

* **Frequency (Hz)**: Number of PWM cycles per second.

---

### 🔹 **Examples of PWM Duty Cycles:**

| Duty Cycle | Description             | Output Behavior         |
| ---------- | ----------------------- | ----------------------- |
| 0%         | Always OFF              | No power delivered      |
| 25%        | ON for 25%, OFF for 75% | Dim LED / slow motor    |
| 50%        | ON for 50%, OFF for 50% | Medium brightness/speed |
| 75%        | ON for 75%, OFF for 25% | Bright LED / fast motor |
| 100%       | Always ON               | Full power              |

---

### 🔹 **Where is PWM used in IoT Projects?**

| Application              | Use of PWM                                     |
| ------------------------ | ---------------------------------------------- |
| Smart Lighting           | LED brightness control                         |
| IoT Fan or Motor Control | Adjust motor speed using PWM                   |
| Audio Tones/Buzzers      | Tone generation using specific PWM frequency   |
| Servo Control            | Controlling position using PWM pulses          |
| Smart Home Appliances    | Power control (ex: dimmers, speed controllers) |

---

### 🔹 **Advantages of PWM**

* Efficient power control
* Doesn’t require DAC hardware
* Easily generated using timers in microcontrollers
* Can be used for communication (IR signals, etc.)

---

### 🔹 **PWM Pins in Popular IoT Boards:**

| Board           | PWM Support Pins                      |
| --------------- | ------------------------------------- |
| **Arduino Uno** | Pins 3, 5, 6, 9, 10, 11               |
| **NodeMCU**     | GPIO 0 to GPIO 16 (software PWM)      |
| **STM32**       | Many TIMx Channels (TIM2, TIM3, etc.) |

---

### 🔹 **Simple PWM Code Examples**

#### ✅ Arduino: Fade LED with PWM

```c
int ledPin = 9;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  for (int i = 0; i <= 255; i++) {
    analogWrite(ledPin, i);  // Increase brightness
    delay(10);
  }

  for (int i = 255; i >= 0; i--) {
    analogWrite(ledPin, i);  // Decrease brightness
    delay(10);
  }
}
```

#### ✅ NodeMCU (ESP8266): PWM Example (ESP8266WiFi library)

```cpp
int ledPin = D1;

void setup() {
  pinMode(ledPin, OUTPUT);
  analogWriteFreq(1000); // 1 kHz PWM
}

void loop() {
  analogWrite(ledPin, 512); // 50% duty cycle out of 1023
  delay(1000);
}
```
* LEDs will glow **with brightness based on a sensor/input value** (e.g., analog input from a sensor or a variable).
* This is very useful in **IoT projects** like smart lighting, ambient light control, or fan speed indication.

---

## ✅ **Project: Multiple LEDs Controlled by Value Using PWM**

### 💡 **Concept:**

* Read an analog value (0–1023) from a sensor or variable.
* Divide the value range into **levels**.
* Based on the level, **glow 1 to 5 LEDs** with increasing brightness using PWM.

---

### ✅ **Hardware Required:**

* Arduino Uno
* 5 LEDs
* 5 resistors (220Ω)
* Potentiometer (or simulate with a variable in code)
* Breadboard and jumper wires

---

### ✅ **Circuit Connection:**

| LED  | Arduino Pin |
| ---- | ----------- |
| LED1 | Pin 3       |
| LED2 | Pin 5       |
| LED3 | Pin 6       |
| LED4 | Pin 9       |
| LED5 | Pin 10      |

**Analog Input**: A0 (e.g., connected to potentiometer)

---

### ✅ **Code with Explanation**

```c
// Define LED pins
int ledPins[] = {3, 5, 6, 9, 10};
int numLeds = 5;

void setup() {
  // Set all LED pins as OUTPUT
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  Serial.begin(9600); // For debugging
}

void loop() {
  // Read analog value from A0 (0 to 1023)
  int sensorValue = analogRead(A0);
  Serial.println(sensorValue); // Print the value to Serial Monitor

  // Map analog value (0-1023) to 0-255 for PWM brightness
  int pwmValue = map(sensorValue, 0, 1023, 0, 255);

  // Logic: Turn on more LEDs as value increases
  for (int i = 0; i < numLeds; i++) {
    if (sensorValue > (i * 200)) {
      analogWrite(ledPins[i], pwmValue); // Glow LED with PWM brightness
    } else {
      analogWrite(ledPins[i], 0); // Turn off LED
    }
  }

  delay(100); // Small delay to avoid flickering
}
```

---

### ✅ **Explanation:**

* `analogRead(A0)`: Reads value from 0 to 1023 from sensor or potentiometer.
* `map(sensorValue, 0, 1023, 0, 255)`: Converts that value into PWM range (0–255).
* Based on the value:

  * LED1 turns ON if value > 0
  * LED2 turns ON if value > 200
  * LED3 turns ON if value > 400
  * LED4 turns ON if value > 600
  * LED5 turns ON if value > 800
* Each turned-on LED glows with the same PWM brightness, creating a **dynamic glowing effect**.

---

### ✅ **Use Case in IoT:**

* Ambient light-based smart lighting.
* Sensor-based fan/LED speed control.
* Health monitoring or battery level indicator with LED bars.
* Any **level-based output system** in your IoT projects.

---
---

## ✅ **Project: RGB LED Control Using Arduino**

### 🎯 **Goal:**

Control an RGB LED to display different colors using **PWM signals** on Arduino.

---

### 🔹 **What is an RGB LED?**

An **RGB LED** combines three LEDs (Red, Green, Blue) in one package. By changing the brightness of each LED using **PWM**, we can create **any color**.

#### Types of RGB LEDs:

* **Common Cathode** – All cathodes (–) are connected together.
* **Common Anode** – All anodes (+) are connected together.

👉 We’ll use **Common Cathode** here (standard type for beginners).

---

### ✅ **Hardware Required:**

* 1 × Common Cathode RGB LED
* 3 × 220Ω resistors
* Arduino Uno
* Breadboard & jumper wires

---

### ✅ **Pin Connection:**

| RGB LED Pin | Color | Connect To       |
| ----------- | ----- | ---------------- |
| 1 (longest) | GND   | GND via 220Ω x 3 |
| R           | Red   | Arduino Pin 9    |
| G           | Green | Arduino Pin 10   |
| B           | Blue  | Arduino Pin 11   |

> ⚠️ Use **220Ω resistors** for each R, G, B pin in series to limit current.

---

### ✅ **Arduino Code: RGB Color Mixing**

```c
// Define RGB LED pins
int redPin = 9;
int greenPin = 10;
int bluePin = 11;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  // Display RED
  setColor(255, 0, 0);
  delay(1000);

  // Display GREEN
  setColor(0, 255, 0);
  delay(1000);

  // Display BLUE
  setColor(0, 0, 255);
  delay(1000);

  // Display YELLOW (Red + Green)
  setColor(255, 255, 0);
  delay(1000);

  // Display CYAN (Green + Blue)
  setColor(0, 255, 255);
  delay(1000);

  // Display MAGENTA (Red + Blue)
  setColor(255, 0, 255);
  delay(1000);

  // Display WHITE (All colors)
  setColor(255, 255, 255);
  delay(1000);

  // Display OFF
  setColor(0, 0, 0);
  delay(1000);
}

// Function to set RGB color
void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}
```

---

### ✅ **Explanation:**

* `analogWrite(pin, value)` sends a PWM signal to control LED brightness.
* Values range from `0` (OFF) to `255` (FULL brightness).
* By mixing different PWM values, you can generate **millions of colors**.

---

### 🔹 **Practice: Custom Color Using Potentiometers**

You can connect 3 potentiometers to A0, A1, A2 and use them to adjust RGB values in real-time. Great hands-on IoT exercise!

---

### ✅ **Applications in IoT**

* Smart home RGB lighting
* Notification indicators (color-based alerts)
* Mood lights or status indicators in embedded systems

---
---

## ✅ **map() Function in Arduino – Full Theory + Examples**

### 🔹 **Purpose of `map()`**

The `map()` function is used to **re-map a number from one range to another**.

### 🧠 Why it's useful in IoT:

Sensors often give values in one range (like `0–1023` from analogRead), but you might need it in a different range (like `0–255` for PWM, or `0–100` for percentage).

---

### 🔹 **Syntax**

```cpp
map(value, fromLow, fromHigh, toLow, toHigh)
```

### 🔸 Parameters:

| Parameter  | Description                     |
| ---------- | ------------------------------- |
| `value`    | The input value to be mapped    |
| `fromLow`  | Lower bound of the input range  |
| `fromHigh` | Upper bound of the input range  |
| `toLow`    | Lower bound of the target range |
| `toHigh`   | Upper bound of the target range |

---

### 🔹 **Example 1: Map Analog to PWM**

```cpp
int sensorValue = analogRead(A0);  // 0 to 1023
int pwmValue = map(sensorValue, 0, 1023, 0, 255);
analogWrite(9, pwmValue);          // Use PWM to control brightness
```

* `analogRead(A0)` → gives value from 0–1023
* `analogWrite()` → requires value from 0–255
* So we use `map()` to **convert 0–1023 to 0–255**

---

### 🔹 **Example 2: Display Sensor Reading as Percentage**

```cpp
int val = analogRead(A0); // 0 to 1023
int percent = map(val, 0, 1023, 0, 100);
Serial.print("Sensor %: ");
Serial.println(percent);
```

---

### 🔹 **Behind the Scenes: Formula Used**

```cpp
mappedValue = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
```

For example:

```cpp
map(512, 0, 1023, 0, 255)
= (512 - 0) * (255 - 0) / (1023 - 0) + 0
= 128 (approximately)
```

---

### 🔹 **Important Notes**

* The result is always an **integer** (no decimal).
* The `map()` function **does not constrain** the result. If `value` goes outside the input range, the result may go outside the output range.

#### ✅ To prevent this:

Use `constrain()`:

```cpp
val = constrain(val, 0, 1023);
mapped = map(val, 0, 1023, 0, 255);
```

---

### 🔹 **Real IoT Use Cases**

| Application           | Use of map()                              |
| --------------------- | ----------------------------------------- |
| Light sensors (LDR)   | Map brightness to % or LED intensity      |
| Temperature sensors   | Map analog signal to Celsius/Fahrenheit   |
| Potentiometer + motor | Control motor speed using PWM             |
| Servo control         | Map sensor value to servo angle (0–180°)  |
| Display               | Convert sensor value to % or progress bar |

---

### ✅ **Assignment for Students**

Try this:

* Connect a potentiometer to A0
* Map its value to:

  * 0–255 for LED brightness
  * 0–180 for servo motor
  * 0–100 for serial display percentage


