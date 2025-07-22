# Smart Door Lock

## Overview
This project implements a smart door lock system using an STM32 microcontroller, RFID reader, OLED display, SG90 servo motor, and ESP8266 Wi-Fi module for SMS notifications via Twilio.

## Demo Video
[Watch demo video on YouTube](https://youtube.com/shorts/pmf_5Fu0lJE)

## Features
- **RFID-Based Access Control**: Authenticate users using RC522 reader over SPI.
- **Three Operation Modes**: Access mode, Enroll mode, Remove mode.
- **Real-Time Feedback**: 0.91" OLED display shows mode and status.
- **Servo Lock Actuation**: SG90 servo motor driven by PWM to lock/unlock door.
- **SMS Alerts**: ESP8266 sends status messages via Twilio API.

## Operation Modes
| Mode     | Interaction        | Description                         |
|----------|--------------------|-------------------------------------|
| Access   |Press and hold (>2s)| Returns to default access mode      |
| Enroll   | Single  Click      | Registers a new RFID card UID       |
| Remove   | Double Click       | Deletes an existing RFID card UID   |

## Hardware Components
| Component          | Quantity | Description                         |
| ------------------ | -------- | ----------------------------------- |
| STM32F411          | 1        | Microcontroller Unit                |
| ESP8266 NodeMCU    | 1        | Microcontroller Unit                |
| RFID RC522         | 1        | RFID reader                         |
| SG90 Servo Motor   | 1        | Door open                           |
| 0.91" OLED Display | 1        | Text display                        |
| Push Button        | 1        | Mode switching                      |
| Connecting Wires   | 19       | Jumper cables                       |

## Wiring Overview
1. **RFID RC522 (SPI)**
    - VCC   → 3V
    - GND   → GND
    - SCK   → PB3  
    - MISO  → PB4 
    - MOSI  → PB5  
    - SDA (CS) → PB6  
    - RST   → PB7 

2. **SG90 Servo Motor (PWM)**

    - VCC → 5V
    - GND → GND
    - PWM → PB10

3. **0.91" OLED (I2C)**
    - VCC → 3V
    - GND → GND
    - SCL → PA8
    - SDA → PC9

4. **Push Button (GPIO & EXTI)**
    - VCC → 3V
    - GND → GND
    - OUT → PB1

5. **ESP8266 (UART)**
    - ESP8266 RX → STM32 TX (PC6)
    - GND → GND

> **Note**: Ensure common ground between all devices.