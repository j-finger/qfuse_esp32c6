# ESP32C6 Daughter Board for the `qfuse` Project

This project provides UART, MQTT, and Wi-Fi functionalities for the RP2040 within the `qfuse` project. It is designed to run on the ESP32C6 microcontroller, specifically the Seeed ESP32C6 Xiao Mini, serving as a daughter board for the RP2040.

## Table of Contents
- [Features](#features)
- [Components](#components)
- [Hardware](#hardware)
- [Software](#software)
- [Libraries](#libraries)
- [System Overview](#system-overview)
- [Firmware Details](#firmware-details)
  - [File Structure](#file-structure)
  - [Wi-Fi Connectivity](#wifi-connectivity)
  - [MQTT Communication](#mqtt-communication)
  - [Time Synchronization](#time-synchronization)
  - [UART Communication](#uart-communication)
  - [JSON Handling](#json-handling)
  - [LED Status Indicator](#led-status-indicator)
- [Setup & Usage](#setup--usage)
- [Dependencies](#dependencies)
- [License](#license)

## Features

- **Wi-Fi Connectivity**: Connects to a Wi-Fi network using credentials defined in `config.h`.
- **MQTT Communication**: Publishes and subscribes to MQTT topics for data exchange.
- **UART Communication**: Handles UART data transmission and reception.
- **Time Synchronization**: Synchronizes system time using SNTP.
- **JSON Handling**: Utilizes JSON for structured data exchange.
- **LED Status Indicator**: Provides visual feedback on system status through an onboard LED.

## Components

- **Wi-Fi Initialization**: Sets up Wi-Fi in station mode and handles connection events.
- **MQTT Client**: Manages MQTT connections, subscriptions, and message handling.
- **UART Interface**: Initializes UART, sends, and receives data.
- **Time Management**: Obtains the current time from an NTP server.
- **JSON Operations**: Appends timestamps to incoming JSON objects from the RP2040 before transmitting them to the MQTT broker.

## Hardware

- **Board**: Seeed Xiao Mini ESP32C6
  - **UART Communication**: Connected via UART to an RP2040 for receiving JSON data.
  - **Wi-Fi Connectivity**: Communicates with a Raspberry Pi 4 server over Wi-Fi using MQTT for data transmission.
  - **LED**: Connected to GPIO 15 for status indication.

## Software

- **Framework**: ESP-IDF v5.3.1
- **IDE**: Visual Studio Code with PlatformIO
- **Programming Language**: C++

## Libraries

- **ESP-IDF**: Core framework providing drivers and network functionality.
- **nlohmann/json**: JSON parsing library for handling JSON data structures received via UART and transmitted via MQTT.
- **FreeRTOS**: Real-time operating system for task management.
- **MQTT Client**: For managing MQTT connections and communication.

## System Overview

The ESP32C6 acts as a bridge between the RP2040 and the Raspberry Pi 4 server:
1. **Receiving Data**: Receives JSON-formatted data from the RP2040 over UART.
2. **Time Synchronization**: Synchronizes the current time using NTP and appends a timestamp to the JSON data.
3. **Data Transmission**: Sends the time-stamped JSON data to the Raspberry Pi 4 server via MQTT over Wi-Fi.

## Firmware Details

### File Structure
- **`main.cpp`**: Contains the main firmware code handling Wi-Fi, MQTT, UART, time synchronization, and JSON processing.
- **`config.h`**: Stores configuration details such as Wi-Fi SSID and password.
- **`json.hpp`**: Header file for the `nlohmann::json` library used for JSON parsing and manipulation.

### Wi-Fi Connectivity
- **Initialization**: Connects to a Wi-Fi network using WPA2-PSK credentials from `config.h`.
- **Event Handling**: Monitors connection events, retries on failure (up to 5 attempts), and logs the IP address upon successful connection.
- **IP Address Wait Loop**: Ensures the system waits until a valid IP address is obtained before proceeding.

### MQTT Communication
- **Client Setup**: Configures the MQTT client with the broker URI defined in `MQTT_URI` (default: `mqtt://192.168.86.20:1883`).
- **Subscriptions**: Subscribes to `sensor/settings`, `sensor/data`, and `sensor/logs` topics.
- **Publishing**: Sends JSON-formatted data with appended timestamps to MQTT topics.
- **Error Handling**: Logs detailed error information for troubleshooting MQTT connection issues.

### Time Synchronization
- **NTP Synchronization**: Uses SNTP to sync time with `pool.ntp.org`.
- **Timezone Setting**: Sets the timezone to AEST (UTC+10).
- **Retry Mechanism**: Attempts to obtain the system time up to 10 times, waiting 2 seconds between attempts.

### UART Communication
- **Configuration**: Initializes UART1 with a baud rate of 921600, using GPIO 16 (TX) and GPIO 17 (RX).
- **Data Handling**: Receives JSON data from the RP2040, buffering incoming bytes until a newline character (`\n`) is detected, then processes the complete JSON message.
- **Event Handling**: Utilizes FreeRTOS queues and tasks to manage UART events and JSON processing.

### JSON Handling
- **Parsing & Validation**: Uses `nlohmann::json` to parse incoming JSON strings received via UART. Validates JSON format before processing.
- **Timestamping**: Appends the current UNIX epoch time to each JSON object before publishing to MQTT.

### LED Status Indicator
- **Initialization**: Configures GPIO 15 as an output pin for the onboard LED.
- **Blinking**: The LED blinks after each major operation (Wi-Fi, MQTT, UART setup) to indicate progress and status.

## Setup & Usage

1. **Configure Wi-Fi Credentials**
   - Edit `config.h` to include your Wi-Fi SSID and password.

2. **Build and Upload Firmware**
   - Use PlatformIO in Visual Studio Code to compile and upload the firmware to the ESP32C6 board.

3. **Monitor Output**
   - Use a serial monitor to observe status messages and data logs from the ESP32C6.

4. **Operation**
   - Upon startup, the ESP32C6 will:
     - Initialize the LED and blink to indicate progress.
     - Connect to the specified Wi-Fi network.
     - Synchronize time using NTP.
     - Establish an MQTT connection and subscribe to relevant topics.
     - Initialize UART communication to receive JSON data from the RP2040.
     - Process incoming JSON data, append timestamps, and publish to MQTT topics.

## Dependencies

- **PlatformIO**: For building and uploading firmware.
- **ESP-IDF**: Development framework for ESP32C6.
- **FreeRTOS**: Real-time operating system for task management.
- **nlohmann/json**: JSON parsing library.
- **MQTT Client**: For MQTT communication.
- **Visual Studio Code**: Recommended IDE with PlatformIO extension.

## License

This project is licensed under the MIT License.
