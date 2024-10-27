# ESP32C6 Firmware for `qfuse` Data Transmission System

## Overview

This repository contains the firmware for the ESP32C6 module used in the `qfuse` development board. The ESP32C6 acts as the communication bridge between the RP2040 microcontroller and the backend server. It receives serialized sensor data over UART from the RP2040, appends timestamps, and transmits the data over Wi-Fi using the MQTT protocol to a backend server for further processing and storage.

**System Overview**

The ESP32C6 acts as a bridge between the RP2040 and the Raspberry Pi 4 server:
1. **Receiving Data**: Receives JSON-formatted data from the RP2040 over UART.
2. **Time Synchronization**: Synchronizes the current time using NTP and appends a timestamp to the JSON data.
3. **Data Transmission**: Sends the time-stamped JSON data to the Raspberry Pi 4 server via MQTT over Wi-Fi.

## Features

- **UART Communication**: Receives serialized JSON data from the RP2040 over UART at a baud rate of 921600.
- **Wi-Fi Connectivity**: Connects to a Wi-Fi network in Station Mode to transmit data to the backend server.
- **MQTT Protocol**: Publishes sensor data, settings, and logs to specific MQTT topics.
- **Time Synchronization**: Requests the current UNIX epoch time from the server and appends it to outgoing data packets.
- **JSON Data Handling**: Parses incoming JSON data and appends necessary metadata before transmission.
- **Error Handling**: Implements robust error handling for MQTT and UART communication.
- **LED Indicators**: Uses GPIO pins to control an LED for status indication.

## Hardware Setup

### Components

- **ESP32C6 Module**: Seeed Xiao Mini ESP32C6 - Communicates with a Raspberry Pi 4 backend server over Wi-Fi using MQTT for data transmission.
	- **LED**: Connected to GPIO 15 for status indication.
- **RP2040 Microcontroller**: Sends serialized sensor data to the ESP32C6 over UART.
- **`qfuse` Development Board**: Custom PCB integrating the RP2040 and ESP32C6.
- **UART Connection**:
  - **ESP32C6 UART Pins**:
    - RXD: GPIO 17 (Receive data from RP2040)
    - TXD: GPIO 16 (Transmit data to RP2040)
  - **Baud Rate**: 921600

### Pin Assignments

- **UART**:
  - **TXD**: GPIO 16
  - **RXD**: GPIO 17
- **LED Indicator**:
  - **LED_PIN**: GPIO 15 (Used to indicate status by blinking)

### Network Configuration

- **Wi-Fi Credentials**:
  - **SSID**: Configured in `config.h`
  - **Password**: Configured in `config.h`
- **MQTT Broker**:
  - **URI**: Defined in `main.cpp` (Default: `mqtt://192.168.4.1:1883`)

## Software Dependencies

- **Visual Studio Code**: Recommended IDE with PlatformIO extension.
- **PlatformIO**: For building and uploading firmware.
- **ESP-IDF**: Espressif IoT Development Framework (version 4.4 or higher recommended).
- **CMake**: Version 3.16 or higher.
- **Python 3.x**: Required for ESP-IDF build system.
- **nlohmann/json**: JSON library for C++ (header-only, included if necessary).
- **Standard C++ Libraries**: Requires C++17 support.
- **FreeRTOS**: Real-time operating system for task management.
- **MQTT Client**: For MQTT communication.
## Directory Structure

- `main/`: Contains the main application source code.
  - `main.cpp`: Entry point of the firmware.
  - `wifi.cpp` and `wifi.h`: Wi-Fi initialization and handling.
  - `config.h`: Wi-Fi credentials and configuration (should be created from `config.h.example`).
- `CMakeLists.txt`: CMake build configuration file for the ESP-IDF project.
- `components/`: Custom components (if any).
- `sdkconfig`: ESP-IDF configuration file (generated during build).

## Build Instructions

### Prerequisites

1. **Install ESP-IDF**:
   - Follow the instructions at [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/index.html) to set up the ESP-IDF environment for the ESP32C6.
   - Ensure that the `IDF_PATH` environment variable is set to the location of the ESP-IDF.
2. **Install CMake**:
   - Version 3.16 or higher is required.
3. **Install Python 3.x**:
   - Required for the ESP-IDF build system and tools.

### Building the Firmware

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/qfuse-esp32c6-firmware.git
   cd qfuse-esp32c6-firmware
   ```
2. **Set Up Wi-Fi Credentials**:
   - Copy `config.h.example` to `config.h`:
     ```bash
     cp main/config.h.example main/config.h
     ```
   - Edit `main/config.h` to include your Wi-Fi SSID and password:
     ```c
     #define WIFI_SSID "YourWiFiSSID"
     #define WIFI_PASS "YourWiFiPassword"
     ```
3. **Configure the Project**:
   - Run the menu configuration tool:
     ```bash
     idf.py menuconfig
     ```
   - In the menu, you can set additional configurations like UART pins, baud rate, and other settings if needed.
4. **Build the Firmware**:
   ```bash
   idf.py build
   ```
   - This will compile the firmware and generate a `.bin` file.
5. **Flash the Firmware**:
   - Connect the ESP32C6 module to your computer via USB.
   - Flash the firmware:
     ```bash
     idf.py -p /dev/ttyUSB0 flash
     ```
     - Replace `/dev/ttyUSB0` with the appropriate serial port.
6. **Monitor the Output**:
   - To monitor the serial output:
     ```bash
     idf.py -p /dev/ttyUSB0 monitor
     ```

## Usage Instructions

Upon startup, the ESP32C6 will:
 - Initialize the LED and blink to indicate progress.
 - Connect to the specified Wi-Fi network.
 - Synchronize time using NTP.
 - Establish an MQTT connection and subscribe to relevant topics.
 - Initialize UART communication to receive JSON data from the RP2040.
 - Process incoming JSON data, append timestamps, and publish to MQTT topics.

1. **Power On the System**:
   - Ensure that the `qfuse` board is powered and the ESP32C6 module is connected to the RP2040 via UART.
2. **Establish Wi-Fi Connection**:
   - On startup, the ESP32C6 will attempt to connect to the configured Wi-Fi network.
   - The status LED will blink during initialization.
3. **MQTT Communication**:
   - The module will connect to the MQTT broker specified in `main.cpp` (default is `mqtt://192.168.4.1:1883`).
   - It subscribes to the `time/response` topic to receive the current time.
4. **Data Reception and Transmission**:
   - The ESP32C6 receives JSON data from the RP2040 over UART.
   - It appends the current UNIX epoch time to the data.
   - Publishes the data to the MQTT topic `sensor/data`.
5. **Time Synchronization**:
   - Upon connecting to the MQTT broker, the ESP32C6 publishes an empty message to `time/request`.
   - It waits for a response on `time/response` to synchronize its clock.
6. **LED Indicator**:
   - The LED connected to `GPIO 15` provides visual status:
     - **Blinking**: Initialization and setup phases.
     - **On**: Normal operation after setup is complete.

## Code Organization

### `main.cpp`

- **MQTT Handling**:
  - Initializes the MQTT client and handles events such as connection, disconnection, and message reception.
  - Subscribes to necessary topics and publishes data.
- **UART Handling**:
  - Configures UART parameters and sets up event queues.
  - Receives data over UART, parses JSON, and enqueues it for processing.
- **JSON Processing**:
  - Processes incoming JSON data, appends timestamps, and publishes it to the MQTT broker.
- **Time Synchronization**:
  - Requests current time from the server and sets the system time upon receiving it.
- **LED Control**:
  - Initializes and controls the status LED for visual feedback.

### `wifi.cpp` and `wifi.h`
- **Wi-Fi Initialization**:
  - Configures the ESP32C6 as a Wi-Fi station.
  - Connects to the specified Wi-Fi network using credentials from `config.h`.
- **Event Handling**:
  - Handles Wi-Fi events such as connection, disconnection, and obtaining an IP address.

### `config.h` and `config.h.example`
- Contains Wi-Fi credentials and configuration settings.

### `CMakeLists.txt`
- Configures the build process using ESP-IDF's CMake build system.
- Specifies source files and includes directories.

## Configuration

- **Wi-Fi Credentials**:
  - Set in `config.h` by defining `WIFI_SSID` and `WIFI_PASS`.
- **MQTT Broker URI**:
  - Defined in `main.cpp` with the `MQTT_URI` macro.
  - Adjust the URI to point to your MQTT broker's address.
- **UART Configuration**:
  - UART parameters such as baud rate and GPIO pins are defined in `main.cpp`.
  - Adjust `BAUD_RATE`, `TXD_PIN`, and `RXD_PIN` as needed.
- **Buffer Sizes**:
  - MQTT buffer sizes can be adjusted via `MQTT_BUFF_IN_SIZE`, `MQTT_BUFF_OUT_SIZE`, and `MQTT_OUTBOX_SIZE`.
- **LED Pin**:
  - The status LED is connected to `GPIO 15`. Change `LED_PIN` in `main.cpp` if using a different pin.


## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- **Espressif Systems**: For providing the ESP-IDF and ESP32C6 hardware.
- **nlohmann/json**: For the JSON serialization library.
- **FreeRTOS**: For the real-time operating system used in ESP-IDF.
- **MQTT Community**: For resources and documentation on MQTT communication.

