# ESP32C6 Daughter Board - Part of the `qfuse` Project

This project contains the code that provides UART, MQTT and Wi-Fi functionality for the RP2040 within the `qfuse` project. It is intended to operate on an ESP32C6 microcontroller, in particular the Seeed ESP32C6 Xaio Mini, acting as a daughter board for the RP2040. 

It has been developed with PlatformIO and ESP-IDF. The main functionalities include:

## Features

- **Wi-Fi Connectivity**: Connects to a Wi-Fi network using credentials defined in `config.h`.
- **MQTT Communication**: Publishes and subscribes to MQTT topics for data exchange.
- **UART Communication**: Handles UART data transmission and reception.
- **Time Synchronization**: Synchronizes system time using SNTP.
- **JSON Handling**: Utilizes JSON for structured data exchange.

## Components

- **Wi-Fi Initialization**: Sets up Wi-Fi in station mode and handles connection events.
- **MQTT Client**: Manages MQTT connections, subscriptions, and message handling.
- **UART Interface**: Initializes UART, sends, and receives data.
- **Time Management**: Obtains the current time from an NTP server.
- **JSON Operations**: Appends the time to incoming JSON objects fromt the RP2040 before transmission to the MQTT broker.

## Usage

1. Configure Wi-Fi credentials in `config.h`.
2. Compile and upload the code to the ESP32C6 using PlatformIO.
3. Monitor the serial output for status messages and data logs.

## Dependencies

- PlatformIO
- FreeRTOS
- ESP-IDF
- nlohmann/json

## License

This project is licensed under the MIT License.