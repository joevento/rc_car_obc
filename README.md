# ESP32 RC Car OBC with LiDAR and nRF24

This repository contains the firmware for an ESP32-based on-board computer (OBC) for an RC car. It integrates:
- RPLIDAR-compatible LiDAR over UART
- Dual DC motor control via PWM and H-bridges
- nRF24L01+ radio in TX mode with ACK payloads for bidirectional command/control
- FreeRTOS tasks and queues for decoupled LiDAR acquisition and RF framing

It’s built for an electronics workshop course project and aims to be simple, robust, and easy to extend.

## Features

- LiDAR
  - Initializes UART and motorized LiDAR spin (PWM) and starts scanning
  - Synchronizes on the RPLIDAR scan descriptor and reads 5-byte scan packets
  - Packs full scans into a queue for downstream transmission
- Motor control
  - Four DC motors using tank steering (side A and B) with direction pins and LEDC PWM speed control
  - Simple API: `motor_init`, `motorA_control`, `motorB_control`
- nRF24L01+ Radio
  - TX-only mode with dynamic payloads and ACK payloads enabled
  - Sends fragmented LiDAR frames over 32-byte payloads
  - Receives motor commands from ACK payloads and applies them on the fly
- FreeRTOS-based
  - Dedicated LiDAR task reading and queuing scans
  - Main loop fragments queued scans and transmits them via nRF24
  - Small, deterministic delays to keep the pipeline smooth

## Repository Structure

- Lidar
  - `lidar.h`, `lidar.c`: UART + PWM setup, scanning start, scan data acquisition
- Motors
  - `motor.h`, `motor.c`: LEDC PWM setup, H-bridge direction control
- Radio
  - `nrf24_comm.h`, `nrf24_comm.c`: High-level radio API, ACK payload hook
  - `rf24_wrapper.h`, `rf24_wrapper.cpp`: C wrapper around the RF24 C++ library
- App
  - `main.c`: System init, LiDAR task, RF fragmentation/transmission, ACK motor control
- Third-party
  - `components/rf24/`: RF24 library (C++)

## Hardware Overview

- MCU: ESP32
- Radio: nRF24L01+ (CE on GPIO 21, CSN on GPIO 5 by default)
- LiDAR: RPLIDAR-compatible UART LiDAR with motor PWM control
- Motor driver: Dual H-bridge (pins defined below)

### Pin Assignments

- LiDAR
  - UART TX: GPIO 17
  - UART RX: GPIO 16
  - Motor PWM (LiDAR spin): GPIO 13 (LEDC High-Speed, 5 kHz, 10-bit)

- Motors
  - Motor A: AIN1 GPIO 12, AIN2 GPIO 14, PWM GPIO 32 (LEDC Low-Speed)
  - Motor B: BIN1 GPIO 26, BIN2 GPIO 25, PWM GPIO 33 (LEDC Low-Speed)
  - STBY (driver enable): GPIO 27
  - PWM config: 30 kHz, 8-bit resolution

- nRF24L01+
  - CE: GPIO 21
  - CSN: GPIO 5
  - Channel: 76
  - Data rate: 2 Mbps
  - Power: PA_MAX
  - Retries: delay 5, count 15
  - Payload size: 32 bytes, dynamic payloads enabled

Remember to adjust pins and parameters in the headers/source if your hardware differs.

## Build and Flash

- ESP-IDF recommended (v5.x works well)
- Place the RF24 library in `components/rf24/` (already referenced by the wrapper)
- Typical steps:
  - idf.py set-target esp32
  - idf.py menuconfig (set UART pins, logging, etc., if needed)
  - idf.py build
  - idf.py flash monitor

Ensure power integrity for the nRF24L01+ (consider a decoupling capacitor on VCC) and motor driver.

## How It Works

1. Init sequence (app_main):
   - `motor_init()`
   - `nrf24_init()`
   - `lidar_init_handler()` sets up a queue and starts `lidar_scan_task`

2. LiDAR:
   - `lidar_init()` sets up UART, LEDC PWM for LiDAR spin motor, ramps to 100 % duty, waits 2 seconds to stabilize, and sends the scan command 0xA5 0x20
   - Waits for a 7-byte response descriptor and enters scanning mode
   - `get_lidar_scan_data()` reads 5-byte packets, aligns on start-of-scan flag, collects one full revolution into a buffer

3. Framing and radio TX:
   - Main loop drains the LiDAR queue
   - Splits scan bytes into 24-byte payload chunks inside `rf_frame_t`:
     - Header: magic 0xA7, version 0x01, scan_id, fragment_id, flags (bit0 is_last), payload_len
     - CRC16-CCITT of header-with-zeroed-crc + payload
   - Each frame is sent via `nrf24_send()`

4. Motor commands from ACK:
   - If RX side sends ACK payloads with a packed `motor_command_t`, the TX side will read them after each write and invoke `nrf24_on_ack_payload()`
   - The hook applies `motorA_control()` and `motorB_control()` immediately

## Frame Format (RF)

- `rf_frame_t` (packed):
  - magic: 0xA7
  - version: 0x01
  - scan_id: increments per full scan
  - fragment_id: increments per fragment
  - flags: bit0 = is_last
  - payload_len: 0..24
  - crc: CRC16-CCITT over header (crc field zeroed) + payload
  - payload[24]

This is designed to fit within 32-byte nRF24 payloads while leaving room for header + CRC.

## Motor Control

- API:
  - `motor_init()`
  - `motorA_control(int speed, bool direction)`
  - `motorB_control(int speed, bool direction)`

- Speed is raw duty count for 8-bit resolution (0..255). The example uses values like 150/300; adjust to your LEDC resolution. If you keep 8-bit, cap at 255.

- Direction:
  - true: forward
  - false: reverse

## LiDAR Notes

- UART is configured at 115200 8N1
- The code expects the LiDAR to respond with a descriptor after the 0xA5 0x20 command
- Spin motor is PWM-driven; 2-second wait ensures stable RPM before starting scan (simple but effective)
- Scan packets are 5 bytes; first bit indicates start of scan for a full revolution

If you use a different LiDAR model, verify the command and descriptor format.

## Radio Notes

- TX-only node with dynamic payloads and ACK payloads enabled
- Set the RX base to mirror:
  - Same channel, data rate, addressing, dynamic payload settings
  - Send ACK payloads with `motor_command_t` to control motors remotely
- Address in this repo:
  - TX writing pipe: "00001" (5 bytes). Adjust to your network.

## Configuration and Debug

- Define `DEBUG` at compile time for additional logs:
  - More verbose init logs (UART, PWM, radio details)
  - LiDAR scan acquisition info
  - RF24 `printDetails()` dump
- Logging uses ESP-IDF `ESP_LOGx` macros

## Common Issues and Tips

- nRF24 power/brownouts:
  - Use proper decoupling (e.g., 10 µF + 100 nF at the module)
  - Keep SPI wiring short; avoid breadboard jumpers if possible

- Motor driver standby:
  - STBY must be high to enable the driver; this code sets it high at init

- LEDC resolution vs. speed values:
  - Motor code assumes 8-bit resolution; keep speed in 0..255
  - LiDAR spin uses 10-bit resolution on a different LEDC unit

- LiDAR start timing:
  - If the LiDAR doesn’t start scanning, ensure the motor reaches stable RPM
  - Check UART pins and baud rate

## Extending

- Add a receiver node firmware to:
  - Reassemble LiDAR frames via scan_id/fragment_id
  - Validate CRC16
  - Visualize point cloud or forward to a host via USB/serial
  - Populate ACK payloads with `motor_command_t`

- Add closed-loop RPM control for the LiDAR motor using a tach input if available

- Add CLI commands via UART or ESP-IDF console for runtime configuration

## License

MIT. Use it, modify it, have fun, and contribute improvements.

## Acknowledgments

- RF24 library by TMRh20 and contributors
- ESP-IDF by Espressif
- RPLIDAR protocol inspiration from Slamtec devices (verify your model’s protocol)
