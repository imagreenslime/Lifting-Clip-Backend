# Lifting Clip Firmware
Firmware for my barbell "lifting clip" project. The goal is to track rep count, bar velocity, acceleration, and range of motion, then stream that data over Bluetooth to a phone app.

## Hardware (current prototype)
- ESP32-S3 WROOM
- MPU6050 IMU
- VL53L1X ToF sensor
- Breadboard + jumper wires for now
- Power over USB-C (working on a custom PCB next)

## What works right now
- Basic firmware setup on ESP32-S3 (PlatformIO)
- Reading raw values from the IMU and ToF sensor
- Sending test data over BLE to a phone
- Serial output for quick debugging/logging

## What's in progress
- Cleaning up sensor drivers
- Adding filtering (probably a simple moving average or complementary filter)
- Packaging data into a BLE characteristic
- Designing a PCB in KiCad to replace the breadboard setup

## How to build/run
- Clone this repo
- Install [PlatformIO](https://platformio.org/)
- Plug in ESP32-S3 board
- Run `pio run -t upload` to flash
- Run `pio device monitor` to see serial output

## Repo layout
- `src/` – main firmware (currently `main.cpp`)
- `include/` – headers/config
- `lib/` – sensor drivers or external libs
- `test/` – future unit tests
- `hardware/` – PCB design files (WIP)
- `docs/` – wiring diagram, notes, and screenshots

## Demo
- Breadboard prototype photo:
- Short video demo (bar movement + serial output):
- Screenshot of app receiving BLE packets:

## Roadmap
- Finish basic filtering
- Clean BLE service with real metrics
- Add OTA updates + config storage
- Fabricate first PCB and do bring-up testing
