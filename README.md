# Luggage-Classification-Conveyor

This repository contains the firmware for an automated luggage sorting conveyor belt system powered by an ESP32 microcontroller. The project uses the FreeRTOS real-time operating system to manage multiple sensors and actuators concurrently, creating a robust, event-driven sorting solution.



## Overview

The system is designed to sort objects (referred to as "luggage") based on a classification received via Bluetooth Low Energy (BLE) and subsequent sensor readings. A central state machine controls the entire process, from detecting an object's arrival to its final sorting destination.

The workflow is as follows:
1.  An object is placed on the conveyor and detected by an IR sensor.
2.  The conveyor belt starts moving the object.
3.  A remote device (e.g., an ESP32-CAM) classifies the object as "light" or "heavy" and sends the result via BLE.
4.  The conveyor has two paths: one for "light" luggage and another for "heavy" luggage. A servo motor directs the object to the correct path.
5.  Along the path, a magnetic sensor checks for prohibited items. If a magnetic field is detected, the object is immediately re-routed to be discarded.
6.  "Heavy" or magnetic items are stopped on a load cell to be weighed.
7.  Based on the weight, a final sorting gate either accepts or discards the item.

## Features

* **Real-Time Control**: Built on FreeRTOS for responsive, parallel task management.
* **State Machine Architecture**: A robust state machine manages system logic, making the code organized and scalable.
* **Multi-Core Processing**: Utilizes both cores of the ESP32; Core 0 is dedicated to BLE communication, while Core 1 handles the main application logic.
* **BLE Communication**: Acts as a BLE client to receive commands and classification data from a remote server.
* **Multi-Sensor Integration**:
    * **IR Sensor**: Detects when an object enters the conveyor.
    * **Magnetic Hall Sensor**: Detects potentially prohibited items.
    * **Load Cell**: Measures the weight of objects for final sorting.
* **Dual-Servo Sorting**: Uses two servo motors for a two-stage sorting process (classification path and final accept/discard).
* **Error Handling**: Includes logic to handle BLE disconnections and object overlap (emergency stop).

***

## Repository Structure

This project is organized into several repositories. This main repository contains the firmware, while the hardware designs and supplementary tools are included as Git submodules.

* **`/` (root)**: Contains the main ESP32 firmware for the conveyor system.
* **`Conveyor-Hardware-Design`**: Contains all mechanical design files.
    * SolidWorks models
    * STL files for 3D printing
    * DXF files for manufacturing
* **`Conveyor-Electronic-Design`**: Contains the electronic design files for the custom PCB.
    * KiCad project (schematic and PCB layout)
    * Gerber files for PCB fabrication
* **`Conveyor-Tools`**: Includes PC-based software tools for:
    * Sensor calibration
    * System diagnostics
    * Initial setup

To clone this repository and all its submodules, use the `--recurse-submodules` flag:
```sh
git clone --recurse-submodules <your-repository-url>
```
If you have already cloned the repository, you can initialize the submodules with:
```sh
git submodule update --init --recursive
```

***

## Hardware Requirements

| Component              | Description                                     |
| ---------------------- | ----------------------------------------------- |
| **Microcontroller** | ESP32-WROOM-32 or similar                       |
| **Motor & Driver** | 1x DC Motor + L293D Motor Driver IC             |
| **Servos** | 2x SG90 or similar servo motors                 |
| **Object Detection** | 1x E18-D80NK IR Proximity Sensor                |
| **Weighing** | 1x Load Cell + HX711 Amplifier Module           |
| **Magnetic Detection** | 1x KY-024 Linear Magnetic Hall Sensor           |
| **Status Indicator** | On-board LED or external LED                    |
| **Power Supply** | 5V DC supply for motors and ESP32 |

***

## Pinout

The following table details the GPIO connections used in the firmware.

| Pin | Component            | Connection           |
| --- | -------------------- | -------------------- |
| 2   | On-board LED         | `LED_BUILTIN`        |
| 16  | L293D Motor Driver   | `L293D_1A_PIN`       |
| 17  | L293D Motor Driver   | `L293D_2A_PIN`       |
| 19  | HX711 Load Cell      | `HX711_SCK_PIN`      |
| 21  | Servo 1 (Classifier) | `SERVO1_PIN`         |
| 22  | Servo 2 (Sorter)     | `SERVO2_PIN`         |
| 23  | HX711 Load Cell      | `HX711_DOUT_PIN`     |
| 32  | KY-024 Magnetic Sensor| `KY_024_PIN`         |
| 34  | E18-D80NK IR Sensor  | `E18_D80NKPin`       |

***

## System Architecture

The firmware is designed around a set of independent tasks, each responsible for a specific piece of hardware or logic. Communication between tasks is handled by a central FreeRTOS queue (`stateMachineQueue`), which passes events to the main state machine.

### Task Distribution

* **Core 0 (Protocol Core)**
    * `bleMessageHandlerTask`: Manages incoming BLE data. It parses messages ("light", "heavy", "none") and sends corresponding events to the state machine. It includes a timeout to handle cases where an object is seen but not classified.
    * *BLE Stack*: The underlying Bluetooth stack runs on this core.

* **Core 1 (Application Core)**
    * `stateMachineTask`: The brain of the system. It waits for events and transitions the system between states, controlling the motor and servos. (Highest Priority)
    * `E18_D80NKTask`: Monitors the IR sensor to detect objects entering or leaving the conveyor's starting point.
    * `HX771Task`: Manages the load cell, taking weight readings when required.
    * `KY_024Task`: Polls the magnetic sensor to detect metallic objects.

### State Machine

The system can be in one of the following states:

* `PAIRING`: Waiting to establish a connection with the BLE server.
* `CONNECTED_IDLE`: Connected and waiting for an object to be placed on the conveyor.
* `RUNNING`: An object has been detected, and the conveyor is moving. Waiting for classification.
* `LIGHT_LUGGAGE`: Object classified as "light" is moving towards the magnetic sensor.
* `HEAVY_LUGGAGE`: Object classified as "heavy" (or a magnetic "light" object) is moving towards the scale.
* `WEIGHING`: The conveyor is stopped, and the load cell is measuring the object's weight.
* `DISCONNECTED`: The BLE connection has been lost.
* `OVERLAP_DETECTED`: An emergency stop state triggered if a new object is detected while another is being processed.

***

## Configuration

Before uploading the firmware, you must configure the BLE identifiers to match your BLE server (e.g., the ESP32-CAM).

1.  Open the `FREE_RTOS_CONVEYOR.ino` file.
2.  Locate the following lines:
    ```cpp
    // --- IMPORTANT: CONFIGURE YOUR SERVER's UUIDs HERE ---
    const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    const char* CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    ```
3.  Replace the placeholder UUIDs with the exact UUIDs used by your BLE server device.

Additionally, you may need to adjust the following constants based on your specific hardware and requirements:
* `CALIBRATION_FACTOR`: This value for the HX711 load cell must be determined by running a calibration sketch.
* `THRESHOLD_..._GRAMS`: Adjust the weight thresholds for object detection and acceptance.
* `SERVO..._ANGLE`: Calibrate the servo angles for your physical setup.
* `THRESHOLD_MAGNETIC_FIELD_DETECTED`: Calibrate the magnetic sensor's trigger threshold.

## Getting Started

1.  **Clone the Repository**: Clone the repo and its submodules (see **Repository Structure** section).
2.  **Open in IDE**: Open the project folder in a compatible IDE like PlatformIO (recommended) or the Arduino IDE.
3.  **Install Libraries**: Ensure you have the necessary libraries installed:
    * `ESP32Servo`
    * Any custom libraries included in the `lib` folder (e.g., `BLEClient`, `DCMotor`, `HX711_RTOS`).
4.  **Configure**: Update the BLE UUIDs and other constants as described in the **Configuration** section above.
5.  **Build and Upload**: Connect your ESP32, select the correct COM port, and upload the firmware.
6.  **Run**: Power the system and open the Serial Monitor at `115200` baud to see debug messages and system status.
