# Arduino-LTM-to-IBUS-conververter
An Arduino-based converter that translates Lightweight Telemetry (LTM) from INAV/Betaflight into the FlySky iBus protocol for real-time radio telemetry.


# Arduino LTM to iBus Telemetry Converter

This repository provides an ultra-reliable Arduino sketch (`.ino`) that acts as a plug-and-play real-time bridge between **Lightweight Telemetry (LTM)** and the **FlySky iBus** telemetry protocol. 

It is specifically optimized to run flawlessly at **19200 baud**, making it the perfect companion for **INAV SoftSerial** ports where hardware resources are limited.

## 🚀 Features
* **Zero Configuration:** The converter automatically detects and accepts the incoming LTM data stream and outputs it directly to your receiver's sensor port.
* **19200 Baud Optimization:** Rock-solid stability engineered specifically for low-overhead INAV SoftSerial telemetry output.
* **Ultra-Lightweight:** Runs easily on compact ATmega328P boards (Arduino Nano, Pro Mini) to save space and weight on your aircraft.

## ⚠️ Important Radio Requirement
To reliably view all extended telemetry data screens (like GPS, battery metrics, and altitude) on a FlySky FS-i6 transmitter, it is **highly recommended to upgrade your radio to the qba667 custom firmware**. The stock FlySky firmware has limited sensor display options.

## 🔌 Connection Diagram
Connect the Arduino to your setup using this simple pin map:

| From Device & Pin | To Arduino Pin | Description |
| :--- | :--- | :--- |
| **Flight Controller TX** (SoftSerial) | **RX Pin** | Automatically accepts incoming 19200 LTM stream |
| **FlySky Receiver SENS Port** | **TX Pin (11)** | Transmits the converted iBus telemetry packets |
| **System 5V & GND** | **VCC & GND** | Shared common power and ground rails |

## 📄 License
This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
