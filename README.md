# Roborock Q7 & 3irobotix B01 Diagnostic Tool (Port 6001)

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Dependencies](https://img.shields.io/badge/Dependencies-Zero%20(Standard%20Library)-success.svg)]()
[![License](https://img.shields.io/badge/License-The%20Unlicense-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Roborock%20Q7%20%7C%203irobotix%20B01-orange.svg)]()

The best way to support this project is the next time you are buying a Roborock device come back here and use one of my affiliate links where I will receive a commission:

[![Amazon Affiliate][badge-amazon]][link-amazon]
[![Roborock Affiliate][badge-roborock-affiliate]][link-roborock-affiliate]

You can also support via GitHub Sponsors, Buy Me a Coffee, or PayPal:

[![GitHub Sponsors][badge-sponsor]][link-sponsor]
[![Buy Me a Coffee][badge-bmac]][link-bmac]
[![PayPal][badge-paypal]][link-paypal]

---

A standalone hardware diagnostic suite, real-time telemetry decoder, actuation test bench, and DIY repair guide for the **Roborock Q7 series** and **Shenzhen 3irobotix B01 / Everest OS** ODM robot vacuums.

Connects directly to the robot over its factory Wi-Fi access point via **TCP port 6001** (`192.168.5.1:6001`). Decodes the proprietary 36-tag Nanopb telemetry stream (`RobotCleaner_Info_msg`), reads raw optical cliff sensor ADC reflections against calibrated thresholds, tests motor and actuator subsystems in real-time, inspects microswitches, and generates actionable repair guides for common hardware faults like **Error 4 (Cliff Sensor Fault)** and **Error 2 (Bumper Stuck)**.

---

## Key Features

- **Zero Dependencies**: Runs entirely on the Python 3.8+ standard library (`socket`, `struct`, `http.server`, `json`, `dataclasses`). No `pip install` required.
- **Real-Time Nanopb Telemetry Decoder**: Decodes 36 protobuf tags in real-time at 10 Hz over TCP port 6001.
- **Optical Cliff Sensor Diagnostic (Error 4)**:
  - Live 10-bit ADC reflection readings across all optical receivers.
  - Compares raw ADC against calibrated floor (`0x70`) and drop (`0x19`) thresholds stored in vendor NVRAM.
  - Distinguishes between dirty lenses, physical drop conditions, disconnected wiring harnesses, and dead IR emitter diodes.
- **Hardware Actuation Test Bench**:
  - **Drive Wheels**: Test left and right brushless wheel motors forward and reverse (100 mm/s) with automated safety interlock clearing.
  - **Suction Blower Fan**: Spin up the turbine through 5 speeds: Quiet (1,200 RPM), Balanced (2,500 RPM), Turbo (5,000 RPM), Max (7,500 RPM), and Off.
  - **Vibrating Mop Actuator**: Sonic mopping frequency pulse tests (Low, High, Off).
  - **Status LEDs & Audio**: Drive front white and red button LEDs (`0x8D`), Wi-Fi indicator (`0x8E`), and speaker test tones (`0x81`).
- **Bumper & Sensor Inspection (Error 2)**: Real-time status of left and right optical bumper interrupters, LiDAR optical cover bumper switch, and dustbin magnetic reed switch.
- **Battery & Power Subsystem**: Pack voltage, current draw, cycle count, temperature, and battery health degradation scores.
- **Automatic Port Fallback**: Web server starts on `8080` and gracefully increments (`8081`, `8082`...) if port collisions occur.

---

## Platform & Hardware Compatibility

> [!IMPORTANT]
> This diagnostic tool targets models designed on the **Shenzhen 3irobotix B01 / Everest OS** ODM architecture (SigmaStar SSC337DE SoC). It connects exclusively via TCP port 6001.

### Supported Models

*(Verified on the **Roborock Q7**, but likely supports all vacuums built on the **Shenzhen 3irobotix B01 / Everest OS** ODM platform).*

- **Roborock Q-Series (3irobotix B01 Architecture)**:
  - Roborock Q7 / Q7+ (`roborock.vacuum.sc05`)
  - Roborock Q7 Max / Q7 Max+
  - Roborock Q10 Series
- **3irobotix B01 ODM Platform Vacuums**:
  - **Cecotec Conga** (B01 series: Conga 3000, 4000, 5000, 7000, 8000, 9000 B01 variants)
  - **Wilfa Innobot** (B01 variants)
  - **Tesvor & Mamibot** B01 series
  - Any Everest OS platform device listening on TCP port 6001 with password `test3irobotix`.

### Unsupported Models (Incompatible)
- **Roborock S-Series & Flagships**:
  - Roborock **S5, S5 Max, S6, S6 Pure, S6 MaxV, S7, S7 MaxV, S7 Pro Ultra, S8, S8 Pro Ultra, Q Revo** series.
  - *Why unsupported*: S-series robots are engineered on Roborock's in-house platform (Rockchip RK3308 / Allwinner SoCs) running the proprietary `RRCore` Linux system. They do not have the 3irobotix Everest factory diagnostic harness, do not open TCP port 6001, and do not use the Nanopb `RobotCleaner_Info_msg` protocol.

---

## Required Physical Entry Steps (Activates Port 6001)

Roborock vacuums do not expose port 6001 during normal retail operation. You must perform the two-pass physical entry sequence to activate the factory diagnostic AP:

### Pass 1: Clear the Hardware Gate (Uncover Cliff Sensors)
1. **Raise the vacuum off the floor** (e.g. balance it on a central box or stand) so that all **4 cliff sensors are completely uncovered** and face open air.
2. Gently push and hold **both Left & Right front bumper sides inward** simultaneously.
3. While holding the bumpers inward, press and hold **Home/Dock + Power** together.
4. **Keep holding until the power-up chime completes** (~20 seconds).
5. **The LEDs will turn a distinct whitish-red color** (both white and red LEDs driven at 99% duty cycle), confirming the hardware test gate is cleared.
6. Turn the vacuum off using the Power button and wait **15 seconds** for a complete cold shutdown.

### Pass 2: Boot into Factory AP (Port 6001 Diagnostic Mode)
1. Place the vacuum on a **clear, level floor** with the front bumper **completely released** (ensure neither bumper side is pressed).
2. Press and hold **Home/Dock** (press Home first), then hold **Power** together.
3. **Keep holding until the power-up chime completes** (~20 seconds).
4. The LEDs will turn whitish-red again as the factory runtime engages.
5. The vacuum will broadcast its factory Wi-Fi access point (SSID is the robot's **Serial Number** or `DEFAULTSN`).

### Pass 3: Connect PC to the Robot's Wi-Fi
1. On your PC, connect to the robot's Serial Number Wi-Fi network.
2. Enter the factory WPA2 password: **`test3irobotix`**.
3. The vacuum's default gateway IP address is **`192.168.5.1`**.

---

## Quick Start

Once connected to the vacuum's Wi-Fi network:

```bash
# Clone the repository
git clone https://github.com/Lash-L/roborock-q7-diagnostics.git
cd roborock-q7-diagnostics

# Launch the diagnostic dashboard (zero pip install required!)
python app.py
```

The web dashboard will open automatically in your browser at `http://127.0.0.1:8080`. Click **Start Diagnostics (Connect)** to begin real-time telemetry streaming and hardware inspection.

---

## Hardware Safety & Testing Notes

### MCU Hardware Drop Interlock
The STM32 MCU on the Q7 enforces a hardcoded finger-safety interlock in firmware:
- If the robot is elevated or inverted, suspension springs extend the drive wheels (`left_wheel_up = 1`) and cliff sensors read open air (`0 ADC`).
- In this state, the MCU **locks out wheel motors and high-speed fan commands**.
- **To test drive wheels or the suction fan**: Always place the robot flat on the floor (or compress the wheel suspension into the chassis) before triggering actuation buttons.

### Carpet Detection on the Q7
If the dashboard reports **Carpet Ultrasonic (Tag 27): HARD FLOOR** while sitting on carpet, this is normal:
- The Roborock Q7 **does not contain a physical ultrasonic acoustic transducer** (that sensor is exclusive to S7/S8 series).
- On the Q7, carpet detection is performed dynamically in software (`CCarpetDetect::detectCarpetWithRoll`) by monitoring main roller brush motor load (`Tag 7: rolling_brush.current_ma`). When stationary on the floor, the roller is idle, so it defaults to hard floor.

---

## Protocol Architecture & Framing

Port 6001 communicates using a 16-byte binary TCP header:

```
+--------------------+--------------------+--------------------+--------------------+
|  Magic (4 bytes)   |  Command (4 bytes) |  Length (4 bytes)  | Reserved (4 bytes) |
|     0x51589158     |   (cmd_id << 8)    |  Payload byte len  |     0x00000000     |
+--------------------+--------------------+--------------------+--------------------+
```

- **Command `0x65` (Passive Telemetry)**: The vacuum streams 10 Hz Nanopb binary payloads prefixed by magic `0xabbaccdd`.
- **Command `0x70` (Wrapper Robot Packet)**: Used for dispatching motor drive commands (`0x67`), cliff bypass (`0x77`), error resets (`0x0A`), and manual test mode (`0x65`).
- **Command `0x84` / `0x85` / `0x68`**: Direct fan blower PWM and target RPM registers.
- **Command `0x6C`**: Sonic mop vibrator struct control (`TSonicTank`).
- **Command `0x8D` / `0x8E`**: Front button and Wi-Fi indicator LED control registers.
- **Command `0xF7`**: Reads vendor storage slot 33 (factory authentication credential). *Note: SSH (port 22) is disabled by default on stock firmware.*

---

## Project Structure

```
roborock-q7-diagnostics/
├── .github/
│   └── FUNDING.yml       # GitHub Sponsors, Buy Me a Coffee, PayPal
├── app.py                # Standalone HTTP/SSE server with port fallback & REST API
├── q7_protocol.py        # Port 6001 client, Nanopb 36-tag decoder & packet framer
├── diagnostic_engine.py  # Fault detector, health scoring & step-by-step DIY repair guides
├── static/
│   └── index.html        # Responsive real-time diagnostic dashboard & test bench UI
├── requirements.txt      # Zero dependencies note (Python 3.8+ standard library)
├── LICENSE               # The Unlicense (Public Domain Dedication)
└── README.md             # Documentation, support links & entry guide
```

---

## Contributing

Contributions, bug reports, and hardware capture dumps from other 3irobotix B01 models are welcome! Please feel free to submit a Pull Request or open an Issue.

---

## License

This project is released into the public domain under [The Unlicense](LICENSE). You are free to copy, modify, publish, use, compile, sell, or distribute this software for any purpose without restrictions or attribution.

<!-- Badge & Affiliate Links -->
[link-sponsor]: https://github.com/sponsors/Lash-L
[badge-sponsor]: https://img.shields.io/badge/Sponsor-EA4AAA?style=for-the-badge&logo=githubsponsors&logoColor=white
[link-bmac]: https://buymeacoffee.com/lashl
[badge-bmac]: https://img.shields.io/badge/Buy%20Me%20a%20Coffee-donate-yellow?style=for-the-badge&logo=buymeacoffee&logoColor=black
[link-paypal]: https://paypal.me/LLashley304
[badge-paypal]: https://img.shields.io/badge/PayPal-donate-00457C?style=for-the-badge&logo=paypal&logoColor=white
[link-roborock-affiliate]: https://roborock.pxf.io/B0VYV9
[badge-roborock-affiliate]: https://img.shields.io/badge/Roborock-affiliate-B22222?style=for-the-badge
[link-amazon]: https://amzn.to/4cx8zg3
[badge-amazon]: https://img.shields.io/badge/Amazon-affiliate-FF9900?style=for-the-badge&logo=amazon&logoColor=white
