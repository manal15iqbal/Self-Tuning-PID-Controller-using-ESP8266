# Self-Tuning PID Controller using ESP8266
Self-tuning PID motor control system using ESP8266 with vibration and Back EMF feedback, featuring real-time monitoring via a Python-based dashboard.

## Overview

Manual PID tuning is inefficient and often results in instability, oscillations, and energy loss.
This project solves that by implementing an **automatic PID tuning system** that:

* Detects instability using **vibration signals**
* Estimates system response using **Back EMF feedback**
* Automatically computes optimal PID parameters
* Improves motor stability without manual intervention


## Core Idea

The system combines:

* **Vibration feedback (Microphone + FFT)** → detects oscillations
* **Back EMF feedback** → estimates motor speed behavior
* **Ziegler–Nichols method** → computes PID parameters

This hybrid approach enables **fast and reliable auto-tuning**.

---

## System Architecture

```id="arch1"
ESP8266 (PID Controller)
        │
        │ Serial Communication
        ▼
Python Flask Server (Bridge)
        │
        ▼
Web Dashboard (Real-Time Visualization)
```

---

## Methodology

1. System starts in unstable condition
2. Auto-tuning is triggered
3. Back EMF → determines **Ku (ultimate gain)**
4. Vibration + FFT → determines **Pu (oscillation period)**
5. PID parameters calculated
6. New parameters applied
7. System stabilizes

---

## Hardware Components

* ESP8266 (Microcontroller)
* DC Motor (Plant)
* L298N Motor Driver
* MAX4466 Microphone (Vibration sensing)
* Back EMF sensing circuit
* Resistors for signal conditioning

---

## Software Stack

* Arduino IDE (Embedded programming)
* Python (Flask + PySerial)
* HTML + Bootstrap + Chart.js (Dashboard)

---

## PID Tuning Algorithm

Using Ziegler–Nichols method:

```id="pid_formula"
Kp = 0.6 × Ku  
Ki = (2 × Kp) / Pu  
Kd = (Kp × Pu) / 8  
```

Where:

* **Ku** → Ultimate gain (from oscillations)
* **Pu** → Oscillation period (from FFT)

---

## Results

### Before Auto-Tuning

* High vibration amplitude
* Oscillatory motor behavior
* Slow settling time

### After Auto-Tuning

* Reduced oscillations
* Faster settling time
* Stable motor speed
* Lower vibration peaks

---

## System Behavior

The system operates in three modes:

* **IDLE** → Motor stopped
* **RUNNING** → PID-controlled operation
* **TUNING** → Automatic parameter adjustment

---

## 📡 Serial Communication Protocol

ESP8266 sends structured data:

```id="serial_format"
RPM:1200
Noise=15
Kp = 0.5
Ki = 0.1
Kd = 0.05
```

---


## How to Run

1. Install dependencies:

```bash id="run1"
pip install -r requirements.txt
```

2. Upload Arduino code to ESP8266

3. Run backend:

```bash id="run2"
python backend/bridge_server.py
```

4. Open browser:

```id="run3"
http://localhost:5000
```

---

## Future Improvements

* Closed-loop response plotting (setpoint vs output)
* Data logging and export
* WebSocket-based real-time streaming
* Improved noise filtering for sensor signals

---

## Conclusion

This project successfully demonstrates a **low-cost, sensor-driven self-tuning PID controller** that:

* Eliminates manual tuning
* Improves system stability
* Integrates embedded systems with real-time software monitoring

---
