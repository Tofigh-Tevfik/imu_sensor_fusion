# Orientation Estimation Filters on ESP32

This project implements and compares multiple **orientation estimation algorithms** on a **6-axis Inertial Measurement Unit (IMU)** using an **ESP32 microcontroller**. It demonstrates the practical performance, accuracy, and computational trade-offs of various filtering techniques for real-time orientation estimation on resource-constrained embedded systems.

---

## 🧭 Overview

The system uses a **3-axis accelerometer** and a **3-axis gyroscope** to estimate orientation. Raw IMU data is processed on the ESP32 using several sensor fusion algorithms, and the resulting orientation is transmitted via **UDP** to a host computer for visualization and analysis.

The following filters are implemented and compared:

- 🟢 **Low-Pass Filter** — simple, noise-tolerant but slow and inaccurate during fast motion  
- 🔵 **Complementary Filter** — lightweight, blends low- and high-frequency data from sensors  
- 🟣 **Madgwick Filter** — fast convergence, good noise robustness  
- 🟠 **Mahony Filter** — stable and responsive, suitable for real-time systems  
- 🔴 **Extended Kalman Filter (EKF)** — probabilistic optimal estimator, high accuracy, but computationally intensive  

---

## ⚙️ System Architecture

- **Microcontroller:** ESP32  
- **Sensors:** 6-DoF IMU (3-axis accelerometer + 3-axis gyroscope)  
- **Communication:** UDP (ESP32 → Host PC)  
- **Processing:** Real-time filter computation on ESP32  
- **Visualization:** Host-side software (e.g., Python, MATLAB, or Unity)  

---

## 🧩 Project Goals

- Implement and run multiple orientation estimation filters on ESP32  
- Evaluate **stability**, **responsiveness**, and **robustness** to sensor noise and drift  
- Compare **computational cost vs. accuracy** for embedded implementations  
- Provide **practical insights** into real-time orientation estimation for drones, robotics, and wearable devices  

---

## 🧠 Key Concepts

- **Sensor Fusion:** Combining accelerometer and gyroscope data to produce accurate orientation estimates  
- **Drift Correction:** Using accelerometer data to correct gyroscope drift over time  
- **Real-Time Filtering:** Achieving stable performance on an embedded system with limited processing resources  

---

## 🛠️ Getting Started

### 1. Hardware Setup
- ESP32 development board  
- 6-axis IMU sensor (e.g., MPU6050, MPU9250, or similar)  
- USB connection to PC  
- Optional: serial monitor for debugging  

### 2. Software Requirements
- Arduino IDE or PlatformIO  
- ESP32 board package  
- Required libraries for IMU and filters (specified in the source code)  

### 3. Running the Project
1. Connect the IMU to the ESP32 (I2C or SPI, depending on sensor).  
2. Flash the code to the ESP32.  
3. Open the UDP listener or visualization tool on your PC.  
4. Observe the estimated orientation data in real time.  

---

## 📊 Evaluation Criteria

| Filter | Accuracy | Responsiveness | Noise Robustness | Computation Cost |
|:-------|:----------|:----------------|:-----------------|:-----------------|
| Low-Pass | ⭐ | ⭐⭐ | ⭐⭐ | ⭐ |
| Complementary | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| Madgwick | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| Mahony | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| EKF | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

---

## 🧩 Applications

- UAV and drone attitude estimation  
- Robotic motion tracking  
- Human motion and wearable sensing  
- Control systems requiring real-time orientation feedback  

---

### 📁 Repository Structure

ESP32-Orientation-Filters/
│
├── src/
│   ├── lib/
│   │   ├── IMU/                # IMU data acquisition over I2C
│   │   └── Filters/            # All filter implementations (Low-pass, Complementary, Madgwick, Mahony, EKF)
│   └── main.cpp                # Main firmware entry point for ESP32
│
├── python_visualizer/
│   └── over_udp.py             # Python script for real-time UDP data visualization
│
├── docs/                       # Documentation, analysis, and filter performance results
└── README.md                   # Project documentation (this file)


