# Autonomous Line-Following Car with TI-RSLK MAX 🚗

### Final Project for ECE3 - UCLA
This project showcases an autonomous line-following car built using the TI-RSLK MAX robotics kit. The robot is programmed to 
follow a line, detect intersections, and perform turnarounds based on specific conditions. The goal was to apply concepts 
from ECE3 to develop a functional and reliable embedded system.

---

## 📜 Overview
The TI-RSLK MAX robot uses 8 infrared (IR) sensors for line detection and a PID control algorithm for smooth navigation. It 
includes additional logic for detecting intersections and time constraints to ensure consistent operation. The car adjusts its
speed dynamically and performs maneuvers like turnarounds when specific conditions are met.

---

## ⚙️ Features
- **Line Following**: Uses a PID control system for precise steering.
- **Turnaround Detection**: Detects intersections and performs a turnaround when necessary.
- **Timing Constraints**: Ensures actions occur within defined time frames for better performance.
- **Customizability**: Adjustable PID constants and thresholds for optimal performance.

---

## 🛠 Hardware Requirements
- **TI-RSLK MAX Robotics Kit** (includes chassis, IR sensors, and MSP432 microcontroller)
- 8 IR sensors for line detection

---

## 🖥 Software Requirements
- **Arduino IDE** (v1.8.19 or later)
- ECE3 library (included in the repository)

