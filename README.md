# 🌍 Advanced Carbon Footprint Monitoring System

An IoT‑based system that measures real‑time fuel flow, estimates CO₂ emissions, and shows data on a web dashboard using an ESP8266 microcontroller.

---

## 🚗 Project Overview

- Monitors **fuel consumption** using a YF‑S402 flow sensor.  
- Uses **DHT11** temperature sensor for temperature‑compensated CO₂ calculation.  
- Uses **NEO‑6M GPS** to estimate distance and efficiency (km/L).  
- Displays data on a **16x2 I2C LCD** and a **responsive web dashboard**.  
- Target accuracy: **≈93%** in Phase‑1 using indirect CO₂ estimation.

---

## ✨ Features

- Real‑time flow rate and total fuel used.  
- Temperature‑compensated CO₂ estimation.  
- GPS‑based distance and basic efficiency calculation.  
- Simple web UI hosted on the ESP8266.  
- Status display on LCD (multiple rotating screens).  

---

## 🧩 Hardware Used

- ESP8266 NodeMCU  
- YF‑S402 flow sensor  
- NEO‑6M GPS module  
- DHT22 temperature & humidity sensor  
- 16x2 I2C LCD  
- Breadboard, jumper wires, 5V/2A supply, USB cable  

> Full cost and parts list: see **BOM.md**.

---

## 🛠️ How to Use

1. **Wire the hardware** according to the pinout in **REFERENCE_CARD.md**.  
2. Install Arduino IDE and required libraries (DHT, Adafruit Unified Sensor, LiquidCrystal_I2C, TinyGPS++).  
3. Open `code.ino` in Arduino IDE, set your Wi‑Fi SSID/password, select **NodeMCU 1.0 (ESP8266)** board.  
4. Upload the sketch to the ESP8266.  
5. Open the Serial Monitor to note the IP address printed on boot.  
6. Open a browser and go to `http://<that-ip>` to see the dashboard.

For a detailed, step‑by‑step guide, read **QUICKSTART.md**.

---

## 📂 Repository Structure
- `code.ino` – main firmware for ESP8266.  
- `BOM.md` – bill of materials (all components and approximate prices).  
- `QUICKSTART.md` – quick start setup and flashing steps.  
- `REFERENCE_CARD.md` – pin mapping, formulas, and quick troubleshooting.  
---

## 📜 License

This project is released under the **MIT License**.  
See the `LICENSE` file for details.
