# Bill of Materials (BOM) – Carbon Footprint Monitor

## Summary
- **Total Cost (approx):** ₹2,700–3,000
- **New Components to Buy Now:** ₹0 (you already have everything)
- **Status:** All hardware available, ready to build

---

## Components You HAVE

| Component            | Qty | Specs                              | Est. Price (₹) | Notes                    |
|----------------------|-----|------------------------------------|----------------|--------------------------|
| ESP8266 NodeMCU      | 1   | 80 MHz, 4MB Flash                  | 400–500        | Main microcontroller     |
| YF-S402 Flow Sensor  | 1   | 1–60 L/min, Pulse output           | 800–1000       | Measures fuel/water flow |
| NEO-6M GPS Module    | 1   | 10 Hz, ±5 m accuracy              | 500–700        | Distance tracking        |
| 16x2 I2C LCD         | 1   | I2C, addr 0x27                     | 200–300        | Status display           |
| DHT22 Sensor         | 1   | Temp/Humidity, ±0.5 °C            | 250–350        | Temp compensation        |
| 10k Resistor         | 1+  | 1/4 W, Carbon film                 | 5–20           | Pull‑up for DHT22        |
| Breadboard           | 1   | 830‑point                          | 150–200        | Prototyping              |
| Jumper Wires         | 50+ | Male/Female                        | 100–150        | Connections              |
| 5V/2A Power Supply   | 1   | USB output                         | 200–300        | System power             |
| USB Cable            | 1   | USB‑A to Micro‑B/Type‑C            | 50–100         | Programming + power      |
| **TOTAL (approx)**   |     |                                    | **2,700–3,000**| Already purchased        |

---

## Tools Needed

| Tool            | Purpose                     | Est. Cost (₹) |
|-----------------|-----------------------------|---------------|
| Soldering iron  | Permanent connections (opt) | 300–500       |
| Multimeter      | Testing voltage/resistance  | 300–600       |

---

## Pin Connection Overview
ESP8266 → DHT22
D7 (GPIO13) → DATA (via 10k pull‑up to 3.3V)
3.3V → VCC
GND → GND

ESP8266 → Flow Sensor (YF-S402)
D5 (GPIO14) → FLOW PULSE
5V → VCC
GND → GND

ESP8266 → GPS (NEO‑6M)
D3 (TX) → GPS RX
D4 (RX) ← GPS TX
5V → VCC
GND → GND

ESP8266 → 16x2 I2C LCD
D2 (SDA) → SDA
D1 (SCL) → SCL
5V → VCC
GND → GND

---

## Notes

- Prices are approximate and depend on seller and city.
- Since you already own everything, you can directly start from wiring and uploading `code`.

