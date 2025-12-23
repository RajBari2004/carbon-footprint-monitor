# Reference Card – Carbon Footprint Monitor (DHT11)

## Hardware Used
- ESP8266 NodeMCU  
- YF‑S402 flow sensor  
- NEO‑6M GPS module  
- DHT11 temperature sensor + 10k pull‑up  
- 16x2 I2C LCD  
- Breadboard, jumper wires, 5V/2A supply, USB cable  

---

## Pin Connections (ESP8266)

- D1 (GPIO5) → LCD SCL  
- D2 (GPIO4) → LCD SDA  
- D4 (GPIO2) → Flow sensor signal  
- TX (GPIO1) → GPS RX  
- RX (GPIO3) ← GPS TX  
- D7 (GPIO13) → DHT11 DATA + 10k to 3.3V  
- 3.3V → DHT11 VCC  
- 5V → LCD VCC, Flow sensor VCC, GPS VCC  
- GND → All grounds common  

---

## Libraries Required (Arduino IDE)

- DHT sensor library (by Adafruit)  
- Adafruit Unified Sensor  
- LiquidCrystal I2C (by Frank de Brabander)  
- TinyGPS++ (by Mikal Hart)  

---

## Code Settings (important defines)

#define FLOW_SENSOR_PIN D4
#define DHT_PIN D7
#define DHT_TYPE DHT11 // DHT11 selected

FuelType currentFuel = FUEL_PETROL; // PETROL / DIESEL / WATER / LPG

text

Wi‑Fi (set before upload):

const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

text

---

## How to Run

1. Wire all sensors as per pins above.  
2. Install required libraries.  
3. Open `code.ino`, set Wi‑Fi credentials, board = **NodeMCU 1.0 (ESP8266)**.  
4. Upload code.  
5. Open Serial Monitor @ 115200, note IP address.  
6. Open browser: `http://<IP_SHOWN>` to view dashboard.

---

## Quick Troubleshooting

- **DHT shows `--`** → check 10k from D7 to 3.3V, wiring of DHT11.  
- **Compile error: DHT.h / Adafruit_Sensor.h missing** → install DHT + Adafruit Unified Sensor libs and restart IDE.  
- **No web page** → check IP in Serial Monitor, same Wi‑Fi network, try again after reset.  
- **Flow always 0** → check D4 wiring and that flow sensor is powered by 5V.  
- **GPS waiting** → normal indoors; test near window or outside for a few minutes.
