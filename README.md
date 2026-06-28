# Farm-sensor-and-control-network
sensor network, camera and control of remote farm
🌾 Farm Sensor Network (LoRa + Solar IoT)
A low-power, long-range sensor network designed for farm monitoring, including tank levels, soil conditions, and environmental data. Built for reliability, minimal maintenance, and scalability using solar-powered LoRa nodes.

🚀 Project Overview
This system consists of distributed solar-powered sensor nodes that periodically wake, collect data, and transmit it over LoRa to a central gateway connected to the internet for remote monitoring.
Core goals:

✅ Long-range communication (km-scale)
✅ Ultra-low power operation
✅ Solar-powered, maintenance-free nodes
✅ Modular design (easy to expand)
✅ Reliable outdoor deployment


🏗️ System Architecture
[ Sensor Nodes ]
      ↓ (LoRa)
[ Gateway / Hub ]
      ↓ (WiFi / Ethernet / LTE)
[ Cloud / Dashboard ]


📡 Sensor Nodes
Each node is a self-contained unit deployed across the farm.
🔋 Power System

Solar panel (oversized for reliability)
Lithium battery (single-cell)
Solar charge controller (MPPT-style preferred)
Optional regulated output (3.3V rail)

Power Strategy:

Sleep most of the time
Wake every ~60 minutes
Measure → transmit → sleep again


🧠 Controller

Low-power microcontroller (ESP32-based currently)
Deep sleep for power efficiency
Controls sensors and radio


📶 Communication

LoRa radio (915 MHz band)
Long-range, low power transmission
Star topology (node → gateway)


⚙️ Sensor Power Management

High-side MOSFET switching
Sensors powered only during measurement
Prevents drift, corrosion, and battery drain


🌊 Tank Level Node
Function

Measures water level in tanks using ultrasonic sensing

Characteristics

Mounted at top of tank
Periodic measurement
Noise filtering via multiple readings

Considerations

Wind/water surface noise mitigation
Physical mounting stability
Weatherproof enclosure


🌱 Soil Sensor Node
Measurements

Soil moisture (primary)
Temperature
Electrical conductivity (EC)
pH (trend-level accuracy)
Optional: NPK (experimental)

Notes

Moisture is the primary actionable metric
EC provides fertiliser/salinity insight
pH useful for trends only
NPK values are approximate and not relied upon for decisions


Sensor Strategy

Sensors powered only during readings
Stabilisation delay before sampling
Averaged readings for noise reduction


🔌 Communication Strategy

LoRa packets sent periodically (e.g. hourly)
Payload includes:

Sensor data
Battery voltage
Node ID



Example:
JSON{  "node": "soil_1",  "moisture": 42,  "temp": 18.3,  "ec": 1200,  "ph": 6.7,  "battery": 3.9}Show more lines

🧭 Gateway
Role

Receives LoRa messages from nodes
Forwards data to cloud or local system

Connectivity

WiFi / Ethernet / Cellular

Software options

MQTT-based systems
Node-RED dashboards
Cloud IoT platforms


⚡ Power Design Philosophy

Solar-first design (battery as buffer)
Oversized panels for reliability
Minimise continuous loads
Prefer deep sleep over idle


📡 RF Design Considerations

External antenna strongly preferred
Antenna placement critical (height matters)
Avoid PCB/spring antennas for long range
Matching frequency band (915 MHz AU)


🔧 Deployment Considerations

Weatherproof enclosures (IP-rated)
Cable glands and sealing
Condensation control (desiccant)
Physical protection from livestock/environment
Elevated antenna placement


🧠 Software Behaviour (Node Loop)
Wake from deep sleep
    ↓
Enable sensor power
    ↓
Wait for stabilisation
    ↓
Take multiple readings
    ↓
Average/filter values
    ↓
Transmit via LoRa
    ↓
Disable sensor power
    ↓
Return to sleep


🚜 Current Status
✅ Implemented / Designed

Solar-powered LoRa node architecture
Tank level sensing (ultrasonic)
Soil sensing (multi-parameter)
Power gating using MOSFET
Low-power duty cycling model
Component selection and system layout


🔮 Future Expansion
📈 Network

Multiple node deployment across farm
Increased sensor density
Mesh or extended LoRa network if needed


🌍 Data & Dashboard

Cloud-based monitoring dashboard
Alerts (low water, dry soil, etc.)
Historical data analysis
Mobile access


🔧 Hardware Improvements

Custom PCB for node integration
Migration to lower-power MCU variants
Improved antenna systems
Industrial-grade sensor upgrades


🌱 Sensors

Additional environmental sensing (weather, rainfall)
Flow meters
Pump/irrigation control integration
More accurate soil probes (future upgrade)


⚡ Power Optimisation

Further reduction of idle current
Smarter adaptive transmit intervals
Battery health monitoring


🎯 Design Principles

Keep it simple
Keep it robust
Design for failure (remote recovery)
Prioritise real-world reliability over lab precision
Build modular and scalable


✅ Summary
This project is a low-power, long-range farm monitoring system designed for real-world deployment with:

Solar-powered autonomous nodes
LoRa-based long-distance communication
Modular sensor design
Expandable architecture

Built to be:

Reliable, maintainable, and scalable in harsh outdoor environments
