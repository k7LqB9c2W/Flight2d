# Flight2d
🌍 Real World Flight Simulator with Autopilot
This project is a high-performance 2D real-world flight simulator using real satellite imagery tiles from ArcGIS. It supports manual control, autopilot routing between cities using great-circle navigation, and a dynamic tile pre-caching system for seamless visual experience even at high speeds.

✈️ Features
Real satellite map imagery using ArcGIS tiles

Manual flight with acceleration, deceleration, and turning

Autopilot navigation with great-circle routing

Real-time distance and ETA calculation

Precaching of map tiles for improved performance

Dynamic memory-managed tile caching with eviction strategy

Adjustable zoom levels

SR-71 mode for supersonic flight

Time warp options (2x, 5x, 10x speed)

Keyboard-driven input and preset destination shortcuts

Pygame-based HUD and interface

🎮 Controls
Key	Action
Arrow UP	Increase throttle
Arrow DOWN	Decrease throttle
A / D	Turn plane left / right
Z	Enter zoom level selection mode
1	Toggle SR-71 Blackbird mode
2-4	Set time warp to 2x, 5x, 10x
5	Enter autopilot destination input
F1–F10	Jump to preset destinations
BACKSPACE	Cancel autopilot or reset speed
ESC	Cancel input or zoom selection
ENTER	Confirm input or zoom

🌐 Preset Destinations
New York

London

Tokyo

Paris

Sydney

Cairo

Rio de Janeiro

Moscow

Beijing

Los Angeles

🧱 Requirements
Python 3.9+

Pygame

httpx

Install requirements:

bash
Copy
Edit
pip install pygame httpx
🚀 Running the Simulation
bash
Copy
Edit
python flight.py
Make sure you are connected to the internet to load the satellite imagery in real time.

⚙️ Performance Notes
Up to 300 concurrent tile downloads

Uses asyncio + threading for efficient I/O

Tiles are cached with a memory cap (default: 500MB)

Smart eviction ensures on-screen tiles are never removed

📦 Future Ideas
Weather overlays

Realistic aircraft physics

Flight logbook system

Multiplayer or replay system

VR/AR support

📜 License
This project is for educational and personal use only. Map tiles are provided by Esri ArcGIS services and subject to their terms of use.
