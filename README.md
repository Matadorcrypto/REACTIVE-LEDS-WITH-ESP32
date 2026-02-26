# ESP32 Audio Reactive Rectangle LEDs (INMP441 + WS2812B)

Audio-reactive LED controller for a rectangular LED border using:
- ESP32 (Wi‑Fi + Web UI)
- INMP441 I2S microphone
- WS2812B LED strip (275 LEDs)

Includes multiple modes (music bars, VU border, corners hit, rainbow, etc.) and a **Flashy / No‑Flash** toggle for the music visuals.

## Hardware
### Components
- ESP32 Dev Board (ESP32-WROOM)
- INMP441 I2S Microphone Module
- WS2812B LED strip, 275 LEDs
- 5V PSU (recommended 15–20A)
- 330–470Ω resistor (data line, recommended)
- 1000–2200µF capacitor (5V rail near strip start, recommended)

### Wiring
#### INMP441 -> ESP32
- VDD -> 3.3V
- GND -> GND
- SCK/BCLK -> GPIO 14
- WS/LRCL -> GPIO 27
- SD/DOUT -> GPIO 32

#### WS2812B -> ESP32/PSU
- DIN -> GPIO 5 (through 330–470Ω resistor)
- +5V -> PSU +5V
- GND -> PSU GND
- ESP32 GND -> PSU GND (common ground required)

Power inject at both ends of the strip (and optionally middle) to reduce voltage drop.

## Software / Libraries
- ESP32 Arduino Core (Espressif Systems)
- FastLED library

## Build/Upload
Open the `.ino` in Arduino IDE, select your ESP32 board and COM port, set Wi‑Fi credentials in the sketch, then upload.

## Notes
If you see flicker or random resets, it is often power-related:
- increase PSU capacity
- add power injection points
- add capacitor on 5V rail
- ensure common ground
