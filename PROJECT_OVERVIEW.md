\# REACTIVE-LEDS-WITH-ESP32 — Project Overview



Repository: https://github.com/Matadorcrypto/REACTIVE-LEDS-WITH-ESP32



\## What this project is

ESP32 firmware that controls addressable LEDs (e.g. WS2812B) and reacts to audio captured from a microphone.

Goal: a sound-reactive LED controller (an alternative to WLED for this use case) with multiple color/mode options and optional web-based control.



\## Primary language

\- C++ (Arduino / ESP32 ecosystem)



\## Where the “main code” lives

Main sketch (current):

\- `sketch\_feb26a/sketch\_feb26a.ino`



> If you later rename the sketch folder/file, update this document.



\## Hardware (typical)

This section is intentionally generic—update with your exact parts.



\- ESP32 dev board

\- Addressable LED strip (WS2812B/NeoPixel or similar)

\- Microphone module (often I2S mic like INMP441 / SPH0645, or analog mic depending on code)

\- Suitable power supply for LEDs (very important)

\- Common ground between ESP32 and LED power



\## Safety / wiring notes (important)

\- Do NOT power long LED strips directly from the ESP32 5V pin.

\- Use an external 5V supply sized for your LED count.

\- Connect grounds together (ESP32 GND ↔ LED power GND).

\- Consider a resistor on LED data line and a large capacitor across LED power (common best practices).



\## How to contribute changes

\- Make changes in the sketch file/folder above

\- Commit with a clear message describing what changed and why

\- Push to `main` (or use a branch + PR if desired)



\## Changelog (manual)

\- 2026-02-26: Repo created; sketch added under `sketch\_feb26a/`; removed duplicate pasted .ino file

