# Software Troubleshooting (ESP32 Audio-Reactive LEDs)

This guide is for **software/setup** issues (Arduino IDE, libraries, Wi‑Fi/Web UI, audio processing), not wiring/power.

## 0) Quick “sanity check” (do this first)
1. Open the correct sketch:
   - `sketch_feb26a/sketch_feb26a.ino`
2. Arduino IDE settings:
   - Correct **Board** (your ESP32 model)
   - Correct **Port** (COMx)
3. Open **Serial Monitor**:
   - Start with **115200 baud** (change if your sketch prints at another baud)

---

## 1) Compile errors (libraries missing)
### Symptom
Errors like:
- `fatal error: FastLED.h: No such file or directory`
- missing `WiFi.h`, `WebServer.h`, `ESPAsyncWebServer.h`, etc.

### Fix
- Arduino IDE → **Tools/Sketch** → **Include Library** → **Manage Libraries…**
- Install the missing library(ies) (commonly **FastLED**).
- Recompile.

### Still broken?
- You may have duplicate/conflicting libraries installed.
  - Remove older duplicates from your Arduino `libraries/` folder.
- Restart Arduino IDE and rebuild.

---

## 2) Upload fails (ESP32 won’t flash)
### Symptom
- `Failed to connect`
- `Timed out waiting for packet header`
- Upload starts then stops

### Fix
- Confirm correct COM port selected.
- Try another USB cable/port.
- Close any other Serial Monitor/terminal using the port.
- If your board needs it: hold **BOOT** during “Connecting…”, release when upload begins.

---

## 3) “It uploads but nothing happens”
### Symptom
- Upload succeeds, but LEDs don’t run / no web page / no serial output

### Fix
- Press **EN/RESET** after upload.
- Verify Serial Monitor **baud** (try 115200).
- Confirm the sketch actually calls `Serial.begin(...)` and prints something (if not, add a startup print).
- Make sure you didn’t accidentally upload a different sketch (check `Sketch → Show Sketch Folder`).

---

## 4) Device keeps resetting / “Guru Meditation Error”
### Symptom
- Random resets, crash messages on Serial Monitor

### Fix (software-side)
- Reduce load temporarily to isolate:
  - Lower brightness / lower FPS / simplify effects
- If you have debug toggles in code, enable them to see where it crashes.
- If using a web server + heavy LED effects, you may be hitting timing/memory issues:
  - Reduce buffer sizes
  - Avoid large dynamic allocations inside the loop

### When reporting this bug, include
- Full Serial Monitor crash log
- What mode was running
- Arduino IDE version + ESP32 core version
- Library versions

---

## 5) Web UI / Wi‑Fi not working
### Symptom
- Can’t connect to Wi‑Fi, or page doesn’t load

### Fix
- Watch Serial Monitor for:
  - “Connecting…”
  - the assigned IP address
- Try opening the device by IP: `http://<ip-address>/`
- Ensure your router is **2.4 GHz** (ESP32 usually doesn’t support 5 GHz)
- If credentials are placeholders, replace them and reupload.

### If it used to work, now doesn’t
- You might be connecting to a different network
- Router changed IP range (new IP)
- Cached page: hard refresh browser / try incognito

---

## 6) LEDs show effects but don’t react to music
### Symptom
- Patterns work, but audio seems “dead” (no beat/vu reaction)

### Fix (software-side)
- Look for audio debug values (RMS/peak/beat flags) in Serial Monitor if available.
- Check thresholds/sensitivity:
  - If threshold too high: never triggers
  - If too low: always triggers
- If there are multiple audio modes (I2S vs analog), confirm the correct one is enabled in code.
- Test with a simpler “VU meter” style mode if the code has one.

---

## 7) Git / updating code problems (common)
### Symptom
- `fatal: not a git repository`

### Fix
- You must be inside the repo folder that contains `.git`:
  ```powershell
  cd C:\Users\Propietario\REACTIVE-LEDS-WITH-ESP32
  git status
  ```

### Symptom
- You deleted a file in File Explorer but Git doesn’t “see it”

### Fix
- Stage deletions:
  ```powershell
  git add -u
  git commit -m "Remove old files"
  git push
  ```
