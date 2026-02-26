\# Workflow: Arduino IDE + Git + GitHub (Windows)



\## One-time setup (clone repo)

```powershell

git clone https://github.com/Matadorcrypto/REACTIVE-LEDS-WITH-ESP32.git

cd REACTIVE-LEDS-WITH-ESP32

```



\## Rule #1: Always run git commands from inside the repo folder

Before `git add/commit/push`, do:

```powershell

cd C:\\Users\\Propietario\\REACTIVE-LEDS-WITH-ESP32

git status

```



If you see `fatal: not a git repository`, you are in the wrong folder.



\## Save Arduino sketch into the repo (recommended)

In Arduino IDE:

1\. Open sketch

2\. File → Save As…

3\. Choose the repo folder: `...\\REACTIVE-LEDS-WITH-ESP32\\`



Arduino will create a folder like:

`REACTIVE-LEDS-WITH-ESP32\\<SketchName>\\<SketchName>.ino`



Current sketch location:

\- `sketch\_feb26a/sketch\_feb26a.ino`



\## Typical update flow (after editing code)

```powershell

cd C:\\Users\\Propietario\\REACTIVE-LEDS-WITH-ESP32

git status

git add .

git commit -m "Describe what changed"

git push

```



\## If you deleted files using File Explorer (Windows)

Git needs to be told about deletions:

```powershell

cd C:\\Users\\Propietario\\REACTIVE-LEDS-WITH-ESP32

git add -u

git commit -m "Remove old files"

git push

```



\## Verify remote repo (make sure pushes go to the right place)

```powershell

git remote -v

```



Expected:

\- `https://github.com/Matadorcrypto/REACTIVE-LEDS-WITH-ESP32.git`



\## Line ending warning (LF/CRLF)

You may see:

`LF will be replaced by CRLF`



This is normal on Windows and usually safe to ignore.

