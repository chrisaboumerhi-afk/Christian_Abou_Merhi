# ESP32 GPS + GSM Tracker (NEO-6M + SIM800L)

ESP32 firmware (Arduino framework, PlatformIO) that reads a NEO-6M GPS and sends you your position by SMS through a SIM800L:

- Every **30 s**, if the GPS has a valid fix: SMS with `https://maps.google.com/?q=LAT,LON`
- Text **`LOCATION`** to the tracker: it replies right away with the current position
- No fix: prints `Waiting for GPS fix...` and sends no SMS
- Prints SIM status, signal quality, network registration and supply voltage at 115200 baud
- **Automatic recovery**: if the modem stops answering AT, loses the network for more than 60 s, keeps failing to send SMS, or reboots by itself (brown-out), it gets a hardware reset on RST and is re-initialised
- The main loop has no `delay()`. Timing uses `millis()`, and the modem is driven by a non-blocking state machine

> ⚠️ **The SIM800L is 2G only (GSM 850/900/1800/1900).** Your carrier must still run a 2G network where you use the tracker, and your SIM must be allowed on it. Many carriers have shut 2G down. If the modem never registers, check this first.

## Project structure

```
esp32-gps-tracker/
├── platformio.ini          # 3 environments: tracker, gps_test, gsm_test
├── README.md
└── src/
    ├── main.cpp            # the tracker firmware
    └── tools/
        ├── gps_test.cpp    # GPS-only test (raw NMEA + parsed data)
        └── gsm_test.cpp    # GSM-only test (diagnostics + AT terminal)
```

## Wiring

| Module   | Module pin | ESP32 pin              | Notes                                         |
|----------|------------|------------------------|-----------------------------------------------|
| NEO-6M   | VCC        | 3V3                    |                                               |
| NEO-6M   | GND        | GND                    |                                               |
| NEO-6M   | TX         | GPIO16 (RX2)           | UART2, 9600 baud                              |
| NEO-6M   | RX         | GPIO17 (TX2)           |                                               |
| SIM800L  | VCC        | **4 V / 2 A supply +** | Not from the ESP32! 1000 µF cap close to VCC  |
| SIM800L  | GND        | supply − **and** ESP32 GND | Grounds must be shared                    |
| SIM800L  | TX         | GPIO26                 | UART1, 9600 baud                              |
| SIM800L  | RX         | GPIO27                 | See the logic-level note below                |
| SIM800L  | RST        | GPIO25                 | Active LOW, driven by the firmware            |

Notes:
- The SIM800L draws current peaks of up to 2 A while transmitting. A weak supply or thin wires cause random reboots (`RDY` in the log, or "modem restarted unexpectedly"). Keep the 1000 µF capacitor right at the module and use short, thick wires.
- The SIM800L's UART runs at 2.8 V. Most breakout boards accept 3.3 V on RX. If yours does not, put a resistor divider on ESP32 TX → SIM800L RX (for example 1 kΩ in series and 2 kΩ to GND).
- Screw on the GSM antenna. Without it the signal (CSQ) stays very low.
- Use a SIM without a PIN, or put the PIN in `SIM_PIN`.

## Required libraries

PlatformIO installs these on its own (`lib_deps` in `platformio.ini`):

| Library | PlatformIO ID | Used for |
|---|---|---|
| TinyGPSPlus | `mikalhart/TinyGPSPlus @ ^1.0.3` | NMEA parsing |
| TinyGSM | `vshymanskyy/TinyGSM @ ^0.12.0` | SIM800L AT commands, SMS |
| StreamDebugger | `vshymanskyy/StreamDebugger @ ^1.0.1` | optional AT command dump |

With the Arduino IDE instead: install **TinyGPSPlus** (Mikal Hart), **TinyGSM** (Volodymyr Shymanskyy) and **StreamDebugger** from the Library Manager, select board *ESP32 Dev Module*, and copy `src/main.cpp` into a `.ino` sketch.

## Configuration

At the top of `src/main.cpp`:

```cpp
static const char OWNER_PHONE_NUMBER[] = "+10000000000";   // your number, international format
static const char SIM_PIN[] = "";                          // SIM PIN, if any
static const bool     PERIODIC_SMS_ENABLED   = true;
static const uint32_t SMS_REPORT_INTERVAL_MS = 30UL * 1000UL;
static const bool ONLY_OWNER_CAN_REQUEST = true;           // answer LOCATION only from your number
```

> 💸 One SMS every 30 s is about 2,880 SMS a day. Check your SIM plan, or raise `SMS_REPORT_INTERVAL_MS` once testing is done.

`ONLY_OWNER_CAN_REQUEST` is on by default, so strangers cannot ask the tracker where it is. Numbers are compared on their last 8 digits, so `+96170123456` and `70123456` count as the same number.

## Build, upload, monitor

```bash
# install PlatformIO Core once:  pip install platformio   (or use the VS Code PlatformIO extension)
cd esp32-gps-tracker
pio run -e tracker -t upload          # build + flash the tracker
pio device monitor                    # 115200 baud
```

If uploading hangs at `Connecting....`, hold the **BOOT** button on the DevKit until the upload starts.

To see every AT command exchanged with the modem, uncomment `build_flags = -DDUMP_AT_COMMANDS` in `[env:tracker]`.

## Testing step by step

### 1. GPS on its own (`gps_test`)

```bash
pio run -e gps_test -t upload -t monitor
```

1. You should see raw NMEA lines (`$GPRMC,...`, `$GPGGA,...`) within a second. If you see nothing, `chars=0` and "No data!", check that GPS **TX** goes to GPIO16, then power and GND.
2. Garbage characters or a rising `bad-checksum` mean the baud rate is wrong. The NEO-6M default is 9600.
3. Take the antenna outdoors or to a window with a clear view of the sky. A cold start takes **1–5 minutes** (sometimes longer the first time). The blue LED on the GPS board starts blinking once it has a fix.
4. When `FIX lat=... lon=...` appears, open the printed Maps link and check that it points to where you are.
5. Type `r` + Enter to hide the raw NMEA and see only the summary.

### 2. GSM on its own (`gsm_test`)

Insert the SIM, attach the antenna, and power the SIM800L from the 4 V supply.

```bash
pio run -e gsm_test -t upload -t monitor
```

The sketch resets the modem and runs a set of diagnostic commands. Here is what each answer should look like:

| Command | Good answer | If not |
|---|---|---|
| `AT` | `OK` | No answer: check TX/RX (they may be swapped) and the power supply |
| `AT+CBC` | `+CBC: 0,xx,3700..4200` (mV) | Below 3500 mV: supply too weak or wires too thin |
| `AT+CPIN?` | `+CPIN: READY` | `SIM PIN`: remove the PIN. `NOT INSERTED`: reseat the SIM |
| `AT+CSQ` | `+CSQ: 10..31,0` | `99`: no signal. Attach the antenna, move to a better spot |
| `AT+CREG?` | `+CREG: 0,1` (home) or `0,5` (roaming) | `0,2` for a long time or `0,3`: **no 2G network** or SIM not allowed on it |
| `AT+COPS?` | `+COPS: 0,0,"Operator"` | |

The onboard LED blinks fast (about once a second) while searching and slowly (about every 3 s) once registered.

The monitor then acts as an AT terminal:
- `!sms +1234567890 hello`: sends a test SMS. You should get `+CMGS: <n>` and `OK`, then the message on your phone.
- Send an SMS to the tracker's SIM. With `AT+CMGF=1` typed first, you will see `+CMTI: "SM",<index>`. Read it with `AT+CMGR=<index>`.
- `!diag` repeats the diagnostics. `!reset` pulses RST (checks the GPIO25 wiring).
- `^Z` sends Ctrl+Z if you type `AT+CMGS` by hand.

### 3. Everything together (`tracker`)

```bash
pio run -e tracker -t upload -t monitor
```

Expected log:

```
[GSM] State RESET_PULSE -> BOOTING
[GSM] State BOOTING -> WAIT_AT
[GSM] Modem answered AT. Initialising...
[GSM] SIM status: READY
[GSM] Signal quality: 18/31 (-77 dBm, good)
[GSM] Network registration: registered (home)
[GSM] ===== GSM READY =====
Waiting for GPS fix... (satellites: 3, ...)
...
------------------- GPS -------------------
Latitude  : 33.888630
...
[SMS] Sending to +1...: Tracker update https://maps.google.com/?q=...
[SMS] Sent OK
```

Checks:
1. **Periodic SMS**: with a fix, you get an SMS every 30 s. Without a fix you see `Waiting for GPS fix... (periodic SMS skipped)` and no SMS is sent.
2. **LOCATION command**: from the owner phone, text `LOCATION` (case does not matter). Within about 5 s the log shows `[SMS] From ...: "LOCATION"` and you get the reply.
3. **Recovery**: briefly unplug the SIM800L power, or take out the antenna for more than a minute. The log shows `*** Hardware reset #n ...` and the modem comes back to `GSM READY` on its own.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Modem resets during "Sending SMS" / `RDY` in the log | Power supply cannot deliver 2 A peaks. Check the capacitor, wire gauge, and the supply set to about 4.0 V |
| `registration DENIED` or `searching...` forever | No 2G in your area, SIM not activated, or SIM not allowed on 2G |
| `Signal quality: unknown (99)` | Antenna missing or poor coverage |
| `No data from the GPS module` | GPS TX not on GPIO16, or no power |
| Fix never arrives indoors | Normal. GPS needs a view of the sky |
| `LOCATION` gets no answer | Sender is not `OWNER_PHONE_NUMBER` (see the log), or there is no GPS fix yet |
