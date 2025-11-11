# TBBNB Controller  
**ESP32 DOIT DevKit V1 + OLED SSD1306 + Elechouse PN532**

Smart geocache access controller with NFC authorization, OLED display, quiet-hour power management, and admin configuration via Serial or NFC tags.

---

## � Overview

### Daytime Operation
- PN532 is **active** and polled (throttled).
- After `active_ms` of inactivity → OLED **turns off**.
- After `sleep_ms` → ESP32 enters **daytime deep sleep** with PN532 **AutoPoll** and **IRQ wake** (EXT0 GPIO33 LOW).
- IRQ wake triggers instantly when a tag is scanned.

### Night (Quiet Hours)
- During `quiet` hours (default **22:00 → 07:00**) the ESP32 enters **true deep sleep**:
  - PN532 is **held in reset (RSTPDN LOW)**.
  - No IRQ wake (timer-only wake at QUIET_END_H).
  - Power consumption is minimal.

---

## ⚙️ Hardware Wiring

| Component | ESP32 Pin | Description |
|------------|------------|--------------|
| OLED (SSD1306 I²C) | SDA = 21, SCL = 22 | Shared I²C bus |
| PN532 | SDA = 21, SCL = 22 | Elechouse PN532 (I²C mode) |
| PN532 IRQ | GPIO33 | RTC-capable wake input (EXT0 LOW) |
| PN532 RSTPDN | GPIO32 | Active LOW — pulled LOW at night |
| Power | 3.3V / GND | Common ground with ESP32 |

---

## �️ Serial Command Reference

Open Serial Monitor at **115200 baud**.  
Type `help` to see all commands.

### � Basic
| Command | Description |
|----------|-------------|
| `help` | Show all commands |
| `show` | Print current settings, quiet hours, time validity |
| `list` | List all authorized tag UIDs |
| `add <UIDHEX>` | Add tag UID (e.g., `add 04AABBCCDD`) |
| `remove <UIDHEX>` | Remove tag UID |
| `clear` | Clear allowlist |

### � Tag Helpers
| Command | Description |
|----------|-------------|
| `last` | Show last scanned UID + recognition status |
| `lastgranted` | Show last UID that granted access |
| `addlast` | Add last scanned tag to allowlist |

---

### � Timing & Display Parameters
| Command | Description | Units / Default |
|----------|-------------|-----------------|
| `active_ms <ms>` | OLED off after inactivity | 120000 |
| `sleep_ms <ms>` | Daytime deep-sleep after inactivity | 180000 |
| `day_fallback_ms <ms>` | Optional timer wake (0=disabled) | 0 |
| `welcome_ms <ms>` | “Welcome Home” duration | 3000 |
| `reject_ms <ms>` | “Poop” duration | 3000 |
| `marquee_ms <ms>` | Border animation speed | 35 |
| `code_delay_ms <ms>` | Delay before unlock code | 1500 |
| `unlock <4digits>` | Set unlock code | 3510 |

---

### � Quiet Hours & Time
| Command | Description | Example |
|----------|-------------|----------|
| `quiet on` | Enable quiet hours (22→7 default) | |
| `quiet off` | Disable quiet hours | |
| `quiet on <start> <end>` | Set start & end hours | `quiet on 22 7` |
| `time` or `date` | Show current local time | |
| `ntp` | Sync via Wi-Fi (uses stored SSID/PASS) | |
| `settime YYYY-MM-DD HH:MM:SS` | Set clock manually | `settime 2025-11-11 21:05:00` |
| `setepoch <unix_seconds>` | Set from UNIX timestamp | `setepoch 1731365400` |
| `settz <POSIX_TZ>` | Set and persist timezone | `settz EST5EDT,M3.2.0/2,M11.1.0/2` |
| `tz` | Show current timezone | |

---

### � PN532 / I²C / Diagnostics
| Command | Description |
|----------|-------------|
| `i2cscan` | Scan I²C bus for devices |
| `pnreset` | Hard-reset the PN532 |
| `pninfo` | Print PN532 firmware info |
| `irq` | Show PN532 IRQ pin level |
| `sleepdiag` | Show quiet/time/IRQ diagnostics |
| `sleepnow` | Force immediate night sleep |

---

## � NFC Admin Tag Configuration

Admin tags are standard **NDEF Text** records containing one or more `key=value` pairs.  
Each line, semicolon (`;`), or comma (`,`) separates keys.  
Use the **NFC Tools** app → “Write” → “Add a record” → “Text record”.

---

### � Available Keys

#### General
```
active_ms=<ms>
sleep_ms=<ms>
day_fallback_ms=<ms>
welcome_ms=<ms>
reject_ms=<ms>
marquee_ms=<10..65535>
code_delay_ms=<ms>
code=<4digits>
```

#### Quiet Hours
```
quiet=on|off|1|0|true|false
qstart=<0..23>
qend=<0..23>
```

#### Time & Timezone
```
datetime=YYYY-MM-DD HH:MM:SS
epoch=<unix_seconds>
tz=<POSIX_TZ>
```

#### Allowlist Management
```
add=<UIDHEX>
remove=<UIDHEX>
clear=1
```

---

### � Admin Tag Examples

**Set display and unlock code**
```
welcome_ms=3000
reject_ms=2000
code_delay_ms=1500
code=1234
```

**Quiet hours and timezone**
```
quiet=on
qstart=22
qend=7
tz=EST5EDT,M3.2.0/2,M11.1.0/2
```

**Manual time set**
```
datetime=2025-11-11 21:07:00
```

**Add tags**
```
add=04AABBCCDD
add=0455667788
```

**Reset / clear configuration**
```
clear=1
quiet=off
```

### �️ Feedback
- On scan, the OLED briefly displays:
  - `ADMIN OK / <key>`  
  - or `ADMIN ERR / <reason>`

---

## �️ Operation Flow

### Daytime
1. PN532 active, polling.  
2. Tag scanned → UID read →  
   - **Authorized:** “Welcome Home” → “CODE: ####”  
   - **Unauthorized:** Poop animation.  
3. OLED blanks after `active_ms`.  
4. Device enters deep sleep after `sleep_ms` inactivity (IRQ wake armed).

### Night (Quiet Hours)
- Enters deep sleep until next `QUIET_END_H` (default 07:00).  
- PN532 reset LOW, IRQ held HIGH, no polling.  
- Wake by **timer only**.

---

## � Troubleshooting

| Symptom | Likely Cause | Fix |
|----------|--------------|-----|
| All tags grant access | Allowlist empty | Add tag (`addlast`) or admin tag `add=<UID>` |
| Unknown tag shows Welcome | Ensure allowlist not empty |
| PN532 not found | Wiring/SDA/SCL or power issue |
| “time unknown” | Set manually (`settime`) or use `ntp` |
| Immediate reboot after sleep | IRQ line low; confirm PN532 IRQ idle HIGH before sleep |
| I²C spam | Fixed via `esp_log_level_set("i2c", ESP_LOG_NONE)` |

---

## � Files

| File | Description |
|------|--------------|
| `TBBNB.ino` | Main program |
| `slewfoot_sign_128x64.h` | Bitmap sign for OLED display |
| `README.md` | This documentation |

---

## �‍� Credits

Developed for **Slewfoot’s Travel Bug Bed & Breakfast** Geocache Project.  
Firmware and documentation by **Slewfoot + ChatGPT (2025)**.
