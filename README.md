# GPS Track for Flipper Zero

A GPS navigation application for Flipper Zero with breadcrumb tracking, POI navigation, and LED guidance. Built for [Unleashed firmware](https://github.com/DarkFlippers/unleashed-firmware).

## Hardware

Connect a NMEA 0183 GPS module to Flipper Zero GPIO:

| GPS Pin | Flipper Zero Pin |
|---------|-----------------|
| VCC     | Pin 9 (3.3V)    |
| GND     | Pin 11 (GND)    |
| TX      | Pin 14 (RX)     |
| RX      | Pin 13 (TX) — optional, needed for PMTK commands |

**Tested modules:** Quescan G10A F30 (AG3335M), GY-NEO6MV2 (u-blox NEO-6M)

Default baud rate: **9600**. Use Settings to change if your module uses a different rate (e.g. 38400 for AG3335M).

## Features

### Screens

**Main screen** — latitude, longitude, speed, course, altitude, satellites, time (UTC with timezone offset)

**Breadcrumbs (CRUMBS)** — follow your recorded track back:
- Direction arrow (compass when stationary, relative bearing when moving)
- Distance to next waypoint and total remaining
- Time stamp on each waypoint
- Auto-advance at 30m or after 30s within 60m

**POI Navigation (NAV)** — navigate to saved points of interest:
- Course, speed, ETA
- Direction arrow same as CRUMBS

**Settings** — speed units, baud rate, deep sleep, timezone, NMEA dump

### Navigation

```
Ring (Long ← / →):  NORMAL ↔ NAV ↔ CRUMBS
Back:               Any screen → NORMAL
```

| Screen | Action | Description |
|--------|--------|-------------|
| NORMAL | Long ↑ | Save HOME POI (current position) |
| NORMAL | Long ↓ | Save new POI (current position) |
| NORMAL | OK | Open Settings |
| CRUMBS | Long OK | Crumbs menu (delete/clear/last point) |
| CRUMBS | ↑ / ↓ | Navigate waypoints |
| CRUMBS | OK | Toggle total distance display |
| NAV | Long ↑ | Select POI from list |
| NAV | Long ↓ | Enter coordinates manually |
| NAV | OK | Toggle speed / ETA display |
| NAV list | OK | Select POI as target |
| NAV list | Long ← | Delete POI (with confirmation) |
| NAV list | Long → | Rename POI |

### LED Status Indicators

**Main screen:**
| Color | Pattern | Meaning |
|-------|---------|---------|
| Red | 1× per second | No GPS signal |
| Blue | 2× per second | GPS data received, no fix yet |
| Green | 1× per 3 seconds | GPS fix acquired |

**Navigation (CRUMBS / NAV):**
| Color | Meaning |
|-------|---------|
| Green | On course (< 15° error) |
| Yellow | Slight left (15–45°) |
| Cyan | Slight right (15–45°) |
| Magenta | Far left (45–90°) |
| Blue | Far right (45–90°) |
| Red | Wrong direction (> 90°) |
| White | Waypoint switched |

LED turns off when stopped (< 0.5 m/s), on charger, or outside navigation screens.

### Breadcrumb Recording

- First waypoint: **1 minute** after first GPS fix
- Subsequent waypoints: every **10 minutes**
- Minimum distance between waypoints: **50 meters**
- Recording pauses while viewing CRUMBS screen
- Time stored with each waypoint (local time with TZ offset)

### NMEA Dump (Diagnostics)

Settings → NMEA dump → toggle ON → wait for 30 lines → OK to save.

File saved to `/ext/apps_data/gps_track/nmea_dump.txt`.

Diagnosis shown after saving:
- `NMEA OK + GPS fix!` — module working, coordinates acquired
- `NMEA OK, no fix yet` — module working, waiting for satellites
- `Wrong baudrate (binary)` — change baud rate in Settings
- `No NMEA ($) found` — unexpected data format
- `No RX data` — check wiring

## Building

```bash
# Install ufbt
pip install ufbt

# Update SDK for Unleashed firmware
py -m ufbt update --index-url=https://up.unleashedflip.com/directory.json

# Build
cd flipperzero-gps-track
py -m ufbt

# Build and flash
py -m ufbt launch
```

## Data Files

All files stored on SD card at `/ext/apps_data/gps_track/`:

| File | Format | Description |
|------|--------|-------------|
| `track.csv` | `lat,lon,HH:MM` | Breadcrumb waypoints |
| `poi.csv` | `icon,lat,lon,name` | Points of interest (icon: 0=pin, 1=home) |
| `state.txt` | `crumbs_target,poi_target` | Current navigation state |
| `settings.csv` | `speed_unit,tz_offset,baudrate_idx,deepsleep` | App settings |
| `nmea_dump.txt` | raw NMEA | Diagnostic dump (on demand) |

## Credits

NMEA parsing by [minmea](https://github.com/kosma/minmea).
Based on [flipperzero-gps](https://github.com/ezod/flipperzero-gps) by ezod.
