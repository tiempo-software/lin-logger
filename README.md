# lin-logger

A passive LIN bus logger for the Raspberry Pi Pico, written in MicroPython.
It listens to a LIN bus through a transceiver, prints one line per frame over
USB serial, and saves the frames to CSV files on a microSD card. An optional
SSD1306 OLED shows live stats.

```
   time_ms  id  pid  len  data                      chk  cs   status
    2118.3  2A   6A    8  05 06 07 08 09 0A 0B 0C   51   enh  OK
    2143.3  3C   3C    8  7F 06 B2 00 FF 7F FF FF   48   cls  OK
    2218.0  21   61    4  00 55 AA FF               9F   ?    CHECKSUM_ERR
    2242.8  31   B1    0  -                         --        NO_RESP
```

## Features

- **Exact frame boundaries.** A PIO state machine works as the UART receiver
  and also detects the LIN break (13 or more dominant bits). Frames split
  correctly even when the data contains `00 55`.
- **No lost bytes, accurate timestamps.** A hard interrupt timestamps each byte
  to the microsecond and moves it into a 2-second buffer the moment it
  arrives. SD writes, USB printing and the display can stall the main loop
  without dropping data or skewing times.
- **Checksum detection.** Each frame is checked against the enhanced
  (LIN 2.x) and classic (LIN 1.x and diagnostic IDs `0x3C`/`0x3D`) checksums,
  and the line shows which one matched.
- **Error reporting.** Flags bad ID parity, bad sync, bad checksum, headers
  with no slave response, and any data loss.
- **SD card logging.** Frames are written to `lin_0001.csv`, `lin_0002.csv`,
  and so on. Each boot, or each press of the New log button, starts a new
  file.
- **Built-in traffic generator.** `SIMULATE = True` sends test frames out
  of GP4, so you can test with no car and no transceiver, in Wokwi or on a
  bare Pico.
- **Optional OLED.** Shows frame count and rate, errors, data loss, the last
  ID, the current log file and uptime.

## Files

| File | What it is |
|---|---|
| `main.py` | The logger. Runs on boot. |
| `sdcard.py` | SD card driver from [micropython-lib](https://github.com/micropython/micropython-lib) (MIT License), with one fix for cards that reject the block-size command. Only needed for SD logging. |
| `diagram.json` | Wokwi simulator wiring |

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi Pico (RP2040) | Any Pico or Pico W running MicroPython 1.2x |
| LIN transceiver | TJA1021, TJA1020 or MCP2003B, or a breakout module built on one |
| microSD card module (SPI) | Optional. The card must be formatted FAT32. |
| SSD1306 128×64 I²C OLED | Optional |
| 2 momentary push buttons | Optional: New log and Reset |

### Wiring

**LIN transceiver**

| Transceiver | Pico | Notes |
|---|---|---|
| RXD | GP5 | Open-drain output. The Pico's internal pull-up is enabled. |
| TXD | 3V3 (via 10k) | Holds the transmitter recessive so the logger never drives the bus |
| SLP_N / EN | 3V3 | Keeps the transceiver in normal mode |
| VBAT | Vehicle 12 V | |
| GND | GND | Must share ground with the vehicle or ECU |
| LIN | LIN bus wire | |

**microSD module (SPI1)**

| SD module | Pico |
|---|---|
| SCK | GP10 |
| MOSI / DI | GP11 |
| MISO / DO | GP12 |
| CS | GP13 |
| VCC | 3V3, or VSYS for modules with an onboard 5 V regulator |
| GND | GND |

**OLED (I²C0)**

| OLED | Pico |
|---|---|
| SDA | GP8 |
| SCL | GP9 |
| VCC | 3V3 |
| GND | GND |

**Buttons**

| Button | Wiring | What it does |
|---|---|---|
| New log | GP15 to GND | Closes the current file, starts the next one, and resets the counters and the clock. Handled in code. |
| Reset | RUN (pin 30) to GND | Hardware reboot of the Pico. No code involved. |

The logger only listens and sends nothing, but double-check the TXD and
SLP_N wiring before connecting it to a vehicle.

### Power

- **USB:** the simplest option, from a laptop or a USB power bank.
- **Battery:** connect 1.8–5.5 V to VSYS, for example a single LiPo cell or
  3×AA.
- **Vehicle 12 V:** never connect 12 V straight to the Pico. Use a buck
  converter from 12 V down to 5 V into VSYS. Car power is noisy and can
  spike well above 12 V, so pick a converter rated for automotive use, with
  reverse-polarity and load-dump protection. If you also plug in USB while
  powered from VSYS, put a Schottky diode in series with the converter
  output, as the Pico datasheet describes.

Without a laptop attached, the SD card is the record and the OLED shows the
live status.

## Usage

1. Flash MicroPython onto the Pico.
2. Set the options at the top of `main.py`:

   ```python
   LIN_BAUD = 19200   # common: 19200, 10417, 9600
   RX_PIN = 5
   SIMULATE = False   # True = fake traffic on GP4 (jumper GP4 -> GP5)
   ```

3. Copy the files to the board:

   ```sh
   mpremote cp main.py sdcard.py :
   mpremote reset
   ```

4. Watch the output live, or save it from the USB side as well:

   ```sh
   mpremote connect auto repl --capture lin.log
   ```

   Any serial terminal works too (Thonny, `screen`, PuTTY).

### Try it without hardware

- **Wokwi.** Create a MicroPython Pi Pico project at
  [wokwi.com](https://wokwi.com). Paste in `main.py` and `diagram.json`, and
  add `sdcard.py` as a new file. Leave `SIMULATE = True`. The diagram includes
  the GP4→GP5 jumper, the OLED, an SD card and the New log button. Wokwi
  doesn't simulate the RUN pin, so use its own restart button instead.
- **Bare Pico.** Set `SIMULATE = True` and put a jumper from GP4 to GP5.

## Output

The console and the CSV hold the same fields. The CSV header is:

```
time_ms,id,pid,len,data,chk,cs,status
```

| Column | Meaning |
|---|---|
| `time_ms` | Time of the break since boot or the last New log press, with 0.1 ms resolution |
| `id` | 6-bit frame ID |
| `pid` | Protected ID as seen on the wire (ID plus parity bits) |
| `len` | Number of data bytes |
| `data` | Payload bytes |
| `chk` | Checksum byte |
| `cs` | Checksum type that matched: `enh` (enhanced), `cls` (classic) or `?` |

| Status | Meaning |
|---|---|
| `OK` | Valid frame |
| `NO_RESP` | Header with no response; the slave didn't answer |
| `CHECKSUM_ERR` | Neither checksum type matches |
| `PARITY_ERR` | PID parity bits are wrong |
| `SYNC_ERR` | The byte after the break wasn't `0x55` |
| `SHORT` | Only one byte after the header |
| `TOO_LONG` | More than 8 data bytes |
| `BREAK_ONLY` / `NO_PID` | The frame was cut off |

If bytes are ever lost, a `!! data lost` line is printed (at most once per
second) and the OLED's `ovr` count goes up.

On the SD card, lines are buffered and written every 2 seconds
(`SD_FLUSH_MS`). A power cut loses at most the last 2 seconds. If the card is
removed or a write fails, SD logging stops (the OLED shows `sd ERROR`) and USB
output carries on. Reboot to start logging again.

## Limitations

- **Data length is inferred.** LIN doesn't send the length, so the logger
  takes everything between two breaks as one frame. Signal decoding needs
  the bus's LDF file, which isn't supported yet.
- **Fixed baud rate.** The rate is set in code and there's no auto-baud. The
  8× oversampling receiver tolerates about ±4 % error, which covers master
  and synced slaves.
- **Relative timestamps.** Times are counted from boot. The Pico has no
  real-time clock, so files carry no date or time of day.

## Ideas / roadmap

- Auto-baud detection from the sync byte
- LDF import to decode signals by name
- Filtering by frame ID
- Real-time clock module for wall-clock timestamps
