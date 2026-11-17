"""
LIN bus logger for Raspberry Pi Pico (MicroPython)

Listens passively to a LIN bus through a transceiver (TJA1021, MCP2003B, ...),
prints one line per frame over USB serial and, if a microSD card is attached,
writes the same frames to a CSV file (lin_0001.csv, lin_0002.csv, ...).

    time_ms     id  pid  len  data                      chk  cs   status

How it works
  * A PIO state machine acts as the UART receiver. Besides normal bytes it
    detects the LIN *break* (>= 13 dominant bits) and pushes a marker, so
    frame boundaries are exact even if 00 55 shows up inside the data.
  * A hard interrupt moves every received byte into a 2 s ring buffer with a
    microsecond timestamp, so slow SD writes, USB printing or the OLED can
    block the main loop without losing data or skewing timestamps.
  * LIN doesn't put the data length on the wire, so everything between one
    break and the next is one frame: sync, PID, data..., checksum.
  * Checksum is checked as enhanced (LIN 2.x, includes PID) and classic
    (LIN 1.x / diagnostic IDs 0x3C-0x3D) and the matching type is reported.

Buttons
  * GP15 to GND: clear counters and timestamps and start a new log file.
  * RUN (pin 30) to GND: hardware reset, no code needed.

SIMULATE = True drives fake LIN traffic out of GP4 (second PIO state machine)
so the whole thing runs in Wokwi or on a bench: wire GP4 -> GP5.
"""

LIN_BAUD = 19200        # common: 19200, 10417, 9600
RX_PIN = 5              # transceiver RXD -> GP5
SIMULATE = True         # fake LIN master on GP4 (wire GP4 -> GP5)
SIM_TX_PIN = 4
OLED_SDA, OLED_SCL = 8, 9           # optional SSD1306 128x64 status display
BTN_RESET_PIN = 15                  # button to GND: new log + clear counters
SD_SCK, SD_MOSI, SD_MISO, SD_CS = 10, 11, 12, 13   # optional microSD (SPI1)
SD_FLUSH_MS = 2000      # write buffered lines to the card this often
RING = 4096             # capture buffer in bytes-on-the-wire (power of 2)

BIT_US = 1_000_000 // LIN_BAUD
IDLE_US = 200 * BIT_US  # ~ max LIN frame time; ends the last frame of a burst
BREAK = 0x1FF           # marker the PIO pushes for a break


# ---------------------------------------------------------------- LIN helpers
def make_pid(fid):
    """6-bit frame ID -> protected ID with parity bits P0 (b6) and P1 (b7)."""
    b = [(fid >> n) & 1 for n in range(6)]
    p0 = b[0] ^ b[1] ^ b[2] ^ b[4]
    p1 = 1 ^ (b[1] ^ b[3] ^ b[4] ^ b[5])
    return (fid & 0x3F) | (p0 << 6) | (p1 << 7)


def lin_checksum(data, seed=0):
    """Classic: seed=0. Enhanced: seed=PID."""
    s = seed
    for x in data:
        s += x
        if s > 0xFF:
            s -= 0xFF
    return (~s) & 0xFF


def decode(fr):
    """fr = bytes received after a break: sync, pid, data..., checksum.
    Returns (fid, pid, data, chk, cs_type, status)."""
    if not fr:
        return None, None, b"", None, "", "BREAK_ONLY"
    if fr[0] != 0x55:
        return None, None, bytes(fr), None, "", "SYNC_ERR"
    if len(fr) < 2:
        return None, None, b"", None, "", "NO_PID"
    pid = fr[1]
    fid = pid & 0x3F
    parity_ok = make_pid(fid) == pid
    body = fr[2:]
    if not body:
        return fid, pid, b"", None, "", "NO_RESP" if parity_ok else "PARITY_ERR"
    if len(body) == 1:
        return fid, pid, b"", body[0], "", "SHORT"
    data, chk = bytes(body[:-1]), body[-1]
    if fid < 0x3C and chk == lin_checksum(data, pid):
        cs = "enh"
    elif chk == lin_checksum(data):
        cs = "cls"
    else:
        cs = "?"
    if not parity_ok:
        status = "PARITY_ERR"
    elif cs == "?":
        status = "CHECKSUM_ERR"
    elif len(data) > 8:
        status = "TOO_LONG"
    else:
        status = "OK"
    return fid, pid, data, chk, cs, status


def h2(v):
    return "--" if v is None else "%02X" % v


def format_lines(t_us, fr):
    """-> (console line, CSV line, status). Integer math only: MicroPython
    floats are single precision and would lose resolution on long logs."""
    fid, pid, data, chk, cs, status = decode(fr)
    hexdata = " ".join("%02X" % x for x in data)
    t = "%d.%d" % (t_us // 1000, (t_us // 100) % 10)
    con = "%10s  %s   %s  %3d  %-24s  %s   %-3s  %s" % (
        t, h2(fid), h2(pid), len(data), hexdata or "-", h2(chk), cs, status)
    csv = "%s,%s,%s,%d,%s,%s,%s,%s\n" % (
        t, h2(fid), h2(pid), len(data), hexdata, h2(chk), cs, status)
    return con, csv, status


HEADER = "   time_ms  id  pid  len  data                      chk  cs   status"
CSV_HEADER = "time_ms,id,pid,len,data,chk,cs,status\n"


# ------------------------------------------------------------------- hardware
def main():
    import time
    import array
    import rp2
    import framebuf
    from machine import Pin, I2C, SPI, mem32

    # PIO UART receiver with break detection. Pushes each data byte as
    # 0x00-0xFF and a break (all-zero byte + low stop bit) as 0x1FF, then
    # raises an IRQ. Other framing errors are dropped.
    @rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, fifo_join=rp2.PIO.JOIN_RX)
    def lin_rx():
        label("start")
        wait(0, pin, 0)
        set(x, 7)               .delay(10)
        label("bitloop")
        in_(pins, 1)
        jmp(x_dec, "bitloop")   .delay(6)
        jmp(pin, "good_stop")
        mov(x, isr)
        jmp(not_x, "brk")
        mov(isr, null)
        wait(1, pin, 0)
        jmp("start")
        label("brk")
        mov(isr, invert(null))
        in_(null, 23)           # 0xFFFFFFFF >> 23 = 0x1FF
        push()
        irq(rel(0))
        wait(1, pin, 0)
        jmp("start")
        label("good_stop")
        in_(null, 24)           # move the byte down to bits 7..0
        push()
        irq(rel(0))

    rx_pin = Pin(RX_PIN, Pin.IN, Pin.PULL_UP)
    sm = rp2.StateMachine(0, lin_rx, freq=8 * LIN_BAUD,
                          in_base=rx_pin, jmp_pin=rx_pin)
    PIO0_FDEBUG = 0x50200000 + 0x008   # RXSTALL bit 0 = SM0 FIFO overflowed

    # Ring buffer filled by a hard IRQ: value + arrival time for every byte.
    MASK = RING - 1
    ring = array.array("H", bytes(2 * RING))
    tring = array.array("I", bytes(4 * RING))
    st = array.array("I", [0, 0, 0])   # head, tail, dropped

    # Everything is passed as default args: a closure over this many names
    # would allocate on each call, which a hard IRQ isn't allowed to do.
    def on_rx(_, sm=sm, st=st, ring=ring, tring=tring, mask=MASK,
              ticks_us=time.ticks_us):
        while sm.rx_fifo():
            w = sm.get()
            h = st[0]
            n = (h + 1) & mask
            if n == st[1]:
                st[2] += 1
            else:
                ring[h] = w
                tring[h] = ticks_us()
                st[0] = n

    sm.irq(on_rx, hard=True)
    sm.active(1)

    # ---- optional fake LIN master: a second PIO state machine that clocks
    # out raw line bits (break, delimiter, start/data/stop) prepared in Python.
    @rp2.asm_pio(out_init=rp2.PIO.OUT_HIGH, out_shiftdir=rp2.PIO.SHIFT_RIGHT,
                 autopull=True, pull_thresh=32, fifo_join=rp2.PIO.JOIN_TX)
    def bits_tx():
        out(pins, 1)            .delay(7)

    class SimMaster:
        def __init__(self):
            self.sm = rp2.StateMachine(1, bits_tx, freq=8 * LIN_BAUD,
                                       out_base=Pin(SIM_TX_PIN))
            self.sm.active(1)
            self.next = time.ticks_add(time.ticks_ms(), 500)
            self.n = 0

        def send(self, pkt, break_bits=13):
            v, n = 1 << break_bits, break_bits + 1   # break + 1-bit delimiter
            for b in pkt:                            # start(0) data stop(1)
                v |= ((b << 1) | 0x200) << n
                n += 10
            words = (n + 31) // 32
            v |= ((1 << (words * 32 - n)) - 1) << n  # pad with idle (1)
            for _ in range(words):
                self.sm.put(v & 0xFFFFFFFF)
                v >>= 32

        def step(self):
            if time.ticks_diff(time.ticks_ms(), self.next) < 0:
                return
            if self.sm.tx_fifo():
                return
            self.next = time.ticks_add(self.next, 25)
            n = self.n
            self.n += 1
            slot = n % 6
            if slot == 0:    # status frame, 4 bytes, changing
                fid, data = 0x10, bytes((n & 0xFF, (n >> 8) & 0xFF, 0x3C, 0x80))
            elif slot == 1:  # tricky payload containing 00 55
                fid, data = 0x21, bytes((0x00, 0x55, 0xAA, 0xFF))
            elif slot == 2:  # slave that doesn't answer
                fid, data = 0x31, None
            elif slot == 3:  # 8-byte frame
                fid, data = 0x2A, bytes(range(n & 0x0F, (n & 0x0F) + 8))
            elif slot == 4:  # diagnostic request, classic checksum
                fid, data = 0x3C, bytes((0x7F, 0x06, 0xB2, 0x00, 0xFF, 0x7F, 0xFF, 0xFF))
            else:            # 2-byte frame
                fid, data = 0x05, bytes((0x01, n & 0xFF))
            pid = make_pid(fid)
            pkt = bytearray((0x55, pid))
            if data is not None:
                cs = lin_checksum(data, 0 if fid >= 0x3C else pid)
                if n % 37 == 36:
                    cs ^= 0x01            # inject an occasional bad checksum
                pkt += data + bytes((cs,))
            self.send(pkt)

    sim = SimMaster() if SIMULATE else None

    # ---- optional SSD1306, refreshed in small chunks to keep the loop snappy
    class Oled(framebuf.FrameBuffer):
        def __init__(self, i2c, addr):
            self.i2c, self.addr = i2c, addr
            self.buf = bytearray(1024)
            self.mv = memoryview(self.buf)
            super().__init__(self.buf, 128, 64, framebuf.MONO_VLSB)
            self.cmd(0xAE, 0x20, 0x00, 0x40, 0xA1, 0xA8, 63, 0xC8, 0xD3, 0x00,
                     0xDA, 0x12, 0xD5, 0x80, 0xD9, 0xF1, 0xDB, 0x30, 0x81,
                     0xFF, 0xA4, 0xA6, 0x8D, 0x14, 0xAF)
            self.chunk = -1

        def cmd(self, *c):
            self.i2c.writeto(self.addr, bytes((0x00,) + c))

        def start_refresh(self):
            self.chunk = 0

        def step(self):      # send 64 bytes per call, 16 calls per screen
            if self.chunk < 0:
                return
            page, half = self.chunk >> 1, self.chunk & 1
            col = half * 64
            self.cmd(0x21, col, col + 63, 0x22, page, page)
            off = page * 128 + col
            self.i2c.writevto(self.addr, (b"\x40", self.mv[off:off + 64]))
            self.chunk += 1
            if self.chunk == 16:
                self.chunk = -1

    oled = None
    try:
        i2c = I2C(0, sda=Pin(OLED_SDA), scl=Pin(OLED_SCL), freq=400_000)
        addrs = i2c.scan()
        if 0x3C in addrs or 0x3D in addrs:
            oled = Oled(i2c, 0x3C if 0x3C in addrs else 0x3D)
    except Exception as e:
        print("OLED disabled:", e)

    # ---- optional microSD logging (needs sdcard.py on the Pico)
    class SdLog:
        def __init__(self):
            self.ok = False
            self.f = None
            self.name = ""
            self.lines = []
            self.next_flush = time.ticks_ms()
            try:
                import os
                import sdcard
                self.os = os
                spi = SPI(1, baudrate=1_000_000, sck=Pin(SD_SCK),
                          mosi=Pin(SD_MOSI), miso=Pin(SD_MISO))
                card = sdcard.SDCard(spi, Pin(SD_CS, Pin.OUT), baudrate=4_000_000)
                os.mount(card, "/sd")
                self.ok = True
                self.new_file()
            except Exception as e:
                self.ok = False
                self.name = "off"
                print("SD logging disabled:", e)

        def new_file(self):
            if not self.ok:
                return
            self.flush()
            if self.f:
                self.f.close()
            nums = [int(n[4:8]) for n in self.os.listdir("/sd")
                    if n.startswith("lin_") and n.endswith(".csv") and n[4:8].isdigit()]
            self.name = "lin_%04d.csv" % (max(nums) + 1 if nums else 1)
            self.f = open("/sd/" + self.name, "w")
            self.f.write(CSV_HEADER)
            self.f.flush()

        def add(self, line):
            if self.ok:
                self.lines.append(line)

        def flush(self):
            if not (self.ok and self.f and self.lines):
                return
            try:
                self.f.write("".join(self.lines))
                self.f.flush()
                self.lines = []
            except Exception as e:
                self.ok = False
                self.name = "ERROR"
                print("!! SD write failed, logging stopped:", e)

        def step(self, ms):
            if self.lines and (len(self.lines) >= 100 or
                               time.ticks_diff(ms, self.next_flush) >= 0):
                self.flush()
                self.next_flush = time.ticks_add(ms, SD_FLUSH_MS)

    sd = SdLog()
    btn = Pin(BTN_RESET_PIN, Pin.IN, Pin.PULL_UP)

    # ---- main loop
    time.sleep(0.1)
    print("LIN logger @ %d baud, RX on GP%d%s" % (
        LIN_BAUD, RX_PIN, ", SIMULATE (GP4 -> GP5)" if SIMULATE else ""))
    if sd.ok:
        print("logging to /sd/" + sd.name)
    print(HEADER)

    # ticks_us() wraps every ~18 min, so keep an extended microsecond clock
    clk = [time.ticks_us(), 0]          # last ticks value, extended us

    def ext(t):
        clk[1] += time.ticks_diff(t, clk[0])
        clk[0] = t
        return clk[1]

    t0 = ext(time.ticks_us())
    frame = None
    t_frame = 0
    last_rx = time.ticks_us()
    frames = errors = overruns = 0
    dropped_seen = lost_fifo = 0
    warn_t = time.ticks_ms()
    last_id = None
    last_status = ""
    fps_count, fps, fps_t = 0, 0, time.ticks_ms()
    next_draw = time.ticks_ms()
    btn_prev, btn_t = 1, 0

    def emit(t, fr):
        nonlocal frames, errors, last_id, last_status, fps_count
        con, csv, status = format_lines(ext(t) - t0, fr)
        print(con)
        sd.add(csv)
        frames += 1
        fps_count += 1
        if status not in ("OK", "NO_RESP"):
            errors += 1
        if len(fr) > 1:
            last_id = fr[1] & 0x3F
        last_status = status

    while True:
        ext(time.ticks_us())                 # keep the extended clock fresh
        # 1) frames out of the ring buffer
        tl = st[1]
        while tl != st[0]:
            w = ring[tl]
            t = tring[tl]
            tl = (tl + 1) & MASK
            st[1] = tl
            last_rx = t
            if w == BREAK:
                if frame is not None:
                    emit(t_frame, frame)
                frame = bytearray()
                t_frame = t
            elif frame is not None:
                frame.append(w)
                if len(frame) > 12:          # runaway: no break seen
                    emit(t_frame, frame)
                    frame = None

        if frame is not None and time.ticks_diff(time.ticks_us(), last_rx) > IDLE_US:
            emit(t_frame, frame)
            frame = None

        ms = time.ticks_ms()

        # 2) data-loss checks (reported at most once per second)
        if mem32[PIO0_FDEBUG] & 1:
            mem32[PIO0_FDEBUG] = 1           # write-1-to-clear
            lost_fifo += 1
        if (lost_fifo or st[2] != dropped_seen) and time.ticks_diff(ms, warn_t) >= 0:
            warn_t = time.ticks_add(ms, 1000)
            overruns += 1
            print("!! data lost: PIO FIFO overflowed %d times, ring buffer dropped %d bytes"
                  % (lost_fifo, st[2] - dropped_seen))
            lost_fifo, dropped_seen = 0, st[2]

        # 3) reset button: clear counters, restart time, new log file
        b = btn.value()
        if b != btn_prev and time.ticks_diff(ms, btn_t) > 50:
            btn_prev, btn_t = b, ms
            if b == 0:
                if frame is not None:
                    emit(t_frame, frame)
                    frame = None
                sd.new_file()
                frames = errors = overruns = 0
                last_id, last_status = None, ""
                t0 = ext(time.ticks_us())
                print("# reset%s" % (" - logging to /sd/" + sd.name if sd.ok else ""))
                print(HEADER)

        # 4) housekeeping
        if sim:
            sim.step()
        sd.step(ms)

        if time.ticks_diff(ms, fps_t) >= 1000:
            fps, fps_count, fps_t = fps_count, 0, ms

        if oled:
            if oled.chunk < 0 and time.ticks_diff(ms, next_draw) >= 0:
                next_draw = time.ticks_add(ms, 500)
                oled.fill(0)
                oled.text("LIN %d bd" % LIN_BAUD, 0, 0)
                oled.hline(0, 10, 128, 1)
                oled.text("frm %d %d/s" % (frames, fps), 0, 14)
                oled.text("err %d ovr %d" % (errors, overruns), 0, 24)
                oled.text("last %s %s" % (h2(last_id), last_status[:7]), 0, 34)
                oled.text("sd %s" % sd.name, 0, 44)
                oled.text("up %ds" % ((ext(time.ticks_us()) - t0) // 1_000_000), 0, 54)
                oled.start_refresh()
            oled.step()


if __name__ == "__main__":
    main()
