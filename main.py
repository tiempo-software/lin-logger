"""
LIN bus logger for Raspberry Pi Pico (MicroPython)

Listens passively to a LIN bus through a transceiver (TJA1021, MCP2003B, ...)
and prints one line per frame over USB serial:

    time_ms     id  pid  len  data                      chk  cs   status

How it works
  * A PIO state machine acts as the UART receiver. Besides normal bytes it
    detects the LIN *break* (>= 13 dominant bits) and pushes a marker, so
    frame boundaries are exact even if 00 55 shows up inside the data.
  * LIN doesn't put the data length on the wire, so everything between one
    break and the next is one frame: sync, PID, data..., checksum.
  * Checksum is checked as enhanced (LIN 2.x, includes PID) and classic
    (LIN 1.x / diagnostic IDs 0x3C-0x3D) and the matching type is reported.

SIMULATE = True drives fake LIN traffic out of GP4 (second PIO state machine) so the
whole thing runs in Wokwi or on a bench: wire GP4 -> GP5.
"""

LIN_BAUD = 19200        # common: 19200, 10417, 9600
RX_PIN = 5              # transceiver RXD -> GP5
SIMULATE = True         # fake LIN master on GP4 (wire GP4 -> GP5)
SIM_TX_PIN = 4
OLED_SDA, OLED_SCL = 8, 9   # optional SSD1306 128x64 status display

BIT_US = 1_000_000 // LIN_BAUD
IDLE_US = 200 * BIT_US  # ~ max LIN frame time; ends the last frame of a burst


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


def format_line(t_ms, fr):
    fid, pid, data, chk, cs, status = decode(fr)
    hexdata = " ".join("%02X" % x for x in data) if data else "-"
    return "%10.1f  %s   %s  %3d  %-24s  %s   %-3s  %s" % (
        t_ms, h2(fid), h2(pid), len(data), hexdata, h2(chk), cs, status), status


HEADER = "   time_ms  id  pid  len  data                      chk  cs   status"


# ------------------------------------------------------------------- hardware
def main():
    import time
    import rp2
    import framebuf
    from machine import Pin, I2C, mem32

    # PIO UART receiver with break detection. Data bytes arrive as
    # (byte << 24); a break (all-zero byte + low stop bit) arrives as
    # 0xFFFFFFFF. Other framing errors are dropped.
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
        push()
        wait(1, pin, 0)
        jmp("start")
        label("good_stop")
        push()

    rx_pin = Pin(RX_PIN, Pin.IN, Pin.PULL_UP)
    sm = rp2.StateMachine(0, lin_rx, freq=8 * LIN_BAUD,
                          in_base=rx_pin, jmp_pin=rx_pin)
    sm.active(1)
    PIO0_FDEBUG = 0x50200000 + 0x008   # RXSTALL bit 0 = SM0 FIFO overflowed

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

    # ---- optional SSD1306, refreshed in small chunks so the PIO FIFO
    # (8 words ~ 4 ms at 19200) never overflows while the display updates.
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

    # ---- main loop
    time.sleep(0.1)
    print("LIN logger @ %d baud, RX on GP%d%s" % (
        LIN_BAUD, RX_PIN, ", SIMULATE (GP4 -> GP5)" if SIMULATE else ""))
    print(HEADER)

    t0 = time.ticks_us()
    frame = None
    t_frame = 0
    last_rx = t0
    frames = errors = overruns = stray = 0
    last_id = None
    last_status = ""
    fps_count, fps, fps_t = 0, 0, time.ticks_ms()
    next_draw = time.ticks_ms()

    def emit(t, fr):
        nonlocal frames, errors, last_id, last_status, fps_count
        line, status = format_line(time.ticks_diff(t, t0) / 1000, fr)
        print(line)
        frames += 1
        fps_count += 1
        if status not in ("OK", "NO_RESP"):
            errors += 1
        if len(fr) > 1:
            last_id = fr[1] & 0x3F
        last_status = status

    while True:
        now = time.ticks_us()
        while sm.rx_fifo():
            w = sm.get()
            now = time.ticks_us()
            last_rx = now
            if w & 0xFFFFFF:                 # break marker
                if frame is not None:
                    emit(t_frame, frame)
                frame = bytearray()
                t_frame = now
            elif frame is not None:
                frame.append(w >> 24)
                if len(frame) > 12:          # runaway: no break seen
                    emit(t_frame, frame)
                    frame = None
            else:
                stray += 1

        if frame is not None and time.ticks_diff(now, last_rx) > IDLE_US:
            emit(t_frame, frame)
            frame = None

        if mem32[PIO0_FDEBUG] & 1:
            mem32[PIO0_FDEBUG] = 1           # write-1-to-clear
            overruns += 1
            print("!! RX FIFO overrun (data lost)")

        if sim:
            sim.step()

        ms = time.ticks_ms()
        if time.ticks_diff(ms, fps_t) >= 1000:
            fps, fps_count, fps_t = fps_count, 0, ms

        if oled and not sm.rx_fifo():
            if oled.chunk < 0 and time.ticks_diff(ms, next_draw) >= 0:
                next_draw = time.ticks_add(ms, 500)
                oled.fill(0)
                oled.text("LIN %d bd" % LIN_BAUD, 0, 0)
                oled.hline(0, 10, 128, 1)
                oled.text("frames %d" % frames, 0, 14)
                oled.text("fps    %d" % fps, 0, 24)
                oled.text("errors %d" % errors, 0, 34)
                oled.text("last %s %s" % (h2(last_id), last_status[:7]), 0, 44)
                oled.text("ovr %d  stray %d" % (overruns, stray), 0, 54)
                oled.start_refresh()
            oled.step()


if __name__ == "__main__":
    main()
