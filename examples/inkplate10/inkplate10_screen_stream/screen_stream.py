#!/usr/bin/env python3
"""Mirror a monitor onto the Inkplate 10 over USB serial.

Grabs a monitor, stretches it to the panel, dithers it to 1 bit per pixel
(blue noise by default) and sends the rows that changed since the last
frame to inkplate10_screen_stream (XOR against the old rows, or deflated). One frame is in flight at
a time: the next one is grabbed while the board receives and paints the
current one, and sent once the board ACKs, so it runs at whatever rate the
panel or the link allows and never queues stale frames.

Every 5 s it prints where the time went (the board prints its side too):
grab and dither run in the background; encode and "trip" (write to ACK:
the link plus the board's wait for the panel) are on the frame's path, and
"stall" is the main loop waiting for a grab that was not ready yet.

    pip install numpy pillow pyserial mss
    python3 screen_stream.py --port /dev/cu.usbmodem1101
    python3 screen_stream.py --usb                  # inkplate10_screen_stream_hs
    python3 screen_stream.py --list                 # monitors and ports
    python3 screen_stream.py --selftest             # codec check, no board

macOS: the terminal needs Screen Recording permission (System Settings >
Privacy & Security), or every grab comes back as wallpaper only. The mouse
cursor is not captured.

Wire format: see the comment block at the top of the sketch.
"""

import argparse
import errno
import struct
import sys
import threading
import time
import zlib

import numpy as np
from PIL import Image, ImageFilter

MAGIC = b"EPDF"
ACK, NAK = 0x06, 0x15
READY = "ready"                     # Board.send(): the board rebooted
FLAG_KEY, FLAG_CLEAR = 1, 2
ENC_RAW, ENC_RLE, ENC_ZXOR, ENC_ZNEW = 0, 1, 2, 3
ALL_ENCS = (ENC_RAW, ENC_RLE, ENC_ZXOR, ENC_ZNEW)

# Dithering. Threshold maps (blue noise, the default, and Bayer) decide each
# pixel on its own: an edit changes only the pixels whose grey crosses their
# threshold, nothing around it. Floyd-Steinberg looks smooth but carries
# each pixel's error to its neighbours, so a change ripples outwards (see
# to_bits). All of them give the same pixels for the same picture, so a
# static screen costs nothing to send or paint.
#
# 8x8 Bayer matrix as thresholds 0..255 (--dither bayer): a regular
# cross-hatch, visible as a grid on flat greys.
_B8 = np.array([
    [0, 32, 8, 40, 2, 34, 10, 42],
    [48, 16, 56, 24, 50, 18, 58, 26],
    [12, 44, 4, 36, 14, 46, 6, 38],
    [60, 28, 52, 20, 62, 30, 54, 22],
    [3, 35, 11, 43, 1, 33, 9, 41],
    [51, 19, 59, 27, 49, 17, 57, 25],
    [15, 47, 7, 39, 13, 45, 5, 37],
    [63, 31, 55, 23, 61, 29, 53, 21],
], dtype=np.float32)
BAYER8 = ((_B8 + 0.5) * 4.0).astype(np.uint8)


def blue_noise(n=128, sigma=1.5):
    """n x n blue-noise threshold map 0..255 (--dither blue), by Ulichney's
    void-and-cluster: pixels are ranked so that every prefix of the ranking
    is an evenly spread, clump-free dot pattern. Thresholding a grey image
    against it gives an error-diffusion look with no grid, and no worms or
    ripples, because each pixel is still decided alone.

    Takes a few seconds, so the map is cached in ~/.cache.
    """
    import os
    path = os.path.join(os.path.expanduser("~"), ".cache", "epd_screen_stream",
                        f"bluenoise{n}_s{sigma}.npy")
    try:
        t = np.load(path)
        if t.shape == (n, n) and t.dtype == np.uint8:
            return t
    except (OSError, ValueError):
        pass
    print(f"building the {n}x{n} blue-noise map (once, cached in {path})")
    N = n * n
    # Toroidal Gaussian centred on (0, 0): rolled to (y, x) it is one dot's
    # contribution to the "energy" (local density) map.
    d = np.minimum(np.arange(n), n - np.arange(n)).astype(np.float64)
    G = np.exp(-(d[:, None] ** 2 + d[None, :] ** 2) / (2 * sigma * sigma))
    Gf = np.fft.rfft2(G)

    def energy(b):
        return np.fft.irfft2(np.fft.rfft2(b) * Gf, s=(n, n))

    def add(E, b, i, sign):
        y, x = divmod(i, n)
        b.flat[i] = sign > 0
        E += sign * np.roll(np.roll(G, y, 0), x, 1)

    # Initial pattern: 10% random dots, relaxed by moving the tightest
    # cluster's dot into the largest void until the move undoes itself.
    rng = np.random.default_rng(12345)
    b = np.zeros((n, n), bool)
    b.flat[rng.choice(N, N // 10, replace=False)] = True
    E = energy(b.astype(np.float64))
    while True:
        c = int(np.argmax(np.where(b, E, -np.inf)))
        add(E, b, c, -1)
        v = int(np.argmin(np.where(b, np.inf, E)))
        add(E, b, v, +1)
        if v == c:
            break
    ones = int(b.sum())
    rank = np.zeros(N, np.int64)
    # Phase 1: rank the initial dots, tightest cluster last removed first.
    b1, E1 = b.copy(), E.copy()
    for r in range(ones - 1, -1, -1):
        c = int(np.argmax(np.where(b1, E1, -np.inf)))
        add(E1, b1, c, -1)
        rank[c] = r
    # Phase 2 (and 3, which for a Gaussian filter is the same rule): fill
    # the largest void next.
    for r in range(ones, N):
        v = int(np.argmin(np.where(b, np.inf, E)))
        add(E, b, v, +1)
        rank[v] = r
    t = ((rank.reshape(n, n) + 0.5) * 256 / N).astype(np.uint8)
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np.save(path, t)
    except OSError:
        pass
    return t


# ---------------------------------------------------------------- capture --

def _mss_new(mss):
    # mss 10 renamed the factory to MSS; older releases only have mss().
    return getattr(mss, "MSS", None)() if hasattr(mss, "MSS") else mss.mss()


class Grabber:
    """One monitor (or a region of it) as a greyscale PIL image."""

    def __init__(self, monitor, region):
        self.region = region
        try:
            import mss
            self.sct = _mss_new(mss)
            mons = self.sct.monitors
            if not 1 <= monitor < len(mons):
                sys.exit(f"monitor {monitor} not found (have 1..{len(mons) - 1})")
            m = mons[monitor]
            if region:
                x, y, w, h = region
                m = {"left": m["left"] + x, "top": m["top"] + y, "width": w, "height": h}
            self.box = m
            self.backend = "mss"
        except ImportError:
            from PIL import ImageGrab
            self.ImageGrab = ImageGrab
            self.backend = "PIL.ImageGrab (install mss for monitor choice and speed)"
            if monitor != 1:
                print("note: --monitor needs mss; grabbing the main screen")

    def grab(self):
        if self.backend == "mss":
            s = self.sct.grab(self.box)
            return Image.frombytes("RGB", s.size, s.bgra, "raw", "BGRX").convert("L")
        img = self.ImageGrab.grab()
        if self.region:
            x, y, w, h = self.region
            img = img.crop((x, y, x + w, y + h))
        return img.convert("L")


def _fs(g):
    """Floyd-Steinberg (PIL's C error diffusion) of a grey array: True = black."""
    return ~np.asarray(Image.fromarray(g).convert("1", dither=Image.Dither.FLOYDSTEINBERG))


def _minmax3(g):
    """3x3 max and min of a uint8 image, edges replicated. Separable numpy:
    ~1 ms at 1200x825, where PIL's MaxFilter/MinFilter take ~90 ms."""
    p = np.pad(g, 1, mode="edge")
    hmax = np.maximum(np.maximum(p[:, :-2], p[:, 1:-1]), p[:, 2:])
    hmin = np.minimum(np.minimum(p[:, :-2], p[:, 1:-1]), p[:, 2:])
    return (np.maximum(np.maximum(hmax[:-2], hmax[1:-1]), hmax[2:]),
            np.minimum(np.minimum(hmin[:-2], hmin[1:-1]), hmin[2:]))


def to_bits(img, W, H, fit, dither, gamma, invert, rotate, state=None, threshold=128,
            strength=1.0, sharpen=0, text_edges=0):
    """Greyscale image -> (H, W/8) uint8, 1 = black, physical panel order.

    Text controls (see --dither-strength, --sharpen, --text-edges):
      sharpen     unsharp-mask percent applied after scaling (0 = off): the
                  downscale softens glyphs into grey smears.
      strength    0..1, how far a threshold map (blue, bayer) spreads around
                  mid-grey. At 1 every grey but pure black/white is dotted, so
                  dark UI backgrounds fill with speckle; at 0.5 greys below
                  ~64 and above ~192 snap solid and only mid-tones dither.
      text_edges  local contrast (3x3 max - min) above which a pixel is a
                  stroke or a border and is cut hard at `threshold` instead
                  of dithered (0 = off). Keeps glyph edges clean.

    state: a dict kept between calls of one stream, for --dither fs. Plain
    Floyd-Steinberg carries every change's error into all the rows below it,
    so moving one window re-dithers the rest of the screen and the whole
    lower panel is resent and repainted. With state, only the boxes whose
    grey pixels changed are dithered again (each on its own, starting with
    no error); every other pixel keeps its previous value.
    """
    sw, sh = img.size
    if fit == "stretch":
        img = img.resize((W, H), Image.BOX)
    else:
        s = (min if fit == "letterbox" else max)(W / sw, H / sh)
        nw, nh = max(1, round(sw * s)), max(1, round(sh * s))
        img = img.resize((nw, nh), Image.BOX)
        canvas = Image.new("L", (W, H), 255)
        canvas.paste(img, ((W - nw) // 2, (H - nh) // 2))
        img = canvas
    if sharpen > 0:
        img = img.filter(ImageFilter.UnsharpMask(radius=1.0, percent=int(sharpen), threshold=2))
    g = np.asarray(img, dtype=np.uint8)
    if gamma != 1.0:
        lut = (255.0 * (np.arange(256) / 255.0) ** gamma).round().astype(np.uint8)
        g = lut[g]
    if dither == "fs" and state is not None and state.get("g") is not None \
            and state["g"].shape == g.shape:
        black = state["black"].copy()
        diff = g != state["g"]
        changed = np.flatnonzero(diff.any(axis=1))
        if changed.size:
            # Contiguous runs of changed rows -> one box each, as wide as
            # the changed columns in that run.
            breaks = np.flatnonzero(np.diff(changed) > 1)
            for a, b in zip(np.r_[changed[0], changed[breaks + 1]],
                            np.r_[changed[breaks], changed[-1]]):
                cols = np.flatnonzero(diff[a:b + 1].any(axis=0))
                x0, x1 = cols[0], cols[-1] + 1
                black[a:b + 1, x0:x1] = _fs(np.ascontiguousarray(g[a:b + 1, x0:x1]))
        state["g"], state["black"] = g, black
    elif dither == "fs":
        black = _fs(g)
        if state is not None:
            state["g"], state["black"] = g, black
    elif dither in ("blue", "bayer"):
        m = blue_noise() if dither == "blue" else BAYER8
        t = np.tile(m, (H // m.shape[0] + 1, W // m.shape[1] + 1))[:H, :W]
        if strength != 1.0:
            t = (128 + (t.astype(np.float32) - 128) * strength).astype(np.uint8)
        black = g < t
        if text_edges > 0:
            mx, mn = _minmax3(g)
            edge = (mx.astype(np.int16) - mn) > text_edges
            black[edge] = g[edge] < threshold
    else:
        black = g < threshold          # none / threshold: hard cut, no pattern
    if invert:
        black = ~black
    if rotate == 180:
        black = black[::-1, ::-1]
    return np.packbits(black, axis=1)          # first pixel in the MSB


class FrameSource:
    """Grabs and dithers one frame in a background thread, on request.

    The Grabber is made inside the thread: mss handles are per thread.
    """

    def __init__(self, monitor, region, to_bits_args, **opts):
        self.args = to_bits_args
        self.opts = opts           # threshold, strength, sharpen, text_edges
        self.dither_state = {}     # stable Floyd-Steinberg, see to_bits()
        self.cv = threading.Condition()
        self.want = False
        self.frame = None          # (bits, grab_s, dither_s)
        self.error = None
        self.backend = None
        threading.Thread(target=self._run, args=(monitor, region), daemon=True).start()
        with self.cv:
            while self.backend is None and self.error is None:
                self.cv.wait()
        if self.error:
            raise self.error

    def _run(self, monitor, region):
        try:
            g = Grabber(monitor, region)
        except BaseException as e:   # sys.exit() included
            with self.cv:
                self.error = e
                self.cv.notify_all()
            return
        with self.cv:
            self.backend = g.backend
            self.cv.notify_all()
        while True:
            with self.cv:
                while not self.want:
                    self.cv.wait()
                self.want = False
            t0 = time.perf_counter()
            img = g.grab()
            t1 = time.perf_counter()
            bits = to_bits(img, *self.args, state=self.dither_state, **self.opts)
            t2 = time.perf_counter()
            with self.cv:
                self.frame = (bits, t1 - t0, t2 - t1)
                self.cv.notify_all()

    def request(self):
        """Start grabbing the next frame (drops one not yet taken)."""
        with self.cv:
            self.frame = None
            self.want = True
            self.cv.notify_all()

    def get(self):
        with self.cv:
            while self.frame is None:
                self.cv.wait()
            f, self.frame = self.frame, None
            return f


# ------------------------------------------------------------------ codec --

def rle_encode(x, min_zero=3):
    """XOR bytes -> tokens: c < 0x80 = c+1 literals follow, c >= 0x80 =
    (c & 0x7F)+1 zero bytes. Zero runs shorter than min_zero stay literal."""
    n = len(x)
    z = np.concatenate(([0], (x == 0).view(np.int8), [0]))
    d = np.diff(z)
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1)
    keep = (ends - starts) >= min_zero
    starts, ends = starts[keep], ends[keep]

    out = []
    raw = x.tobytes()
    pos = 0

    def literal(a, b):
        while a < b:
            k = min(128, b - a)
            out.append(bytes((k - 1,)))
            out.append(raw[a:a + k])
            a += k

    for s, e in zip(starts.tolist(), ends.tolist()):
        literal(pos, s)
        r = e - s
        out.append(bytes([0xFF]) * (r // 128))
        if r % 128:
            out.append(bytes((0x80 | (r % 128 - 1),)))
        pos = e
    literal(pos, n)
    return b"".join(out)


def rle_decode(p, n):
    """Reference decoder, mirrors applyPayload() in the sketch."""
    out = bytearray(n)
    i = done = 0
    while i < len(p):
        c = p[i]
        i += 1
        k = (c & 0x7F) + 1
        if done + k > n:
            raise ValueError("overrun")
        if c & 0x80:
            done += k
        else:
            out[done:done + k] = p[i:i + k]
            i += k
            done += k
    if done != n:
        raise ValueError(f"short: {done} of {n}")
    return bytes(out)


def deflate(b, level):
    c = zlib.compressobj(level, zlib.DEFLATED, -15)    # raw: no zlib header
    return c.compress(b) + c.flush()


def build_packet(new, old, key, clear, level=6, encs=ALL_ENCS):
    """Packet for the rows that differ, or None if nothing changed.

    Sends the smallest of: the XOR of the changed rows raw, RLE coded or
    deflated, or the new rows themselves deflated. XOR wins on small edits
    (unchanged bytes are zero); deflating the new rows wins on scrolling,
    where shifted dither XORs to noise but the rows still compress.
    """
    H = new.shape[0]
    if key:
        y0, y1 = 0, H - 1
        x = new
    else:
        diff = np.flatnonzero((new != old).any(axis=1))
        if diff.size == 0 and not clear:
            return None
        if diff.size == 0:
            diff = np.array([0])
        y0, y1 = int(diff[0]), int(diff[-1])
        x = new[y0:y1 + 1] ^ old[y0:y1 + 1]
    if clear:
        # A clear repaints everything from the board's copy; send the full
        # frame so that copy is certainly right.
        y0, y1, x, key = 0, H - 1, new, True
    flat = x.reshape(-1)
    cands = [(ENC_RAW, flat.tobytes())]
    if ENC_RLE in encs and np.count_nonzero(flat) < 0.6 * flat.size:
        cands.append((ENC_RLE, rle_encode(flat)))
    if ENC_ZXOR in encs:
        cands.append((ENC_ZXOR, deflate(cands[0][1], level)))
    if ENC_ZNEW in encs and not key:     # a key frame's XOR is the new rows
        cands.append((ENC_ZNEW, deflate(new[y0:y1 + 1].tobytes(), level)))
    if len(encs) == 1:
        cands = [c for c in cands if c[0] == encs[0]] or cands
    enc, payload = min(cands, key=lambda c: len(c[1]))
    flags = (FLAG_KEY if key else 0) | (FLAG_CLEAR if clear else 0)
    hdr = MAGIC + struct.pack("<BBHHI", flags, enc, y0, y1 - y0 + 1, len(payload))
    return hdr + payload + struct.pack("<I", zlib.crc32(payload) & 0xFFFFFFFF), y1 - y0 + 1


# -------------------------------------------------------------- the board --

class SerialLink:
    """A serial port: the board's UART bridge or its USB Serial/JTAG port."""

    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud            # UART only; ignored by USB CDC
        self.ser.timeout = 0.05
        self.ser.write_timeout = 10
        self.ser.dtr = False                # don't pulse the reset lines
        self.ser.rts = False
        try:
            self.ser.open()
        except OSError as e:
            if e.errno == errno.EBUSY:
                sys.exit(f"{port} is busy: another program has it open. The "
                         "CP2102 is also /dev/cu.SLAB_USBtoUART; only one may "
                         "be open at a time.")
            # Some ports refuse non-standard rates (a pty, some CDC drivers);
            # USB CDC ignores the rate anyway.
            print(f"port refused {baud} baud, opening at 115200")
            self.ser.baudrate = 115200
            self.ser.open()
        self.baud = self.ser.baudrate
        self.name = f"serial {port} at {self.baud} baud"

    def read(self):
        return self.ser.read(4096)

    def write(self, data):
        self.ser.write(data)


# inkplate10_screen_stream_hs's vendor interface: class, subclass, protocol.
USB_ITF = (0xFF, 0x45, 0x50)


def _usb_backend():
    """libusb for pyusb. Tries, in order: the system search path; the
    libusb-package wheel (bundles the DLL, the easy route on Windows); and
    Homebrew's lib, which is not on the default path on Apple Silicon."""
    import usb.backend.libusb1 as libusb1
    be = libusb1.get_backend()
    if be is None:
        try:
            import libusb_package
            be = libusb_package.get_libusb1_backend()
        except ImportError:
            pass
    if be is None:
        for path in ("/opt/homebrew/lib/libusb-1.0.dylib", "/usr/local/lib/libusb-1.0.dylib"):
            be = libusb1.get_backend(find_library=lambda _n, p=path: p)
            if be is not None:
                break
    if be is None:
        sys.exit("libusb not found: pip install libusb-package (any OS, needed "
                 "on Windows), brew install libusb (macOS) or "
                 "apt install libusb-1.0-0 (Linux)")
    return be


def usb_devices():
    """(device, interface) for every board running the high-speed sketch."""
    try:
        import usb.core
    except ImportError:
        sys.exit("--usb needs pyusb: pip install pyusb")
    found = []
    for dev in usb.core.find(find_all=True, backend=_usb_backend()):
        try:
            for cfg in dev:
                for itf in cfg:
                    if (itf.bInterfaceClass, itf.bInterfaceSubClass,
                            itf.bInterfaceProtocol) == USB_ITF:
                        found.append((dev, itf))
        except Exception:
            pass                            # a device we may not inspect
    return found


class UsbLink:
    """USB 2.0 bulk endpoints of inkplate10_screen_stream_hs (no driver)."""

    def __init__(self):
        import usb.core
        import usb.util
        found = usb_devices()
        if not found:
            sys.exit("no board found on USB: is inkplate10_screen_stream_hs "
                     "running and the high-speed port connected? (--list)"
                     + ("\nOn Windows the interface also needs the WinUSB "
                        "driver: bind it once with Zadig (zadig.akeo.ie)."
                        if sys.platform == "win32" else ""))
        self.dev, itf = found[0]
        try:
            self.dev.set_configuration()
        except usb.core.USBError:
            pass                            # already configured
        usb.util.claim_interface(self.dev, itf.bInterfaceNumber)
        eps = list(itf)
        d = usb.util.endpoint_direction
        self.ep_out = next(e for e in eps if d(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        self.ep_in = next(e for e in eps if d(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
        self.mps = self.ep_out.wMaxPacketSize
        speed = {1: "low", 2: "full", 3: "high", 4: "super"}.get(getattr(self.dev, "speed", 0), "?")
        self.baud = None
        self.name = (f"USB {self.dev.idVendor:04x}:{self.dev.idProduct:04x} "
                     f"bus {self.dev.bus} addr {self.dev.address}, {speed} speed, "
                     f"{self.mps}-byte bulk")
        if speed != "high":
            print(f"warning: link is {speed} speed, not high speed (cable, hub, "
                  "or a full-speed-only port)")
        self.USBTimeoutError = usb.core.USBTimeoutError

    def read(self):
        try:
            return bytes(self.ep_in.read(4096, timeout=50))
        except self.USBTimeoutError:
            return b""

    def write(self, data):
        # The board's OUT transfer ends on a short packet. A packet that
        # fills the last USB packet exactly would leave it waiting, so pad
        # one byte; the board skips bytes between frames.
        if len(data) % self.mps == 0:
            data = data + b"\0"
        self.ep_out.write(data, timeout=10000)


class Board:
    """The link to the board: writes packets, collects ACK/NAK, prints the
    board's log."""

    def __init__(self, link):
        self.link = link
        self.cv = threading.Condition()
        self.replies = []
        self.ready = False                  # board (re)booted: send a KEY
        # Deflate frames: True/False once READY says (" z" = the board's ROM
        # inflate passed its selftest); None = not heard, try and see.
        self.deflate = None
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        line = bytearray()
        while True:
            try:
                data = self.link.read()
            except Exception as e:
                print(f"link read failed: {e}")
                return
            for b in data:
                if b in (ACK, NAK):
                    with self.cv:
                        self.replies.append(b)
                        self.cv.notify_all()
                elif b == 0x0A:
                    text = line.decode(errors="replace").rstrip()
                    line.clear()
                    if text:
                        print(f"[board] {text}")
                    if text.startswith("EPDSTREAM READY"):
                        with self.cv:
                            self.deflate = "z" in text.split()[4:]
                            self.ready = True
                            self.cv.notify_all()
                elif b != 0x0D:
                    line.append(b)

    def send(self, packet, timeout):
        """Write a packet, return ACK, NAK, None (timeout) or READY.

        READY: the board (re)booted meanwhile, so the packet was lost or
        went to a board that has forgotten the old frame; resend as a KEY.
        Opening the port often resets the board, so this is the usual start.
        """
        with self.cv:
            self.replies.clear()
        self.link.write(packet)
        deadline = time.time() + timeout
        with self.cv:
            while not self.replies:
                if self.ready:
                    return READY
                left = deadline - time.time()
                if left <= 0:
                    return None
                self.cv.wait(left)
            return self.replies.pop(0)

    def take_ready(self):
        with self.cv:
            r, self.ready = self.ready, False
            return r


# ------------------------------------------------------------------- main --

def selftest(W, H):
    rng = np.random.default_rng(1)
    old = np.zeros((H, W // 8), np.uint8)
    seen = set()
    for it in range(40):
        new = old.copy()
        kind = it % 4
        if kind == 0:
            new = rng.integers(0, 256, new.shape, dtype=np.uint8)
        elif kind == 1:
            y = rng.integers(0, H - 50)
            new[y:y + 50, 10:40] ^= rng.integers(0, 256, (50, 30), dtype=np.uint8)
        elif kind == 2:
            new[rng.integers(0, H), rng.integers(0, W // 8)] ^= 0x81
        elif kind == 3:                         # scroll a structured picture
            new = np.roll(np.tile(np.arange(W // 8, dtype=np.uint8) * 3, (H, 1))
                          ^ (np.arange(H, dtype=np.uint8)[:, None] // 9), it, axis=0)
        res = build_packet(new, old, key=(it == 0), clear=(it == 7),
                           encs=ALL_ENCS if it < 8 else (ALL_ENCS[it % 4],))
        if res is None:
            assert (new == old).all()
            continue
        pkt, _ = res
        flags, enc, y0, rows, ln = struct.unpack("<BBHHI", pkt[4:14])
        payload = pkt[14:14 + ln]
        assert zlib.crc32(payload) & 0xFFFFFFFF == struct.unpack("<I", pkt[14 + ln:])[0]
        n = rows * (W // 8)
        seen.add(enc)
        if enc == ENC_RLE:
            x = rle_decode(payload, n)
        elif enc in (ENC_ZXOR, ENC_ZNEW):
            x = zlib.decompress(payload, -15)
        else:
            x = payload
        assert len(x) == n
        board = np.zeros_like(old) if flags & FLAG_KEY else old.copy()
        rows_new = np.frombuffer(x, np.uint8).reshape(rows, W // 8)
        if enc == ENC_ZNEW:
            board[y0:y0 + rows] = rows_new
        else:
            board[y0:y0 + rows] ^= rows_new
        assert (board == new).all(), f"mismatch at iteration {it}"
        old = new
    for n in (1, 2, 3, 127, 128, 129, 300, 1000):
        for v in (np.zeros(n, np.uint8), np.full(n, 7, np.uint8),
                  rng.integers(0, 3, n, dtype=np.uint8)):
            assert rle_decode(rle_encode(v), n) == v.tobytes()
    assert seen == set(ALL_ENCS), f"encodings exercised: {sorted(seen)}"
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", help="serial port of the board")
    ap.add_argument("--usb", action="store_true",
                    help="USB 2.0 high-speed link to inkplate10_screen_stream_hs "
                         "(needs pyusb and libusb) instead of a serial port")
    ap.add_argument("--baud", type=int, default=3000000,
                    help="UART link only (must match STREAM_BAUD); USB ignores it")
    ap.add_argument("--monitor", type=int, default=1, help="mss monitor number (1 = first)")
    ap.add_argument("--region", help="x,y,w,h inside the monitor")
    ap.add_argument("--width", type=int, default=1200)
    ap.add_argument("--height", type=int, default=825)
    ap.add_argument("--rotate", type=int, choices=(0, 180), default=180,
                    help="180 matches the preset's ROTATION_180")
    ap.add_argument("--fit", choices=("stretch", "letterbox", "crop"), default="stretch",
                    help="stretch fills the whole panel (aspect not kept)")
    ap.add_argument("--dither", choices=("blue", "fs", "bayer", "none", "threshold"), default="blue",
                    help="blue = blue-noise threshold map (smooth, no grid, "
                         "each pixel independent), fs = Floyd-Steinberg, "
                         "bayer = ordered 8x8 grid, none (or threshold) = no "
                         "dithering: black below --threshold, white above")
    ap.add_argument("--dither-strength", type=float, default=0.5,
                    help="0..1 for blue/bayer: 1 dots every grey (busy, text "
                         "breaks up), 0.5 snaps near-black/white solid and "
                         "dots only mid-tones, 0 = no dither")
    ap.add_argument("--sharpen", type=int, default=120,
                    help="unsharp-mask percent after scaling, crisper small "
                         "text (0 = off)")
    ap.add_argument("--text-edges", type=int, default=48,
                    help="local contrast above which a pixel is cut hard "
                         "instead of dithered, keeps glyph strokes clean "
                         "(0 = off)")
    ap.add_argument("--threshold", type=int, default=128,
                    help="grey level 1..255 below which a pixel is black, for "
                         "--dither none (raise it to keep light text, lower "
                         "it to drop light backgrounds)")
    ap.add_argument("--gamma", type=float, default=1.0, help="<1 lightens, >1 darkens")
    ap.add_argument("--invert", action="store_true")
    ap.add_argument("--max-fps", type=float, default=30.0)
    ap.add_argument("--clear-every", type=float, default=0.0,
                    help="seconds between full panel clears (0 = never)")
    ap.add_argument("--codec", choices=("auto", "raw", "small"), default="auto",
                    help="raw = XOR rows as they are (cheapest to code and to "
                         "decode); small = smallest of RAW/RLE/deflate (for slow "
                         "links); auto = raw on --usb, small on a serial port")
    ap.add_argument("--zlevel", type=int, default=6, choices=range(1, 10),
                    help="deflate level: 1 fastest, 9 smallest")
    ap.add_argument("--no-deflate", action="store_true",
                    help="send RAW/RLE frames only (as the first version did)")
    ap.add_argument("--no-overlap", action="store_true",
                    help="grab only after the ACK, as before (to compare)")
    ap.add_argument("--list", action="store_true",
                    help="list monitors, serial ports and high-speed boards")
    ap.add_argument("--selftest", action="store_true", help="check the codec, no board")
    a = ap.parse_args()

    W, H = a.width, a.height
    if W % 8:
        sys.exit("width must be a multiple of 8")
    if a.selftest:
        selftest(W, H)
        return
    if a.list:
        try:
            import mss
            for i, m in enumerate(_mss_new(mss).monitors[1:], 1):
                print(f"monitor {i}: {m['width']}x{m['height']} at {m['left']},{m['top']}")
        except ImportError:
            print("mss not installed: only the main screen can be grabbed")
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"port {p.device}: {p.description}")
        try:
            import usb.core  # noqa: F401
            for dev, itf in usb_devices():
                print(f"usb board {dev.idVendor:04x}:{dev.idProduct:04x} bus {dev.bus} "
                      f"addr {dev.address}, interface {itf.bInterfaceNumber} (--usb)")
        except ImportError:
            print("pyusb not installed: high-speed boards not listed")
        return
    if not a.port and not a.usb:
        sys.exit("--port or --usb is required (see --list)")

    region = tuple(int(v) for v in a.region.split(",")) if a.region else None
    src = FrameSource(a.monitor, region, (W, H, a.fit, a.dither, a.gamma, a.invert, a.rotate),
                      threshold=a.threshold, strength=a.dither_strength,
                      sharpen=a.sharpen, text_edges=a.text_edges)
    print(f"capture: {src.backend}")
    link = UsbLink() if a.usb else SerialLink(a.port, a.baud)
    print(f"link: {link.name}")
    board = Board(link)

    old = np.zeros((H, W // 8), np.uint8)
    need_key, first = True, True
    next_clear = time.time() + a.clear_every if a.clear_every > 0 else None
    period = 1.0 / a.max_fps if a.max_fps > 0 else 0.0
    st = dict(t=time.time(), frames=0, bytes=0, rows=0, ngrab=0, grab=0.0,
              dither=0.0, enc=0.0, trip=0.0, stall=0.0)

    # Opening the port usually resets the board: give it a moment to say
    # READY, so the first frame does not land in its boot. A board that was
    # already running stays quiet; go ahead after the wait.
    with board.cv:
        board.cv.wait_for(lambda: board.ready, timeout=4.0)

    # On a 30 MB/s USB link a raw 124 KB frame takes ~4 ms; deflating it
    # costs the PC tens of ms and the board an inflate. Squeeze only slow links.
    codec = a.codec if a.codec != "auto" else ("raw" if a.usb else "small")
    print(f"codec: {codec}")

    src.request()
    while True:
        t0 = time.time()
        if board.take_ready():
            need_key = True                   # the board's copy is white again
        s0 = time.perf_counter()
        new, tg, td = src.get()
        st["stall"] += time.perf_counter() - s0
        st["grab"] += tg
        st["dither"] += td
        st["ngrab"] += 1

        clear = next_clear is not None and t0 >= next_clear
        if codec == "raw":
            encs = (ENC_RAW,)
        elif board.deflate is not False and not a.no_deflate:
            encs = ALL_ENCS
        else:
            encs = (ENC_RAW, ENC_RLE)
        e0 = time.perf_counter()
        res = build_packet(new, old, need_key, clear, a.zlevel, encs)
        st["enc"] += time.perf_counter() - e0
        if res is None:
            # Nothing changed: look again once the frame period is up (the
            # grab itself already took a while).
            time.sleep(max(period - (time.time() - t0), 0.005))
            src.request()
            continue
        pkt, rows = res
        if not a.no_overlap:
            src.request()                     # grab the next frame meanwhile
        # The first frame may arrive while the board is still booting.
        w0 = time.perf_counter()
        r = board.send(pkt, 30.0 if first else 10.0)
        st["trip"] += time.perf_counter() - w0
        if a.no_overlap:
            src.request()
        if r == ACK:
            old, need_key, first = new, False, False
            if clear:
                next_clear = time.time() + a.clear_every
            st["frames"] += 1
            st["bytes"] += len(pkt)
            st["rows"] += rows
        elif r == READY:
            need_key = True                   # resent as a KEY next round
        else:
            print("board did not ACK (" + ("NAK" if r == NAK else "timeout") +
                  "): resending the whole frame")
            if r == NAK and pkt[5] in (ENC_ZXOR, ENC_ZNEW) and board.deflate is None:
                # An older sketch, or one whose inflate is broken.
                print("  board refused a deflate frame: RAW/RLE only from now on")
                board.deflate = False
            if r is None and first:
                print("  no reply at all: is the sketch's Serial on this port? A USB "
                      "port (usbmodem) needs CDCOnBoot=cdc, the UART bridge "
                      "(usbserial) CDCOnBoot=default, and --baud must match "
                      "STREAM_BAUD.")
            need_key = True

        now = time.time()
        if now - st["t"] >= 5.0:
            s = now - st["t"]
            n = max(st["frames"], 1)
            ng = max(st["ngrab"], 1)
            ms = lambda k, d: 1000 * st[k] / d
            # 8N1: 10 bits per byte. Not shown for USB, which has no baud.
            wire = (f" | UART wire time {1000 * st['bytes'] / n * 10 / link.baud:.0f} ms/frame"
                    if link.baud else "")
            print(f"{st['frames'] / s:5.1f} fps  {st['bytes'] / s / 1024:6.1f} KB/s  "
                  f"{st['bytes'] / n / 1024:6.1f} KB/frame  {st['rows'] / n:4.0f} rows | "
                  f"grab {ms('grab', ng):.0f}  dither {ms('dither', ng):.0f}  "
                  f"encode {ms('enc', ng):.0f}  trip {ms('trip', n):.0f}  "
                  f"stall {ms('stall', ng):.0f} ms{wire}")
            st.update(t=now, frames=0, bytes=0, rows=0, ngrab=0, grab=0.0,
                      dither=0.0, enc=0.0, trip=0.0, stall=0.0)
        wait = period - (time.time() - t0)
        if wait > 0:
            time.sleep(wait)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
