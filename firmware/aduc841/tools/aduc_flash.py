#!/usr/bin/env python3
"""ADuC841 / 842 / 843 serial flasher (replaces WSD.exe on macOS / Linux).

Implements the Version 2 loader protocol described in Analog Devices
application note AN-1074. Speaks to the on-chip bootloader over UART:

    Host                                ADuC841 bootloader
     |  PSEN low + reset                 → enters loader
     |  ─── 0x21 0x5A 0x00 0xA6  ───────►   "interrogate"
     |  ◄── 25-byte ID packet (ADI 841 V202 ...)
     |  ─── 0x07 0x0E [N] [Cmd] [...] [CS] ─►  ACK / NAK
     |  (Erase → Write... → Run)

Packet format:
    [0x07] [0x0E] [N] [Cmd] [data...] [CS]
    N  = 1 + len(data)               (1..25, includes Cmd byte)
    CS = (0x100 - (N + Cmd + Σdata)) & 0xFF

Commands actually used here:
    'C' (0x43) — Erase Flash code memory only
    'W' (0x57) — Write block to Flash  (≤21 data bytes per packet)
    'U' (0x55) — Run from address (Remote RUN)
    'B' (0x42) — Change baud rate (T3-based parts)

Usage:
    python aduc_flash.py --port /dev/cu.usbserial-A100VKSF --hex build/firmware.ihx
    python aduc_flash.py --port ... --hex ... --no-run        # don't auto-start
    python aduc_flash.py --port ... --probe                   # interrogate only
    python aduc_flash.py --port ... --listen                  # passive: just dump RX
    python aduc_flash.py --port ... --scan-baud               # try all common baud rates
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial  # pip install pyserial

# ─── protocol constants ──────────────────────────────────────────────────────

PKT_START = bytes([0x07, 0x0E])
ACK = 0x06
NAK = 0x07

INTERROGATE = bytes([0x21, 0x5A, 0x00, 0xA6])  # "!Z\0¦"

CMD_ERASE_CODE = 0x43       # 'C'  — erase Flash/EE program memory only
CMD_ERASE_ALL  = 0x41       # 'A'  — erase code AND data Flash/EE
CMD_WRITE      = 0x57       # 'W'  — program block of code memory
CMD_PAGEDL     = 0x51       # 'Q'  — quick 256-byte page download
CMD_VERIFY     = 0x56       # 'V'  — read page back
CMD_RUN        = 0x55       # 'U'  — jump to user code
CMD_BAUD       = 0x42       # 'B'  — change baud rate

# Max payload size per Write packet:
#   Total data field = N (≤25) bytes including the cmd
#   For 'W': cmd(1) + addrU(1) + addrM(1) + addrL(1) + payload(≤21)
WRITE_PAYLOAD_MAX = 21


# ─── Intel Hex parser ────────────────────────────────────────────────────────

@dataclass
class HexRecord:
    addr: int
    data: bytes


def parse_intel_hex(text: str) -> list[HexRecord]:
    """Return a list of (addr, data) records from an Intel HEX file.

    Only handles record types 00 (data) and 01 (EOF). Extended-address
    records (02, 04) raise — ADuC841 has at most 62 KB Flash, single
    16-bit address space, so we never need them in practice.
    """
    records: list[HexRecord] = []
    for line_no, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"line {line_no}: missing ':' start mark")

        try:
            payload = bytes.fromhex(line[1:])
        except ValueError as e:
            raise ValueError(f"line {line_no}: bad hex digits: {e}") from e

        if len(payload) < 5:
            raise ValueError(f"line {line_no}: record too short ({len(payload)})")

        nbytes = payload[0]
        addr = (payload[1] << 8) | payload[2]
        rectype = payload[3]
        data = payload[4 : 4 + nbytes]
        chk = payload[4 + nbytes]

        # checksum: sum of all bytes (incl. checksum) is 0 mod 256
        if (sum(payload) & 0xFF) != 0:
            raise ValueError(f"line {line_no}: bad Intel-HEX checksum")

        if rectype == 0x01:
            break
        if rectype != 0x00:
            raise ValueError(
                f"line {line_no}: unsupported record type 0x{rectype:02X} "
                f"(only 00=data, 01=EOF supported)"
            )
        records.append(HexRecord(addr=addr, data=data))
    return records


def coalesce_records(records: list[HexRecord]) -> list[HexRecord]:
    """Merge contiguous records into bigger blocks.

    Most assemblers emit 16-byte records; merging them lets the flasher
    use 21-byte Write packets (efficiency × 1.3) and is a no-op when
    they're already non-contiguous.
    """
    if not records:
        return records
    records = sorted(records, key=lambda r: r.addr)
    out: list[HexRecord] = [HexRecord(records[0].addr, records[0].data)]
    for r in records[1:]:
        last = out[-1]
        if r.addr == last.addr + len(last.data):
            out[-1] = HexRecord(last.addr, last.data + r.data)
        else:
            out.append(HexRecord(r.addr, r.data))
    return out


# ─── packet construction ────────────────────────────────────────────────────

def make_packet(cmd: int, data: bytes = b"") -> bytes:
    """Build a Version-2 data packet: [0x07 0x0E][N][Cmd][data...][CS]."""
    n = 1 + len(data)
    if not (1 <= n <= 25):
        raise ValueError(f"packet payload N={n} out of range 1..25")
    body = bytes([n, cmd]) + data
    cs = (0x100 - (sum(body) & 0xFF)) & 0xFF
    return PKT_START + body + bytes([cs])


# ─── flasher class ──────────────────────────────────────────────────────────

class AducFlasher:

    def __init__(self, port: str, baud: int = 9600, timeout: float = 1.0,
                 verbose: bool = True):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.verbose = verbose
        self.ser: serial.Serial | None = None

    # ── connection management ─────────────────────────────────────────────

    def open(self) -> None:
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )
        # FTDI/CH340/CP210x sometimes assert RTS/DTR on open which can hold
        # the chip in reset. Make sure both are de-asserted.
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ── log helper ────────────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(msg, file=sys.stderr, flush=True)

    # ── primitive I/O ─────────────────────────────────────────────────────

    def _send(self, data: bytes) -> None:
        assert self.ser is not None
        self.ser.write(data)
        self.ser.flush()

    def _read_exact(self, n: int, timeout: float | None = None) -> bytes:
        assert self.ser is not None
        if timeout is not None:
            old = self.ser.timeout
            self.ser.timeout = timeout
            try:
                buf = self.ser.read(n)
            finally:
                self.ser.timeout = old
        else:
            buf = self.ser.read(n)
        if len(buf) != n:
            raise TimeoutError(
                f"expected {n} bytes from chip, got {len(buf)} "
                f"(rx so far: {buf.hex(' ')})"
            )
        return buf

    def _read_ack(self, op: str = "") -> None:
        b = self._read_exact(1, timeout=2.0)
        if b[0] == ACK:
            return
        if b[0] == NAK:
            raise RuntimeError(f"NAK from bootloader during {op or 'op'}")
        raise RuntimeError(
            f"unexpected response 0x{b[0]:02X} during {op or 'op'} "
            f"(expected ACK=0x06 or NAK=0x07)"
        )

    # ── high-level operations ─────────────────────────────────────────────

    def interrogate(self) -> bytes:
        """Send !Z packet, return the 25-byte ID response.

        AN-1074 says the loader auto-emits the same 25-byte packet right after
        reset (in bootloader mode), so we first listen briefly for that, and
        only fall back to !Z if nothing arrives — this lets us distinguish
        "chip just woke up in bootloader" from "chip running user code".
        """
        assert self.ser is not None
        self.ser.reset_input_buffer()

        # First — passive listen for ~250 ms in case the loader just emitted
        # its auto-ID after a fresh reset. If we catch 25 bytes, return them.
        self.ser.timeout = 0.25
        buf = self.ser.read(25)
        if len(buf) == 25:
            self._log("• caught auto-ID (chip was just reset into bootloader)")
            return self._verify_id_checksum(buf)

        # Otherwise — actively interrogate.
        self._send(INTERROGATE)
        buf = self._read_exact(25, timeout=2.0)
        return self._verify_id_checksum(buf)

    def _verify_id_checksum(self, buf: bytes) -> bytes:
        # twos-complement checksum of first 24 bytes should equal byte 25
        s = sum(buf[:24]) & 0xFF
        cs_expected = ((0x100 - s) & 0xFF)
        if buf[24] != cs_expected:
            self._log(
                f"warning: ID checksum mismatch — got 0x{buf[24]:02X}, "
                f"expected 0x{cs_expected:02X}. Probably still OK."
            )
        return buf

    # ── diagnostic helpers ────────────────────────────────────────────────

    def listen(self, seconds: float = 5.0) -> bytes:
        """Passive: read everything that arrives for `seconds` and return it.

        Useful for diagnosing "is the chip even talking?". Press RESET on
        the board while this runs — if any user firmware does printf, you'll
        see it; if the loader is active, you'll see its 25-byte auto-ID.
        """
        assert self.ser is not None
        self.ser.reset_input_buffer()
        self._log(f"• listening on {self.port} @ {self.baud} baud for {seconds:.1f} s …")
        self._log("  (press RESET on the board now)")
        self.ser.timeout = seconds
        buf = self.ser.read(4096)
        return buf

    def scan_baud_rates(self, baud_list: list[int], listen_secs: float = 3.0) -> dict:
        """For each baud, try probe; report what came back. Returns dict
        baud → 'OK <id>' / 'silent' / 'garbage <hex>'."""
        assert self.ser is not None
        results: dict[int, str] = {}
        for b in baud_list:
            self.ser.baudrate = b
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self._log(f"• trying {b} baud …")
            self.ser.timeout = listen_secs
            # First — listen for auto-ID
            buf = self.ser.read(25)
            if len(buf) == 25:
                try:
                    decoded = AducFlasher.decode_id(buf)
                    results[b] = f"OK auto-ID: {decoded}"
                    self._log(f"   ✓ caught auto-ID: {decoded}")
                    continue
                except Exception:
                    pass
            # Then — try active interrogate
            self.ser.write(INTERROGATE)
            self.ser.flush()
            self.ser.timeout = 1.5
            buf = self.ser.read(25)
            if len(buf) == 25:
                try:
                    decoded = AducFlasher.decode_id(buf)
                    results[b] = f"OK probe: {decoded}"
                    self._log(f"   ✓ probe responded: {decoded}")
                    continue
                except Exception:
                    pass
            if not buf:
                results[b] = "silent"
                self._log("   silent")
            else:
                results[b] = f"garbage {buf.hex(' ')}"
                self._log(f"   garbage: {buf.hex(' ')}")
        return results

    @staticmethod
    def decode_id(id_bytes: bytes) -> str:
        """Format the 25-byte ID into something readable."""
        product = id_bytes[0:10].rstrip(b"\x00 ").decode("ascii", errors="replace")
        fw      = id_bytes[10:14].decode("ascii", errors="replace")
        hwcfg   = id_bytes[16:18].hex()
        return f"product={product!r}  fw={fw!r}  hwcfg=0x{hwcfg}"

    def erase_code(self) -> None:
        self._log("• erasing code Flash …")
        self._send(make_packet(CMD_ERASE_CODE))
        self._read_ack("erase")
        self._log("  done")

    def write_block(self, addr: int, data: bytes) -> None:
        """Write up to 21 bytes at the given 16-bit code address."""
        if len(data) > WRITE_PAYLOAD_MAX:
            raise ValueError(f"block too long ({len(data)} > {WRITE_PAYLOAD_MAX})")
        if addr > 0xFFFF:
            raise ValueError(f"address {addr:#x} > 0xFFFF (24-bit not used here)")
        payload = bytes([
            (addr >> 16) & 0xFF,   # ADR_U (always 0 on ADuC841)
            (addr >>  8) & 0xFF,   # ADR_M
            (addr      ) & 0xFF,   # ADR_L
        ]) + data
        self._send(make_packet(CMD_WRITE, payload))
        self._read_ack(f"write @0x{addr:04X}")

    def write_records(self, records: list[HexRecord]) -> None:
        total_bytes = sum(len(r.data) for r in records)
        self._log(
            f"• writing {len(records)} record(s), {total_bytes} bytes total"
        )
        written = 0
        for r in records:
            offset = 0
            while offset < len(r.data):
                chunk = r.data[offset : offset + WRITE_PAYLOAD_MAX]
                self.write_block(r.addr + offset, chunk)
                offset += len(chunk)
                written += len(chunk)
            # tiny progress dot every ~512 bytes
            if self.verbose and (written // 512) != ((written - len(r.data)) // 512):
                print(".", end="", file=sys.stderr, flush=True)
        if self.verbose:
            print(f"\n  done ({written} bytes)", file=sys.stderr, flush=True)

    def run(self, addr: int = 0x0000) -> None:
        """Send Remote-RUN command. Bootloader returns ACK then jumps."""
        self._log(f"• jump to user code @0x{addr:04X}")
        payload = bytes([
            (addr >> 16) & 0xFF,
            (addr >>  8) & 0xFF,
            (addr      ) & 0xFF,
        ])
        self._send(make_packet(CMD_RUN, payload))
        self._read_ack("run")


# ─── CLI ─────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="ADuC841 serial flasher")
    ap.add_argument("--port", required=True,
                    help="serial port (e.g. /dev/cu.usbserial-A100VKSF)")
    ap.add_argument("--baud", type=int, default=9600,
                    help="bootloader baud rate (default 9600 for 11.0592 MHz xtal)")
    ap.add_argument("--hex",  type=Path, default=None,
                    help="Intel-HEX file to flash")
    ap.add_argument("--probe", action="store_true",
                    help="just interrogate the bootloader and print its ID")
    ap.add_argument("--listen", type=float, metavar="SECONDS", default=None,
                    help="passive: dump RX bytes for N seconds (no TX). "
                         "Useful for 'is the chip even talking?'")
    ap.add_argument("--scan-baud", action="store_true",
                    help="probe at the most common baud rates (covers 11.0592, "
                         "16, 20 MHz crystals). Use when --probe times out.")
    ap.add_argument("--no-erase", action="store_true",
                    help="skip the Erase command (unsafe — chip must be blank)")
    ap.add_argument("--no-run",   action="store_true",
                    help="don't issue Remote-RUN after writing")
    ap.add_argument("--run-addr", type=lambda s: int(s, 0), default=0x0000,
                    help="address to jump to (default 0x0000)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    fl = AducFlasher(args.port, baud=args.baud, verbose=not args.quiet)
    fl.open()
    try:
        # ── Diagnostic mode 1: passive listen ─────────────────────────────
        if args.listen is not None:
            buf = fl.listen(args.listen)
            print(f"\ngot {len(buf)} bytes:", file=sys.stderr)
            if buf:
                print(f"  hex   : {buf.hex(' ')}", file=sys.stderr)
                printable = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in buf)
                print(f"  ascii : {printable}", file=sys.stderr)
            else:
                print("  (silence — chip is not transmitting on this port)",
                      file=sys.stderr)
            return 0

        # ── Diagnostic mode 2: scan multiple baud rates ───────────────────
        if args.scan_baud:
            # Common ADuC841 crystals: 11.0592 → 9600, 16 → 13889,
            # 20 → 17361. Plus a couple of generic fallbacks.
            baud_list = [9600, 13889, 17361, 19200, 38400, 57600, 115200]
            results = fl.scan_baud_rates(baud_list)
            print("\n─── scan results ───", file=sys.stderr)
            for b, status in results.items():
                marker = "✓" if status.startswith("OK") else " "
                print(f"  {marker} {b:>6} baud  →  {status}", file=sys.stderr)
            ok_bauds = [b for b, s in results.items() if s.startswith("OK")]
            if ok_bauds:
                print(f"\nuse: --baud {ok_bauds[0]}", file=sys.stderr)
                return 0
            return 2

        # ── Normal probe / flash flow ─────────────────────────────────────
        try:
            id_bytes = fl.interrogate()
        except TimeoutError:
            print(
                "\nNo response from bootloader.\n"
                "Did you:\n"
                "  1. Pull PSEN low (or press the BOOT button)?\n"
                "  2. Reset the chip (RESET button or power cycle)?\n"
                "  3. Release PSEN?\n"
                "  4. Pick the right --port (and --baud, if xtal ≠ 11.0592 MHz)?\n"
                "\nDiagnostics:\n"
                "  • '--listen 5'      — see if the chip transmits anything at all\n"
                "  • '--scan-baud'     — try common baud rates for 11.0592/16/20 MHz xtals",
                file=sys.stderr,
            )
            return 2

        print(f"bootloader: {AducFlasher.decode_id(id_bytes)}", file=sys.stderr)

        if args.probe or args.hex is None:
            return 0

        # 2. Parse HEX
        text = args.hex.read_text()
        records = coalesce_records(parse_intel_hex(text))
        if not records:
            print("error: no data records in HEX file", file=sys.stderr)
            return 3

        # 3. Erase + Write + Run
        if not args.no_erase:
            fl.erase_code()
        fl.write_records(records)
        if not args.no_run:
            fl.run(args.run_addr)

        print("OK", file=sys.stderr)
        return 0

    finally:
        fl.close()


if __name__ == "__main__":
    sys.exit(main())
