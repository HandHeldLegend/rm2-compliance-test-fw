from pathlib import Path
import struct
import re

uf2 = Path(r"C:\Users\Mitch\Downloads\picow-wifi-mfg-tester.uf2")
data = uf2.read_bytes()
print(f"UF2 size: {len(data)} bytes, blocks: {len(data)//512}")

MAGIC0 = 0x0A324655
MAGIC1 = 0x9E5D5157
flash = bytearray()
addrs = []
family = None
for i in range(0, len(data), 512):
    blk = data[i:i+512]
    if len(blk) < 512:
        break
    m0, m1 = struct.unpack_from("<II", blk, 0)
    if m0 != MAGIC0 or m1 != MAGIC1:
        continue
    flags, addr, size, seq, total, fam = struct.unpack_from("<IIIIII", blk, 8)
    payload = blk[32:32+size]
    addrs.append((addr, size))
    if family is None:
        family = fam
    end = addr + size
    if end > len(flash):
        flash.extend(b"\x00" * (end - len(flash)))
    flash[addr:end] = payload

print(f"Family ID: 0x{family:08X}" if family is not None else "Family: unknown")
print(f"Flash image reconstructed length: {len(flash)}")
print(f"Addr range: 0x{min(a for a,_ in addrs):08X} - 0x{max(a+s for a,s in addrs):08X}")

# Extract printable strings >= 6 chars
strings = []
cur = bytearray()
for b in flash:
    if 32 <= b < 127:
        cur.append(b)
    else:
        if len(cur) >= 6:
            strings.append(cur.decode("ascii"))
        cur.clear()
if len(cur) >= 6:
    strings.append(cur.decode("ascii"))

keywords = (
    "Version", "WLTEST", "4343", "CYW", "cyw", "FWID", "wl0", "Pico", "picow",
    "mfg", "MFG", "wlan", "clm", "CLM", "Infineon", "Cypress", "Broadcom",
    "Murata", "1YN", "1DX", "wb434", "w434", "BCM", "chip", "firmware",
)
interesting = [s for s in strings if any(k.lower() in s.lower() for k in keywords)]
print(f"\n=== Interesting strings ({len(interesting)}) ===")
for s in interesting[:120]:
    print(s)

print("\n=== Version: lines ===")
for s in strings:
    if "Version:" in s or s.startswith("wl0:"):
        print(s)

# Find likely firmware blob by Version: trailer near end of a large region
print("\n=== Version: offsets ===")
for m in re.finditer(rb"Version: ", flash):
    start = max(0, m.start() - 40)
    end = min(len(flash), m.start() + 100)
    ctx = flash[start:end]
    printable = "".join(chr(b) if 32 <= b < 127 else "." for b in ctx)
    print(f"0x{m.start():08X}: {printable}")
