"""Extract CYW43439 MFG firmware+CLM from picow-wifi-mfg-tester.uf2.

PROPRIETARY OUTPUT — do not commit the generated files.
See firmware/README.md (NDA holders only).

Example:
  python tools/extract_mfg_fw_from_uf2.py ~/Downloads/picow-wifi-mfg-tester.uf2
  python tools/extract_mfg_fw_from_uf2.py path/to/picow-wifi-mfg-tester.uf2 --out-dir firmware
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

FLASH_BASE = 0x10000000
MAGIC0 = 0x0A324655
MAGIC1 = 0x9E5D5157

# Known-good descriptors from Raspberry Pi picow-wifi-mfg-tester.uf2
EXPECTED_FW_SIZE = 220688
EXPECTED_CLM_SIZE = 4752


def load_uf2(path: Path) -> bytes:
    data = path.read_bytes()
    chunks: dict[int, bytes] = {}
    for i in range(0, len(data), 512):
        blk = data[i : i + 512]
        if len(blk) < 512:
            break
        m0, m1 = struct.unpack_from("<II", blk, 0)
        if m0 != MAGIC0 or m1 != MAGIC1:
            continue
        _flags, addr, size, _seq, _total, _fam = struct.unpack_from("<IIIIII", blk, 8)
        if addr < FLASH_BASE:
            continue
        off = addr - FLASH_BASE
        chunks[off] = blk[32 : 32 + size]
    if not chunks:
        raise SystemExit("ERROR: No RP2040 flash blocks found in UF2.")
    max_end = max(off + len(payload) for off, payload in chunks.items())
    flash = bytearray(max_end)
    for off, payload in chunks.items():
        flash[off : off + len(payload)] = payload
    return bytes(flash)


def align_up(val: int, align: int) -> int:
    return (val + align - 1) & ~(align - 1)


def find_whd_fw_descriptor(flash: bytes) -> tuple[int, int]:
    """Return (fw_start_offset, fw_size) from WHD size/pointer pair before the blob."""
    marker = b"43439a0-roml/sdio-g-mfgtest"
    ver_idx = flash.find(marker)
    if ver_idx < 0:
        marker = b"mfgtest"
        ver_idx = flash.find(marker)
    if ver_idx < 0:
        raise SystemExit("ERROR: MFG version marker not found (is this picow-wifi-mfg-tester.uf2?).")

    # Scan backwards (4-byte aligned) for size + flash pointer to the blob.
    aligned_start = (ver_idx - 8) & ~3
    for i in range(aligned_start, max(0, ver_idx - 400_000) - 1, -4):
        size, ptr = struct.unpack_from("<II", flash, i)
        if not (180_000 <= size <= 280_000):
            continue
        if not (FLASH_BASE <= ptr < FLASH_BASE + len(flash)):
            continue
        start = ptr - FLASH_BASE
        if start < 0 or start + size > len(flash):
            continue
        if start <= ver_idx < start + size and b"Version:" in flash[start : start + size]:
            return start, size
    raise SystemExit("ERROR: Could not locate WHD Wi-Fi firmware size/pointer descriptor.")


def find_whd_clm_descriptor(flash: bytes, fw_start: int, fw_size: int) -> tuple[int, int]:
    """Return (clm_start_offset, clm_size) just after the Wi-Fi image."""
    region = flash[fw_start + fw_size : fw_start + fw_size + 64]
    # Common layout: u32 size, u32 ptr immediately after FW
    if len(region) >= 8:
        size, ptr = struct.unpack_from("<II", region, 0)
        if 512 <= size <= 8192 and FLASH_BASE <= ptr < FLASH_BASE + len(flash):
            start = ptr - FLASH_BASE
            if flash[start : start + 4] == b"BLOB":
                return start, size
        size, ptr = struct.unpack_from("<II", region, 4)
        if 512 <= size <= 8192 and FLASH_BASE <= ptr < FLASH_BASE + len(flash):
            start = ptr - FLASH_BASE
            if flash[start : start + 4] == b"BLOB":
                return start, size

    blob = flash.find(b"BLOB", fw_start + fw_size)
    if blob < 0 or b"CLM DATA" not in flash[blob : blob + 256]:
        raise SystemExit("ERROR: Could not locate CLM BLOB after Wi-Fi firmware.")
    # Fallback length: through last non-zero within 8 KiB
    last = blob
    for i in range(blob, min(len(flash), blob + 8192)):
        if flash[i] != 0:
            last = i
    return blob, last - blob + 1


def write_header(path: Path, combined: bytes, fw_size: int, clm_size: int, ver: str) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("/* PROPRIETARY — generated locally; do not commit or publish */\n")
        f.write("/* Auto-extracted from picow-wifi-mfg-tester.uf2 (Raspberry Pi NDA package) */\n")
        f.write(f"/* {ver} */\n")
        f.write("#include <stdint.h>\n")
        f.write('#include "cyw43_config.h"\n\n')
        f.write("static const unsigned char mfg_cyw43_fw[] CYW43_RESOURCE_ATTRIBUTE = {\n")
        for i in range(0, len(combined), 12):
            chunk = combined[i : i + 12]
            f.write("  " + ", ".join(f"0x{b:02x}" for b in chunk) + ",\n")
        f.write("};\n\n")
        f.write(f"#define CYW43_WIFI_FW_LEN ({fw_size}) // MFG 43439A0 (unpadded)\n")
        f.write(f"#define CYW43_CLM_LEN ({clm_size}) // MFG clm_min BLOB\n")
        f.write("const uintptr_t fw_data = (uintptr_t)&mfg_cyw43_fw[0];\n")


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "uf2",
        type=Path,
        help="Path to picow-wifi-mfg-tester.uf2 (NDA; keep out of git)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=repo_root / "firmware",
        help="Output directory (default: <repo>/firmware)",
    )
    args = parser.parse_args()

    uf2_path = args.uf2.expanduser().resolve()
    if not uf2_path.is_file():
        print(f"ERROR: UF2 not found: {uf2_path}", file=sys.stderr)
        return 1

    print("NOTE: Output files are proprietary and gitignored. Do not commit them.")
    flash = load_uf2(uf2_path)
    fw_start, fw_size = find_whd_fw_descriptor(flash)
    clm_start, clm_size = find_whd_clm_descriptor(flash, fw_start, fw_size)

    fw = flash[fw_start : fw_start + fw_size]
    clm = flash[clm_start : clm_start + clm_size]

    if b"WLTEST" not in fw and b"mfgtest" not in fw:
        print("ERROR: Extracted image does not look like MFG/WLTEST firmware.", file=sys.stderr)
        return 1
    if clm[:4] != b"BLOB" or b"CLM DATA" not in clm:
        print("ERROR: Extracted CLM does not look like a Cypress BLOB.", file=sys.stderr)
        return 1

    if fw_size != EXPECTED_FW_SIZE or clm_size != EXPECTED_CLM_SIZE:
        print(
            f"WARNING: sizes fw={fw_size} clm={clm_size} "
            f"(known picow-wifi-mfg-tester.uf2 was {EXPECTED_FW_SIZE}/{EXPECTED_CLM_SIZE})"
        )

    pad_to = align_up(fw_size, 512)
    combined = fw + bytes(pad_to - fw_size) + clm
    ver = fw[fw.find(b"43439a0") : fw.find(b"\x00", fw.find(b"43439a0"))]
    if not ver:
        ver = b"CYW43439A0 MFG (version string parse failed)"
    ver_s = ver.decode("ascii", errors="replace")

    out_dir = args.out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    bin_path = out_dir / "43439A0_mfg_combined.bin"
    hdr_path = out_dir / "mfg_cyw43_fw.h"
    bin_path.write_bytes(combined)
    write_header(hdr_path, combined, fw_size, clm_size, ver_s)

    print(f"FW:  {fw_size} bytes @ flash+0x{fw_start:X}")
    print(f"CLM: {clm_size} bytes @ flash+0x{clm_start:X}")
    print(f"Combined: {len(combined)} bytes")
    print(ver_s)
    print(f"Wrote {bin_path}")
    print(f"Wrote {hdr_path}")
    print("Next: cmake -B build -DRM2_USE_MFG_WIFI_FW=ON && cmake --build build --clean-first")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
