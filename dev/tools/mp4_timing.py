"""Print an MP4's own timing metadata (mvhd/mdhd/stts/elst), independent of any
decoder backend, to see what frame rate a player could legitimately report.

    python dev/tools/mp4_timing.py recordings/large_world_flyover.mp4
"""
import struct
import sys


def boxes(f, start, end):
    pos = start
    while pos + 8 <= end:
        f.seek(pos)
        size, typ = struct.unpack(">I4s", f.read(8))
        hdr = 8
        if size == 1:
            size = struct.unpack(">Q", f.read(8))[0]
            hdr = 16
        elif size == 0:
            size = end - pos
        yield typ.decode("latin1"), pos + hdr, pos + size
        pos += size


def walk(f, start, end, depth=0, out=None):
    for typ, s, e in boxes(f, start, end):
        if typ in ("moov", "trak", "mdia", "minf", "stbl", "edts"):
            walk(f, s, e, depth + 1, out)
        elif typ in ("mvhd", "mdhd"):
            f.seek(s)
            ver = f.read(1)[0]
            f.read(3)
            if ver == 1:
                f.read(16)
                ts, dur = struct.unpack(">IQ", f.read(12))
            else:
                f.read(8)
                ts, dur = struct.unpack(">II", f.read(8))
            print(f"{'  '*depth}{typ}: timescale {ts} duration {dur} -> {dur/ts:.3f} s")
            out["ts"] = ts
        elif typ == "hdlr":
            f.seek(s + 8)
            print(f"{'  '*depth}hdlr: {f.read(4).decode('latin1')}")
        elif typ == "elst":
            f.seek(s)
            ver = f.read(1)[0]; f.read(3)
            n = struct.unpack(">I", f.read(4))[0]
            for _ in range(n):
                if ver == 1:
                    d, t = struct.unpack(">Qq", f.read(16))
                else:
                    d, t = struct.unpack(">Ii", f.read(8))
                f.read(4)
                print(f"{'  '*depth}elst: segment_duration {d} media_time {t}")
        elif typ == "stts":
            f.seek(s + 4)
            n = struct.unpack(">I", f.read(4))[0]
            ents = [struct.unpack(">II", f.read(8)) for _ in range(min(n, 20))]
            tot = sum(c for c, _ in ents)
            print(f"{'  '*depth}stts: {n} entries; first {ents[:6]}  (samples in shown {tot})")
            if out.get("ts") and ents:
                print(f"{'  '*depth}   -> nominal {out['ts']/ents[0][1]:.3f} fps")
        elif typ == "stsd":
            f.seek(s + 12)
            f.read(4)
            print(f"{'  '*depth}stsd codec: {f.read(4).decode('latin1')}")


path = sys.argv[1]
with open(path, "rb") as f:
    f.seek(0, 2)
    walk(f, 0, f.tell(), 0, {})
