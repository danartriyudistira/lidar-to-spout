import sys
from pathlib import Path
from math import hypot

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from lidar.protocol import PacketParser, packet_points, packet_start_angle
from lidar.background import BackgroundModel

d = Path("data/live_rec.bin").read_bytes()
p = PacketParser()
pk = p.feed(d)

revs, cur, prev = [], [], None
for k in pk:
    if k.is_zero_packet:
        if cur:
            revs.append(cur)
        cur = []
        continue
    for ang, dmm in packet_points(k, min_dist_mm=60.0):
        cur.append((ang, dmm / 1000.0, 0.0))
if cur:
    revs.append(cur)
print("revs:", len(revs), "points/rev:", sum(len(r) for r in revs) / max(1, len(revs)))

bg = BackgroundModel(bins=720, tol_m=0.12, learn_revs=3)
for r in revs[:30]:
    bg.update(r)
print("learned ratio:", round(bg.learned_ratio, 3))

left = []
for r in revs[30:]:
    fg, b = bg.split(r)
    left.append((len(r), len(fg)))
print("rev points -> foreground:", left[:12])
print("avg fg ratio:", round(sum(f for _, f in left) / max(1, sum(n for n, _ in left)), 3))

# periksa contoh foreground terakhir
r = revs[-1]
fg, b = bg.split(r)
for ang, dist, _ in fg[:15]:
    bni = bg._bin(ang)
    print(f"ang {ang:6.1f} dist {dist:5.2f} bin {bni:3d} bg={bg.dists[bni].round(2)} conf={bg.confs[bni]}")
