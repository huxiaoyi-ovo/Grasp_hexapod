"""Per-landing start-offset tolerance of a front-climb config (raw Xiaolan STL height map, model only).

A start error e=(dx,dy) of the robot relative to Xiaolan moves every planned foothold by e relative to
the terrain. For each landing we find, per direction, the largest |e| (mm) such that the terrain under
the shifted foot stays on the same planar face (15 mm planar disc) within 10 mm of the planned height."""
import json, sys
import numpy as np
sys.path.insert(0, '/home/artrc/grasp_sync_backups/claude_front_opt_20261110')
from geom import HERE as OPT, Terrain, FOOT_RADIUS
T = Terrain(OPT / 'terrain_cache.npz')            # raw STL envelope (no recess fill)
L = ('lb', 'lf', 'lm', 'rb', 'rf', 'rm')
cfg = json.load(open(sys.argv[1]))
DIRS = {'+x': (1, 0), '-x': (-1, 0), '+y': (0, 1), '-y': (0, -1)}
STEP = np.arange(0, .0401, .001)


def ok(x, y, h0):
    """Still on the same planar face: inside the flat-safe mask (15 mm planar disc, slope < 15 deg)
    and no cumulative height change beyond 10 mm (a step that would make the foot stub or miss)."""
    return T.safe[T._ij(x, y)] & (np.abs(T.height(x, y) - h0) <= .010)


rows = []
for i, s in enumerate(cfg['stages']):
    end = np.array(s['anchor_knots'][-1])
    for leg in s['active_legs']:
        x, y, z = end[leg]
        h0 = T.height(x, y)
        tol = {}
        for k, (ux, uy) in DIRS.items():
            good = ok(x + STEP * ux, y + STEP * uy, h0)
            bad = np.nonzero(~good)[0]
            tol[k] = int(round(STEP[bad[0] - 1] * 1000)) if len(bad) else 40
        rows.append((f"C{i+1}", s['name'], L[leg], x, y, h0, tol))
for r in rows:
    t = r[6]; worst = min(t.values())
    flag = '  <-- <10mm' if worst < 10 else ('  <- <20mm' if worst < 20 else '')
    print(f"{r[0]:4s} {r[1][:26]:26s} {r[2]}  ({r[3]:.3f},{r[4]:+.3f}) h={r[5]*1000:5.1f}mm  "
          f"tol +x{t['+x']:3d} -x{t['-x']:3d} +y{t['+y']:3d} -y{t['-y']:3d} mm{flag}")
