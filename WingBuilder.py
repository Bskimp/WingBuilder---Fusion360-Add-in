# WingBuilder.py
# Fusion 360 Add-in: spanwise wing builder from DAT airfoils with parametric planform.
#
# Adds (requested):
# A) Sweep distribution / kinked sweep
# B) Airfoil distribution: N airfoils with station fractions
# C) Thickness + camber scaling
# E) Calculated outputs (read-only text boxes)
#
# Notes:
#   - Fusion internal length units in API calls are centimeters.
#   - ValueCommandInput.value returns radians for angle units (deg).

import adsk.core
import adsk.fusion
import traceback
import math
import os
import datetime

CMD_ID = 'wingBuilderCmd'
CMD_NAME = 'WingBuilder (Multi-Section DAT)'
CMD_DESC = 'Create a wing from multiple airfoil .dat files with spanwise stations'

WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel'

_handlers = []
MAX_AIRFOILS = 10  # UI creates up to this many stations

# -----------------------------
# Airfoil parsing + resampling
# -----------------------------

def _try_parse_xy(line: str):
    s = line.strip().replace(',', ' ')
    if not s:
        return None
    parts = [p for p in s.split() if p]
    if len(parts) < 2:
        return None
    try:
        x = float(parts[0])
        z = float(parts[1])
        return (x, z)
    except:
        return None

def load_dat_airfoil(path: str):
    """Returns: (name, pts) where pts is list[(x,z)]"""
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    name = os.path.splitext(os.path.basename(path))[0]
    pts = []

    for i, ln in enumerate(lines):
        xy = _try_parse_xy(ln)
        if xy is None:
            if i == 0 and ln.strip():
                name = ln.strip()
            continue
        pts.append(xy)

    if len(pts) < 10:
        raise ValueError(f'Not enough coordinate points in file: {path}')

    # Remove consecutive duplicates
    clean = []
    for p in pts:
        if not clean or (abs(p[0] - clean[-1][0]) > 1e-12 or abs(p[1] - clean[-1][1]) > 1e-12):
            clean.append(p)
    return name, clean

def normalize_airfoil(pts):
    """Normalize so LE is at x=0 and TE near x=1; scales z by same chord scale."""
    xs = [p[0] for p in pts]
    x_min = min(xs)
    x_max = max(xs)
    rng = x_max - x_min
    if rng <= 1e-12:
        raise ValueError('Airfoil has zero chord length in X.')
    s = 1.0 / rng
    return [((p[0] - x_min) * s, p[1] * s) for p in pts]

def split_upper_lower(pts_norm):
    """
    Split at leading edge (min x). Returns (upper_inc, lower_inc) with x increasing 0->1.
    Works with Selig-style loop as well.
    """
    i_le = min(range(len(pts_norm)), key=lambda i: pts_norm[i][0])

    upper = pts_norm[:i_le + 1]   # usually TE->LE
    lower = pts_norm[i_le:]       # usually LE->TE

    if len(upper) < 3 or len(lower) < 3:
        raise ValueError('Could not split upper/lower surfaces cleanly.')

    if upper[0][0] > upper[-1][0]:
        upper = list(reversed(upper))
    if lower[0][0] > lower[-1][0]:
        lower = list(reversed(lower))

    def dedup_x(surface):
        out = []
        for x, z in surface:
            if not out or abs(x - out[-1][0]) > 1e-9:
                out.append((x, z))
            else:
                out[-1] = (x, z)
        return out

    return dedup_x(upper), dedup_x(lower)

def lerp1d(xs, zs, xq):
    """Linear interpolation. xs must be increasing."""
    if xq <= xs[0]:
        return zs[0]
    if xq >= xs[-1]:
        return zs[-1]

    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= xq:
            lo = mid
        else:
            hi = mid

    x0, x1 = xs[lo], xs[hi]
    z0, z1 = zs[lo], zs[hi]
    t = (xq - x0) / (x1 - x0) if abs(x1 - x0) > 1e-15 else 0.0
    return z0 + t * (z1 - z0)

def resample_selig(pts_norm, n=140):
    """
    Consistent Selig loop with cosine x-spacing:
      TE (upper) -> LE -> TE (lower)
    Returns list[(x,z)] length = 2*n - 1.
    """
    upper, lower = split_upper_lower(pts_norm)

    ux = [p[0] for p in upper]
    uz = [p[1] for p in upper]
    lx = [p[0] for p in lower]
    lz = [p[1] for p in lower]

    xs = []
    for i in range(n):
        t = i / (n - 1)
        xs.append(0.5 * (1.0 - math.cos(math.pi * t)))

    zu = [lerp1d(ux, uz, x) for x in xs]
    zl = [lerp1d(lx, lz, x) for x in xs]

    out = []
    for x, z in zip(reversed(xs), reversed(zu)):
        out.append((x, z))
    for x, z in zip(xs[1:], zl[1:]):  # skip LE duplicate
        out.append((x, z))
    return out

def ensure_airfoil_up(loop_pts):
    """Heuristic: at x=0.25, upper z should be above lower z. If not, invert Z."""
    try:
        upper, lower = split_upper_lower(loop_pts)
        ux = [x for x, z in upper]
        uz = [z for x, z in upper]
        lx = [x for x, z in lower]
        lz = [z for x, z in lower]
        xq = 0.25
        zu = lerp1d(ux, uz, xq)
        zl = lerp1d(lx, lz, xq)
        if zu < zl:
            return [(x, -z) for x, z in loop_pts]
    except:
        pass
    return loop_pts

def scale_thickness_and_camber(loop_pts, thickness_scale: float, camber_scale: float):
    """
    Apply thickness & camber scaling about mean camber line.
    """
    thickness_scale = float(thickness_scale)
    camber_scale = float(camber_scale)

    if abs(thickness_scale - 1.0) < 1e-12 and abs(camber_scale - 1.0) < 1e-12:
        return loop_pts

    upper, lower = split_upper_lower(loop_pts)

    xs_u = [x for x, z in upper]
    zu   = [z for x, z in upper]
    xs_l = [x for x, z in lower]
    zl   = [z for x, z in lower]

    xs = xs_u if len(xs_u) >= len(xs_l) else xs_l

    zu_i = [lerp1d(xs_u, zu, x) for x in xs]
    zl_i = [lerp1d(xs_l, zl, x) for x in xs]

    zc = [(a+b)*0.5 for a, b in zip(zu_i, zl_i)]
    th = [(a-b) for a, b in zip(zu_i, zl_i)]

    zc2 = [camber_scale * v for v in zc]
    th2 = [thickness_scale * v for v in th]

    zu2 = [c + 0.5*t for c, t in zip(zc2, th2)]
    zl2 = [c - 0.5*t for c, t in zip(zc2, th2)]

    out = []
    for x, z in zip(reversed(xs), reversed(zu2)):
        out.append((x, z))
    for x, z in zip(xs[1:], zl2[1:]):
        out.append((x, z))
    return out

def rotate_xz_about(x, z, xp, zp, theta_rad):
    """Rotate (x,z) about pivot (xp,zp) by theta around +Y."""
    dx = x - xp
    dz = z - zp
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    x2 = xp + dx * c + dz * s
    z2 = zp - dx * s + dz * c
    return x2, z2

def rotate_about_axis(pt, axis_pt, axis_dir, theta_rad):
    """Rotate 3D point about an arbitrary axis (Rodrigues)."""
    x, y, z = pt
    ax, ay, az = axis_pt
    kx, ky, kz = axis_dir
    # Normalize axis
    kn = math.sqrt(kx*kx + ky*ky + kz*kz)
    if kn < 1e-12:
        return pt
    kx /= kn; ky /= kn; kz /= kn

    # Translate so axis passes through origin
    vx = x - ax; vy = y - ay; vz = z - az

    c = math.cos(theta_rad)
    s = math.sin(theta_rad)

    # k x v
    cx = ky*vz - kz*vy
    cy = kz*vx - kx*vz
    cz = kx*vy - ky*vx

    # k · v
    dot = kx*vx + ky*vy + kz*vz

    rx = vx*c + cx*s + kx*dot*(1.0 - c)
    ry = vy*c + cy*s + ky*dot*(1.0 - c)
    rz = vz*c + cz*s + kz*dot*(1.0 - c)

    return (ax + rx, ay + ry, az + rz)


# -----------------------------
# Dropdown options
# -----------------------------

TWIST_DISTS = [
    'Piecewise linear (root-mid-tip)',
    'Linear (root-tip)',
    'Cosine ease (root-tip)',
    'Quadratic outboard (root-tip)',
]

CHORD_DISTS = [
    'Piecewise linear (root-mid-tip)',
    'Linear (root-tip)',
    'Elliptical (root-tip)',
    'Custom polynomial (normalized endpoints)',
]

DIHEDRAL_DISTS = [
    'Constant (root-tip)',
    'Kinked (inner/outer)',
]

SWEEP_DISTS = [
    'Constant (root-tip)',
    'Kinked (inner/outer)',
]

TWIST_AXIS = [
    'Leading edge',
    'Quarter-chord',
    'Trailing edge',
    'Elastic axis (% chord)',
]

STATION_SPACING = [
    'Linear',
    'Cosine (root & tip)',
    'Root-clustered',
    'Tip-clustered',
    'Auto (kink-aware cosine)',
]

SCALE_DISTS = [
    'Constant',
    'Linear (root-tip)',
    'Piecewise linear (root-mid-tip)',
    'Cosine ease (root-tip)',
    'Quadratic outboard (root-tip)',
]

TIP_TREATMENTS = [
    'Flat (default)',
    'Tapered cap (extended tip)',
    'Pointed cap (near-zero chord)',
]

TIP_CAP_EASE = [
    'Linear',
    'Cosine',
]

# -----------------------------
# Parameters container
# -----------------------------

class WingParams:
    def __init__(self):
        self.foilStations = []  # list of dict {t, loop}

        self.spanIsTotal = False
        self.span_cm = 30.0
        self.mirror = True

        self.midFrac = 0.5
        self.numStations = 7
        self.stationSpacing = STATION_SPACING[0]

        self.rootChord_cm = 12.0
        self.midChord_cm = 10.0
        self.tipChord_cm = 6.0
        self.chordDist = CHORD_DISTS[0]
        self.polyA0 = 0.0
        self.polyA1 = 1.0
        self.polyA2 = 0.0
        self.polyA3 = 0.0

        self.sweepDist = SWEEP_DISTS[0]
        self.sweep_rad = math.radians(15.0)
        self.sweepInner_rad = math.radians(15.0)
        self.sweepOuter_rad = math.radians(25.0)
        self.sweepBreakFrac = 0.35
        self.sweepRefFrac = 0.25

        self.dihedralDist = DIHEDRAL_DISTS[0]
        self.dihedral_rad = math.radians(3.0)
        self.dihedralInner_rad = math.radians(3.0)
        self.dihedralOuter_rad = math.radians(6.0)
        self.dihedralBreakFrac = 0.35

        self.twistRoot_rad = math.radians(0.0)
        self.twistMid_rad  = math.radians(-2.0)
        self.twistTip_rad  = math.radians(-4.0)
        self.twistDist = TWIST_DISTS[0]

        self.twistAxisMode = TWIST_AXIS[1]
        self.twistAxisFrac = 0.25

        self.resampleN = 140
        self.flipZ = True
        self.thicknessScale = 1.0
        self.camberScale = 1.0


        self.thicknessMidScale = 1.0
        self.thicknessTipScale = 1.0
        self.thicknessDist = SCALE_DISTS[0]

        self.camberMidScale = 1.0
        self.camberTipScale = 1.0
        self.camberDist = SCALE_DISTS[0]

        # Tip treatment (optional)
        self.tipTreatment = TIP_TREATMENTS[0]
        self.tipCapLength_cm = 0.0
        self.tipEndChordRatio = 0.2
        self.tipCapStations = 3
        self.tipCapEase = TIP_CAP_EASE[1]
        # Winglet (optional, added after tip)
        self.wingletEnable = False
        self.wingletHeight_cm = 5.0
        self.wingletCant_rad = math.radians(90.0)      # 0=flat, 90=vertical
        self.wingletSweep_rad = math.radians(0.0)
        self.wingletTipChordRatio = 0.7               # relative to tip chord
        self.wingletTwistTipDelta_rad = 0.0           # added to wing tip twist
        self.wingletUseTipAirfoil = True              # else use wingletPath
        self.wingletPath = ''
        self.wingletUseRails = True

        self.wingletThicknessMult = 1.0
        self.wingletCamberMult = 1.0
# -----------------------------
# Planform helpers
# -----------------------------

def _clamp01(x):
    return max(0.0, min(1.0, x))

def _piecewise_blend(a0, a1, a2, t, midFrac):
    mf = max(0.01, min(0.99, midFrac))
    if t <= mf:
        u = t / mf
        return a0*(1-u) + a1*u
    u = (t - mf) / (1.0 - mf)
    return a1*(1-u) + a2*u


def _station_ts(p: WingParams, nStations: int):
    """Generate spanwise station parameters t in [0..1] with various spacing strategies."""
    nStations = max(3, int(nStations))
    mode = getattr(p, 'stationSpacing', STATION_SPACING[0])

    # Base parameter in [0..1]
    def lin(s): 
        return s

    def cos_ease(s):
        return 0.5 * (1.0 - math.cos(math.pi * s))

    # Choose base mapping
    if mode == STATION_SPACING[0]:
        f = lin
        return [f(i/(nStations-1)) for i in range(nStations)]

    if mode == STATION_SPACING[1]:
        f = cos_ease
        return [f(i/(nStations-1)) for i in range(nStations)]

    if mode == STATION_SPACING[2]:
        # Cluster near root
        pwr = 2.0
        return [(i/(nStations-1))**pwr for i in range(nStations)]

    if mode == STATION_SPACING[3]:
        # Cluster near tip
        pwr = 2.0
        return [1.0 - (1.0 - (i/(nStations-1)))**pwr for i in range(nStations)]

    # Auto: kink-aware cosine-in-each-panel
    # Include only meaningful breakpoints (where slope changes) so density increases near kinks.
    breaks = []
    try:
        if getattr(p, 'chordDist', '') == CHORD_DISTS[0] or getattr(p, 'twistDist', '') == TWIST_DISTS[0]:
            breaks.append(float(getattr(p, 'midFrac', 0.5)))
    except:
        pass
    try:
        if getattr(p, 'sweepDist', '') == SWEEP_DISTS[1]:
            breaks.append(float(getattr(p, 'sweepBreakFrac', 0.35)))
    except:
        pass
    try:
        if getattr(p, 'dihedralDist', '') == DIHEDRAL_DISTS[1]:
            breaks.append(float(getattr(p, 'dihedralBreakFrac', 0.35)))
    except:
        pass

    # sanitize
    b = []
    for x in breaks:
        x = max(0.0, min(1.0, x))
        if x > 1e-6 and x < 1.0 - 1e-6:
            b.append(x)

    # unique + sort
    B = [0.0] + sorted(set([round(x, 6) for x in b])) + [1.0]
    if len(B) <= 2:
        # No kinks: default to cosine (ends clustered)
        return [cos_ease(i/(nStations-1)) for i in range(nStations)]

    segs = [(B[i], B[i+1]) for i in range(len(B)-1)]
    w = [max(1e-9, b-a) for (a,b) in segs]
    W = sum(w)
    target = nStations - 1

    mk = [max(1, int(round((wi/W) * target))) for wi in w]

    # Adjust to hit exact total
    def _sum(): 
        return sum(mk)

    # Add points to largest segments first
    while _sum() < target:
        j = max(range(len(mk)), key=lambda k: w[k])
        mk[j] += 1

    # Remove from smallest segments first (but keep >=1)
    while _sum() > target:
        # choose a segment with mk>1 and smallest weight
        candidates = [k for k in range(len(mk)) if mk[k] > 1]
        if not candidates:
            break
        j = min(candidates, key=lambda k: w[k])
        mk[j] -= 1

    ts = [0.0]
    for (a,b), m in zip(segs, mk):
        for i in range(1, m+1):
            s = i / m
            c = cos_ease(s)  # clusters near both ends of the panel (including kinks)
            ts.append(a + (b-a) * c)

    # Safety: ensure exact endpoints and monotonic
    ts[0] = 0.0
    ts[-1] = 1.0
    out = []
    last = -1e9
    for t in ts:
        t = max(0.0, min(1.0, t))
        if t <= last + 1e-9:
            t = min(1.0, last + 1e-6)
        out.append(t)
        last = t
    out[-1] = 1.0
    return out


def _scale_dist_at(dist_name: str, root: float, mid: float, tip: float, t: float, midFrac: float):
    """Evaluate a spanwise scaling curve."""
    root = float(root); mid = float(mid); tip = float(tip)
    t = max(0.0, min(1.0, float(t)))
    mf = max(0.01, min(0.99, float(midFrac)))

    if dist_name == SCALE_DISTS[0]:
        return root
    if dist_name == SCALE_DISTS[2]:
        return _piecewise_blend(root, mid, tip, t, mf)
    if dist_name == SCALE_DISTS[3]:
        s = 0.5 * (1.0 - math.cos(math.pi * t))
        return root + (tip - root) * s
    if dist_name == SCALE_DISTS[4]:
        s = t * t
        return root + (tip - root) * s

    # Linear (default)
    return root + (tip - root) * t
def _twist_at(p: WingParams, t):
    if p.twistDist == TWIST_DISTS[0]:
        return _piecewise_blend(p.twistRoot_rad, p.twistMid_rad, p.twistTip_rad, t, p.midFrac)

    a0 = p.twistRoot_rad
    a2 = p.twistTip_rad

    if p.twistDist == TWIST_DISTS[1]:
        s = t
    elif p.twistDist == TWIST_DISTS[2]:
        s = 0.5*(1.0 - math.cos(math.pi*t))
    elif p.twistDist == TWIST_DISTS[3]:
        s = t*t
    else:
        s = t
    return a0 + (a2 - a0) * s

def _chord_at(p: WingParams, t):
    if p.chordDist == CHORD_DISTS[0]:
        return _piecewise_blend(p.rootChord_cm, p.midChord_cm, p.tipChord_cm, t, p.midFrac)

    if p.chordDist == CHORD_DISTS[1]:
        return p.rootChord_cm + (p.tipChord_cm - p.rootChord_cm) * t

    if p.chordDist == CHORD_DISTS[2]:
        return (p.rootChord_cm - p.tipChord_cm) * math.sqrt(max(0.0, 1.0 - t*t)) + p.tipChord_cm

    a0, a1, a2, a3 = p.polyA0, p.polyA1, p.polyA2, p.polyA3
    def f(tt):
        return a0 + a1*tt + a2*(tt**2) + a3*(tt**3)

    f0 = f(0.0)
    f1 = f(1.0)
    ft = f(t)
    if abs(f1 - f0) < 1e-9:
        s = t
    else:
        s = (ft - f0) / (f1 - f0)
    s = _clamp01(s)
    return p.rootChord_cm + (p.tipChord_cm - p.rootChord_cm) * s

def _twist_axis_frac(p: WingParams):
    if p.twistAxisMode == TWIST_AXIS[0]:
        return 0.0
    if p.twistAxisMode == TWIST_AXIS[2]:
        return 1.0
    if p.twistAxisMode == TWIST_AXIS[3]:
        return _clamp01(p.twistAxisFrac)
    return 0.25

def _z_offset_at(p: WingParams, semiSpan_cm, y_cm):
    if p.dihedralDist == DIHEDRAL_DISTS[0]:
        return y_cm * math.tan(p.dihedral_rad)

    y_break = semiSpan_cm * max(0.01, min(0.99, p.dihedralBreakFrac))
    if y_cm <= y_break:
        return y_cm * math.tan(p.dihedralInner_rad)
    return (y_break * math.tan(p.dihedralInner_rad)) + ((y_cm - y_break) * math.tan(p.dihedralOuter_rad))

def _x_sweep_offset_at(p: WingParams, semiSpan_cm, y_cm):
    if p.sweepDist == SWEEP_DISTS[0]:
        return y_cm * math.tan(p.sweep_rad)

    y_break = semiSpan_cm * max(0.01, min(0.99, p.sweepBreakFrac))
    if y_cm <= y_break:
        return y_cm * math.tan(p.sweepInner_rad)
    return (y_break * math.tan(p.sweepInner_rad)) + ((y_cm - y_break) * math.tan(p.sweepOuter_rad))

# -----------------------------
# Airfoil interpolation
# -----------------------------

def _interp_airfoil_loop(stations, t):
    if not stations:
        raise ValueError('No airfoil stations provided.')

    if t <= stations[0][0]:
        return stations[0][1]
    if t >= stations[-1][0]:
        return stations[-1][1]

    lo = 0
    hi = len(stations) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if stations[mid][0] <= t:
            lo = mid
        else:
            hi = mid

    t0, a = stations[lo]
    t1, b = stations[hi]
    if abs(t1 - t0) < 1e-9:
        return a

    u = (t - t0) / (t1 - t0)
    out = []
    for (xa, za), (xb, zb) in zip(a, b):
        out.append((xa*(1-u) + xb*u, za*(1-u) + zb*u))
    return out

# -----------------------------
# Build wing geometry
# -----------------------------

def build_wing(design: adsk.fusion.Design, ui: adsk.core.UserInterface, p: WingParams):
    rootComp = design.rootComponent

    occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = 'Wing_MultiSection'

    planes = comp.constructionPlanes
    sketches = comp.sketches

    semiSpan = p.span_cm
    if semiSpan <= 1e-6:
        raise ValueError('Span must be > 0')

    stations = [(st['t'], st['loop']) for st in p.foilStations]
    stations.sort(key=lambda x: x[0])
    if len(stations) < 2:
        raise ValueError('Provide at least 2 airfoil stations.')
    if stations[0][0] > 1e-6:
        stations.insert(0, (0.0, stations[0][1]))
    if stations[-1][0] < 1.0 - 1e-6:
        stations.append((1.0, stations[-1][1]))

    nStations = max(3, int(p.numStations))
    ts = _station_ts(p, nStations)

    xRefRoot = p.sweepRefFrac * p.rootChord_cm
    twistPivotFrac = _twist_axis_frac(p)

    def station_curve_pts(t):
        y_cm = semiSpan * t
        chord = _chord_at(p, t)
        twist = _twist_at(p, t)

        x_refline = xRefRoot + _x_sweep_offset_at(p, semiSpan, y_cm)
        xOffset = x_refline - p.sweepRefFrac * chord
        zOffset = _z_offset_at(p, semiSpan, y_cm)

        xp = xOffset + twistPivotFrac * chord
        zp = zOffset

        loop = _interp_airfoil_loop(stations, t)
        thk = _scale_dist_at(getattr(p, 'thicknessDist', SCALE_DISTS[0]),
                             getattr(p, 'thicknessScale', 1.0),
                             getattr(p, 'thicknessMidScale', getattr(p, 'thicknessScale', 1.0)),
                             getattr(p, 'thicknessTipScale', getattr(p, 'thicknessScale', 1.0)),
                             t, getattr(p, 'midFrac', 0.5))
        cmb = _scale_dist_at(getattr(p, 'camberDist', SCALE_DISTS[0]),
                             getattr(p, 'camberScale', 1.0),
                             getattr(p, 'camberMidScale', getattr(p, 'camberScale', 1.0)),
                             getattr(p, 'camberTipScale', getattr(p, 'camberScale', 1.0)),
                             t, getattr(p, 'midFrac', 0.5))
        loop = scale_thickness_and_camber(loop, thk, cmb)
        pts = []
        for xn, zn in loop:
            x = xOffset + xn * chord
            z = zOffset + zn * chord
            x, z = rotate_xz_about(x, z, xp, zp, twist)
            pts.append((x, z))
        return y_cm, pts

    def make_station_plane(y_cm):
        inp = planes.createInput()
        inp.setByOffset(comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(y_cm))
        return planes.add(inp)

    def le_te_from_curve(curve_pts2d):
        i_le = min(range(len(curve_pts2d)), key=lambda k: curve_pts2d[k][0])
        x_le, z_le = curve_pts2d[i_le]
        x0, z0 = curve_pts2d[0]
        x1, z1 = curve_pts2d[-1]
        return (x_le, z_le), (0.5*(x0+x1), 0.5*(z0+z1))

    def add_airfoil_sketch(y_cm, curve_pts2d):
        pln = make_station_plane(y_cm)
        sk = sketches.add(pln)

        pts = adsk.core.ObjectCollection.create()
        for x, z in curve_pts2d:
            z_draw = -z if p.flipZ else z
            pts.add(adsk.core.Point3D.create(x, z_draw, 0.0))
        spl = sk.sketchCurves.sketchFittedSplines.add(pts)

        te_close = None
        sp = spl.startSketchPoint.geometry
        ep = spl.endSketchPoint.geometry
        if sp.distanceTo(ep) > 1e-6:
            te_close = sk.sketchCurves.sketchLines.addByTwoPoints(ep, sp)

        if sk.profiles.count < 1:
            raise ValueError('No sketch profile created; airfoil may not be closed.')

        (x_le, z_le), (x_te, z_te) = le_te_from_curve(curve_pts2d)
        z_le_draw = -z_le if p.flipZ else z_le
        z_te_draw = -z_te if p.flipZ else z_te

        le_sp = sk.sketchPoints.add(adsk.core.Point3D.create(x_le, z_le_draw, 0.0))
        te_sp = sk.sketchPoints.add(adsk.core.Point3D.create(x_te, z_te_draw, 0.0))

        try:
            sk.geometricConstraints.addCoincident(le_sp, spl)
        except:
            pass
        try:
            if te_close:
                sk.geometricConstraints.addCoincident(te_sp, te_close)
            else:
                sk.geometricConstraints.addCoincident(te_sp, spl)
        except:
            pass

        chord_ln = sk.sketchCurves.sketchLines.addByTwoPoints(le_sp, te_sp)
        chord_ln.isConstruction = True

        best = sk.profiles.item(0)
        best_area = -1.0
        for i in range(sk.profiles.count):
            pr = sk.profiles.item(i)
            try:
                ap = pr.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                a = abs(ap.area)
            except:
                a = 0.0
            if a > best_area:
                best_area = a
                best = pr

        return best, le_sp.worldGeometry, te_sp.worldGeometry

    profiles = []
    le_pts_world = []
    te_pts_world = []

    for t in ts:
        y_cm, crv = station_curve_pts(t)
        prof, le_w, te_w = add_airfoil_sketch(y_cm, crv)
        profiles.append(prof)
        le_pts_world.append(le_w)
        te_pts_world.append(te_w)


    # Optional tip treatment: extend beyond the last station with additional scaled profiles.
    try:
        tip_mode = getattr(p, 'tipTreatment', TIP_TREATMENTS[0])
    except:
        tip_mode = TIP_TREATMENTS[0]

    if tip_mode != TIP_TREATMENTS[0]:
        cap_len = float(getattr(p, 'tipCapLength_cm', 0.0))
        cap_len = max(0.0, cap_len)
        cap_n = int(getattr(p, 'tipCapStations', 3))
        cap_n = max(1, min(20, cap_n))  # number of extra profiles beyond the tip

        # End chord ratio (relative to tip chord)
        end_ratio = float(getattr(p, 'tipEndChordRatio', 0.2))
        if tip_mode == TIP_TREATMENTS[2]:
            end_ratio = min(end_ratio, 0.05)
        end_ratio = max(0.01, min(2.0, end_ratio))

        ease = getattr(p, 'tipCapEase', TIP_CAP_EASE[1])
        def cap_ease(u):
            u = max(0.0, min(1.0, u))
            if ease == TIP_CAP_EASE[1]:
                return 0.5 * (1.0 - math.cos(math.pi * u))
            return u

        if cap_len > 1e-6:
            # Use the tip airfoil loop (t=1) and tip scaling.
            loop_tip = _interp_airfoil_loop(stations, 1.0)
            thk_tip = _scale_dist_at(getattr(p, 'thicknessDist', SCALE_DISTS[0]),
                                     getattr(p, 'thicknessScale', 1.0),
                                     getattr(p, 'thicknessMidScale', getattr(p, 'thicknessScale', 1.0)),
                                     getattr(p, 'thicknessTipScale', getattr(p, 'thicknessScale', 1.0)),
                                     1.0, getattr(p, 'midFrac', 0.5))
            cmb_tip = _scale_dist_at(getattr(p, 'camberDist', SCALE_DISTS[0]),
                                     getattr(p, 'camberScale', 1.0),
                                     getattr(p, 'camberMidScale', getattr(p, 'camberScale', 1.0)),
                                     getattr(p, 'camberTipScale', getattr(p, 'camberScale', 1.0)),
                                     1.0, getattr(p, 'midFrac', 0.5))
            loop_tip = scale_thickness_and_camber(loop_tip, thk_tip, cmb_tip)

            # Tip twist is held constant through the cap.
            tw_tip = _twist_at(p, 1.0)

            # Tip chord from planform at t=1
            c_tip = _chord_at(p, 1.0)
            c_min = max(0.2, 0.01 * max(1e-6, getattr(p, 'rootChord_cm', 1.0)))

            for i in range(1, cap_n + 1):
                u = i / cap_n
                ue = cap_ease(u)
                y_cm = semiSpan + cap_len * ue

                # chord shrinks from tip to end_ratio*tip (tapered/pointed cap)
                ratio = 1.0 + (end_ratio - 1.0) * ue
                chord = max(c_min, c_tip * ratio)

                # Compute offsets using sweep/dihedral extrapolated to y_cm
                x_refline = xRefRoot + _x_sweep_offset_at(p, semiSpan, y_cm)
                xOffset = x_refline - getattr(p, 'sweepRefFrac', 0.25) * chord
                zOffset = _z_offset_at(p, semiSpan, y_cm)

                xp = xOffset + twistPivotFrac * chord
                zp = zOffset

                pts = []
                for xn, zn in loop_tip:
                    x = xOffset + xn * chord
                    z = zOffset + zn * chord
                    x, z = rotate_xz_about(x, z, xp, zp, tw_tip)
                    pts.append((x, z))

                prof, le_w, te_w = add_airfoil_sketch(y_cm, pts)
                profiles.append(prof)
                le_pts_world.append(le_w)
                te_pts_world.append(te_w)

    # Rails through base stations (avoid degenerate cap profiles in rail-guided lofts)
    base_profile_count = len(ts)

    rail_sk = sketches.add(comp.xYConstructionPlane)
    rail_sk.is3D = True

    def add_3d_spline(points_world):
        oc = adsk.core.ObjectCollection.create()
        for pnt in points_world:
            oc.add(pnt)
        spl = rail_sk.sketchCurves.sketchFittedSplines.add(oc)
        try:
            spl.isConstruction = True
        except:
            pass
        return spl

    # Build rails only through the original stations (root -> tip). When a pointed cap is enabled,
    # extra profiles beyond the tip can become near-degenerate and cause self-intersection if used
    # as rail constraints.
    le_rail = add_3d_spline(le_pts_world[:base_profile_count])
    te_rail = add_3d_spline(te_pts_world[:base_profile_count])

    lofts = comp.features.loftFeatures

    def loft_with_sections(section_profiles, use_rails=True,
                           operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                           le=None, te=None):
        loftInput = lofts.createInput(operation)
        for pr in section_profiles:
            loftInput.loftSections.add(pr)
        if use_rails and (le is not None) and (te is not None):
            loftInput.centerLineOrRails.addRail(le)
            loftInput.centerLineOrRails.addRail(te)
        return lofts.add(loftInput)

    loft = None
    try:
        has_cap = (len(profiles) > base_profile_count)

        if has_cap:
            # Best-practice for pointy caps: loft the main wing with rails, then loft the cap
            # without rails and join. This avoids rail-driven folds near the near-zero chord.
            loft_main = loft_with_sections(
                profiles[:base_profile_count],
                use_rails=True,
                operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                le=le_rail, te=te_rail
            )
            try:
                loft_cap = loft_with_sections(
                    profiles[base_profile_count-1:],
                    use_rails=False,
                    operation=adsk.fusion.FeatureOperations.JoinFeatureOperation
                )
            except:
                # If joining the cap fails, remove the partial main loft and fall back.
                try:
                    loft_main.deleteMe()
                except:
                    pass
                raise
            loft = loft_main
        else:
            loft = loft_with_sections(
                profiles,
                use_rails=True,
                operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                le=le_rail, te=te_rail
            )

    except:
        rails_error = traceback.format_exc()
        try:
            loft = loft_with_sections(
                profiles,
                use_rails=False,
                operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
            )
            ui.messageBox('Note: Loft guide rails failed; created loft without rails.\n\nDetails:\n' + rails_error)
        except:
            k = max(1, min(len(profiles)-2, int(round(p.midFrac*(len(profiles)-1)))))
            loft_a = loft_with_sections(
                profiles[:k+1],
                use_rails=False,
                operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
            )
            try:
                loft = loft_with_sections(
                    profiles[k:],
                    use_rails=False,
                    operation=adsk.fusion.FeatureOperations.JoinFeatureOperation
                )
            except:
                loft = loft_with_sections(
                    profiles[k:],
                    use_rails=False,
                    operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
                )
            ui.messageBox('Note: Single loft failed; created two lofts (root->break, break->tip).')

# Winglet (optional, built after main wing)
    if getattr(p, 'wingletEnable', False):
        try:
            # Build a simple 2-section winglet loft from the wing tip profile to a winglet-tip profile.
            # Use the ACTUAL tip LE/TE points returned by the tip airfoil sketch so the winglet
            # inherits sweep/dihedral/twist correctly and rails intersect profiles.
            if len(profiles) < 1 or len(le_pts_world) < 1 or len(te_pts_world) < 1:
                raise RuntimeError('Winglet requested but no tip profile points were available.')

            le_tip = le_pts_world[-1]
            te_tip = te_pts_world[-1]

            # Tip chord direction from actual geometry (includes tip twist).
            chord_vec = adsk.core.Vector3D.create(te_tip.x - le_tip.x, te_tip.y - le_tip.y, te_tip.z - le_tip.z)
            chord_len = chord_vec.length
            if chord_len < 1e-6:
                raise RuntimeError('Tip chord length too small for winglet.')
            chord_vec.normalize()

            # Winglet span direction (cant) in YZ, then orthogonalize against chord to keep section plane well-defined.
            cant = getattr(p, 'wingletCant_rad', math.radians(80.0))
            # Winglet cant convention:
            #   +cant => winglet points UP (+Z)
            #   -cant => winglet points DOWN (-Z)
            # (Independent of flipZ, which is for airfoil section orientation.)
            span0 = adsk.core.Vector3D.create(0.0, math.cos(cant), math.sin(cant))

            # Orthogonalize span direction vs chord direction (important when tip has twist).
            dp = span0.dotProduct(chord_vec)
            span_vec = adsk.core.Vector3D.create(span0.x - chord_vec.x * dp,
                                                 span0.y - chord_vec.y * dp,
                                                 span0.z - chord_vec.z * dp)
            if span_vec.length < 1e-6:
                span_vec = span0
            span_vec.normalize()
            span_n = adsk.core.Vector3D.create(span_vec.x, span_vec.y, span_vec.z)  # keep normal along winglet axis; handedness handled via section frame

            # Reference point at the chosen sweep reference fraction on the tip chord line.
            ref_frac = getattr(p, 'sweepRefFrac', 0.25)
            ref_tip = adsk.core.Point3D.create(le_tip.x + chord_vec.x * (ref_frac * chord_len),
                                               le_tip.y + chord_vec.y * (ref_frac * chord_len),
                                               le_tip.z + chord_vec.z * (ref_frac * chord_len))

            H = float(getattr(p, 'wingletHeight_cm', 0.0))
            if H <= 1e-6:
                raise RuntimeError('Winglet height must be > 0.')

            # Sweep of the winglet reference line (aft shift per unit height).
            winglet_sweep = float(getattr(p, 'wingletSweep_rad', 0.0))
            dx_ref = H * math.tan(winglet_sweep)

            # Reference point at winglet tip
            ref_top = adsk.core.Point3D.create(ref_tip.x + span_vec.x * H + chord_vec.x * dx_ref,
                                               ref_tip.y + span_vec.y * H + chord_vec.y * dx_ref,
                                               ref_tip.z + span_vec.z * H + chord_vec.z * dx_ref)

            # Winglet chord ratio (tip chord = wing tip chord * ratio)
            chord_ratio = float(getattr(p, 'wingletTipChordRatio', getattr(p, 'wingletChordRatio', 0.5)))
            chord_ratio = max(0.05, min(3.0, chord_ratio))
            chord_len_top = chord_len * chord_ratio

            # Additional winglet incidence (twist) at winglet tip relative to wing tip, about winglet span axis.
            tw_delta = float(getattr(p, 'wingletTwistTipDelta_rad', 0.0))

            # Rotate chord direction about span axis by tw_delta (Rodrigues)
            def _rot_vec_about_axis(v, axis, theta):
                # rotate vector v about unit axis by theta
                ax = adsk.core.Vector3D.create(axis.x, axis.y, axis.z)
                if ax.length < 1e-9:
                    return adsk.core.Vector3D.create(v.x, v.y, v.z)
                ax.normalize()
                vx, vy, vz = v.x, v.y, v.z
                kx, ky, kz = ax.x, ax.y, ax.z
                ct = math.cos(theta)
                st = math.sin(theta)
                dot = vx*kx + vy*ky + vz*kz
                cx = ky*vz - kz*vy
                cy = kz*vx - kx*vz
                cz = kx*vy - ky*vx
                rx = vx*ct + cx*st + kx*dot*(1-ct)
                ry = vy*ct + cy*st + ky*dot*(1-ct)
                rz = vz*ct + cz*st + kz*dot*(1-ct)
                return adsk.core.Vector3D.create(rx, ry, rz)

            chord_vec_top = _rot_vec_about_axis(chord_vec, span_vec, tw_delta)
            if chord_vec_top.length < 1e-9:
                chord_vec_top = chord_vec
            chord_vec_top.normalize()

            # Compute LE/TE at winglet tip so that the *reference line* stays continuous.
            le_top = adsk.core.Point3D.create(ref_top.x - chord_vec_top.x * (ref_frac * chord_len_top),
                                              ref_top.y - chord_vec_top.y * (ref_frac * chord_len_top),
                                              ref_top.z - chord_vec_top.z * (ref_frac * chord_len_top))
            te_top = adsk.core.Point3D.create(le_top.x + chord_vec_top.x * chord_len_top,
                                              le_top.y + chord_vec_top.y * chord_len_top,
                                              le_top.z + chord_vec_top.z * chord_len_top)

            # Thickness direction for this section plane.
            thick_vec = chord_vec_top.crossProduct(span_vec)
            if thick_vec.length < 1e-9:
                # fallback to something sensible
                thick_vec = adsk.core.Vector3D.create(0, 0, 1)
            thick_vec.normalize()

            # Select winglet airfoil loop (either reuse wing tip, or load separate DAT if provided)
            if getattr(p, 'wingletUseTipAirfoil', True) or (not getattr(p, 'wingletPath', '')):
                loop2d = _interp_airfoil_loop(stations, 1.0)
            else:
                _, wpts = load_dat_airfoil(getattr(p, 'wingletPath', ''))
                loop2d = normalize_airfoil(wpts)
                loop2d = resample_selig(loop2d, n=p.resampleN)
            thk_wlt = _scale_dist_at(getattr(p, 'thicknessDist', SCALE_DISTS[0]),
                                     getattr(p, 'thicknessScale', 1.0),
                                     getattr(p, 'thicknessMidScale', getattr(p, 'thicknessScale', 1.0)),
                                     getattr(p, 'thicknessTipScale', getattr(p, 'thicknessScale', 1.0)),
                                     1.0, getattr(p, 'midFrac', 0.5))
            cmb_wlt = _scale_dist_at(getattr(p, 'camberDist', SCALE_DISTS[0]),
                                     getattr(p, 'camberScale', 1.0),
                                     getattr(p, 'camberMidScale', getattr(p, 'camberScale', 1.0)),
                                     getattr(p, 'camberTipScale', getattr(p, 'camberScale', 1.0)),
                                     1.0, getattr(p, 'midFrac', 0.5))
            thk_wlt *= float(getattr(p, 'wingletThicknessMult', 1.0))
            cmb_wlt *= float(getattr(p, 'wingletCamberMult', 1.0))
            loop2d = scale_thickness_and_camber(loop2d, thk_wlt, cmb_wlt)

                        # Build the winglet-tip profile sketch on a robust plane.
            # Instead of setByThreePoints (which can throw InternalValidationError in some cases),
            # create a plane PERPENDICULAR to the winglet span axis at the winglet tip reference point.
            # This uniquely defines the section plane (normal = span_vec) and is very stable.
            pln_inp = planes.createInput()

            # Build the winglet tip section plane.
            # Fusion can be picky about creating ConstructionAxis features from free points,
            # so we prefer defining the plane via a temporary Line3D (stable) and only fall back
            # to a construction axis if needed.
            ax_len = max(H, 10.0)  # ensure non-zero length (cm)
            p_axis2 = adsk.core.Point3D.create(
                ref_top.x + span_vec.x * ax_len,
                ref_top.y + span_vec.y * ax_len,
                ref_top.z + span_vec.z * ax_len
            )

            pln_wlt = None

            # Preferred: create the winglet plane NORMAL to the winglet span axis at ref_top.
            # NOTE: In the Fusion API the "perpendicular at point" input exists for AXES, not for PLANES.
            # For planes, use setByDistanceOnPath (plane normal to a path) on a 3D sketch line instead.
            try:
                axis_sk = sketches.add(comp.xYConstructionPlane)
                axis_sk.is3D = True
                axis_ln = axis_sk.sketchCurves.sketchLines.addByTwoPoints(ref_top, p_axis2)
                try:
                    axis_ln.isConstruction = True
                except:
                    pass

                pln_inp2 = planes.createInput()
                try:
                    # Distance is measured from the start of the path; 0 places the plane at ref_top.
                    pln_inp2.setByDistanceOnPath(axis_ln, adsk.core.ValueInput.createByReal(0.0))
                    pln_wlt = planes.add(pln_inp2)
                except:
                    # Some builds can be picky about exactly-zero; use a tiny distance instead.
                    pln_inp2.setByDistanceOnPath(axis_ln, adsk.core.ValueInput.createByReal(1e-4))
                    pln_wlt = planes.add(pln_inp2)

            except:
                # Fallback: define the plane by three *point entities* (SketchPoints).
                # Passing raw Point3D objects into setByThreePoints can fail in parametric mode.
                off = max(chord_len_top * 0.1, 1.0)
                p3 = adsk.core.Point3D.create(
                    ref_top.x + thick_vec.x * off,
                    ref_top.y + thick_vec.y * off,
                    ref_top.z + thick_vec.z * off
                )

                sk_p = sketches.add(comp.xYConstructionPlane)
                sk_p.is3D = True
                sp1 = sk_p.sketchPoints.add(le_top)
                sp2 = sk_p.sketchPoints.add(te_top)
                sp3 = sk_p.sketchPoints.add(p3)

                pln_inp3 = planes.createInput()
                pln_inp3.setByThreePoints(sp1, sp2, sp3)
                pln_wlt = planes.add(pln_inp3)

            if not pln_wlt:
                raise RuntimeError('Winglet: failed to create a valid section plane.')

            sk_wlt = sketches.add(pln_wlt)
            crv = sk_wlt.sketchCurves

            # Convert model points to sketch space
            oc = adsk.core.ObjectCollection.create()
            # Find LE index for later (min x in normalized space)
            i_le = 0
            minx = 1e9
            for i,(xn,zn) in enumerate(loop2d):
                if xn < minx:
                    minx = xn
                    i_le = i

            # Create model points for the loop
            model_pts = []
            for (xn, zn) in loop2d:
                # Flip in thickness direction if requested (matches main wing)
                zz = (-zn if getattr(p, 'flipZ', True) else zn)
                mp = adsk.core.Point3D.create(
                    le_top.x + chord_vec_top.x * (xn * chord_len_top) + thick_vec.x * (zz * chord_len_top),
                    le_top.y + chord_vec_top.y * (xn * chord_len_top) + thick_vec.y * (zz * chord_len_top),
                    le_top.z + chord_vec_top.z * (xn * chord_len_top) + thick_vec.z * (zz * chord_len_top),
                )
                model_pts.append(mp)
                sp = sk_wlt.modelToSketchSpace(mp)
                oc.add(adsk.core.Point3D.create(sp.x, sp.y, 0))

            spl = crv.sketchFittedSplines.add(oc)
            try:
                spl.isConstruction = False
            except:
                pass

            # Close TE with a straight line (first->last) and make sure a TE-mid point exists on that line.
            p_te_u = sk_wlt.modelToSketchSpace(model_pts[0])
            p_te_l = sk_wlt.modelToSketchSpace(model_pts[-1])
            te_line = crv.sketchLines.addByTwoPoints(
                adsk.core.Point3D.create(p_te_u.x, p_te_u.y, 0),
                adsk.core.Point3D.create(p_te_l.x, p_te_l.y, 0),
            )

            # Create LE/TE sketch points (for chord line + rails) and constrain
            p_le = sk_wlt.modelToSketchSpace(model_pts[i_le])
            le_sp = sk_wlt.sketchPoints.add(adsk.core.Point3D.create(p_le.x, p_le.y, 0))
            try:
                sk_wlt.geometricConstraints.addCoincident(le_sp, spl)
            except:
                pass

            te_mid_model = adsk.core.Point3D.create(
                0.5 * (model_pts[0].x + model_pts[-1].x),
                0.5 * (model_pts[0].y + model_pts[-1].y),
                0.5 * (model_pts[0].z + model_pts[-1].z),
            )
            p_te_mid = sk_wlt.modelToSketchSpace(te_mid_model)
            te_sp = sk_wlt.sketchPoints.add(adsk.core.Point3D.create(p_te_mid.x, p_te_mid.y, 0))
            try:
                sk_wlt.geometricConstraints.addCoincident(te_sp, te_line)
            except:
                pass

            # Center chord construction line
            chord_ln = crv.sketchLines.addByTwoPoints(le_sp, te_sp)
            chord_ln.isConstruction = True

            # Pick the largest profile as the loft section
            prof_wlt = None
            best_area = -1.0
            for i in range(sk_wlt.profiles.count):
                pr = sk_wlt.profiles.item(i)
                try:
                    a = abs(pr.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area)
                except:
                    a = 0.0
                if a > best_area:
                    best_area = a
                    prof_wlt = pr
            if prof_wlt is None:
                raise RuntimeError('Winglet tip profile not found.')

            # Loft the winglet using transition sections (prevents loft failures at high cant angles).
            # The old 2-section loft tried to go from the wing tip station plane (normal ~ +Y) directly to the
            # winglet plane (normal ~ span_vec). At 90° cant those planes are perpendicular, which frequently
            # triggers ASM_BAD_UV_SKIN_DIR / tolerant guide failures. We fix it by inserting intermediate
            # profiles that gradually rotate the section plane/frames from +Y toward span_vec.

            # Pick number of transition steps based on cant magnitude (more steps for bigger bends).
            cant_deg = abs(math.degrees(cant))
            # More transition steps + easing greatly reduces the visible "hinge" at the winglet root.
            n_steps = max(4, int(round(cant_deg / 15.0)))  # 90° -> 6 steps -> 7 profiles total
            n_steps = min(24, n_steps)

            def _smoothstep(t):
                try:
                    t = float(t)
                except:
                    t = 0.0
                if t < 0.0:
                    t = 0.0
                if t > 1.0:
                    t = 1.0
                return t * t * (3.0 - 2.0 * t)

            us_lin = [i / float(n_steps) for i in range(n_steps + 1)]
            us = [_smoothstep(u) for u in us_lin]

            # Wing span normal at the joint (root of winglet).
            ny = adsk.core.Vector3D.create(0.0, 1.0, 0.0)

            # Index of LE point in the normalized loop (min x)
            i_le = 0
            minx = 1e9
            for ii, (xn, zn) in enumerate(loop2d):
                if xn < minx:
                    minx = xn
                    i_le = ii

            # Build a shared 3D sketch to host all winglet section axis-lines (keeps browser cleaner).
            wlt_axis_sk = sketches.add(comp.xYConstructionPlane)
            wlt_axis_sk.is3D = True
            wlt_axis_sk.name = 'Winglet_AxisLines'

            # Winglet section orientation continuity (prevents profile flipping/twisting)
            prev_wlt_ch = None
            prev_wlt_th = None
            prev_wlt_nrm = None

            def _make_winglet_section_profile(ref_pt, nrm, chord_dir_in, chord_len_sec, tag):
                # Ensure vectors are unit/valid.
                nonlocal prev_wlt_ch, prev_wlt_th, prev_wlt_nrm
                nrm = adsk.core.Vector3D.create(nrm.x, nrm.y, nrm.z)
                if nrm.length < 1e-9:
                    nrm = adsk.core.Vector3D.create(0.0, 1.0, 0.0)
                nrm.normalize()

                # Keep normal direction continuous (avoid sudden 180° flips).
                if prev_wlt_nrm is not None:
                    try:
                        if nrm.dotProduct(prev_wlt_nrm) < 0.0:
                            nrm.scaleBy(-1.0)
                    except:
                        pass

                ch = adsk.core.Vector3D.create(chord_dir_in.x, chord_dir_in.y, chord_dir_in.z)
                if ch.length < 1e-9:
                    ch = adsk.core.Vector3D.create(1.0, 0.0, 0.0)
                ch.normalize()

                # Force chord direction to be in the section plane (orthogonal to normal).
                dp = ch.dotProduct(nrm)
                ch = adsk.core.Vector3D.create(ch.x - nrm.x * dp, ch.y - nrm.y * dp, ch.z - nrm.z * dp)
                if ch.length < 1e-7:
                    # Fallback to original chord if projection collapses.
                    ch = adsk.core.Vector3D.create(chord_dir_in.x, chord_dir_in.y, chord_dir_in.z)
                if ch.length < 1e-9:
                    ch = adsk.core.Vector3D.create(1.0, 0.0, 0.0)
                ch.normalize()

                # Keep chord direction continuous (avoid profile reversal between sections).
                if prev_wlt_ch is not None:
                    try:
                        if ch.dotProduct(prev_wlt_ch) < 0.0:
                            ch.scaleBy(-1.0)
                    except:
                        pass

                # Section thickness direction (span/chord right-hand).
                th = nrm.crossProduct(ch)
                if th.length < 1e-9:
                    th = adsk.core.Vector3D.create(0.0, 0.0, 1.0)

                # Keep thickness direction continuous (prevents 180° flips / loft twists).
                if prev_wlt_th is not None:
                    try:
                        if th.dotProduct(prev_wlt_th) < 0.0:
                            th.scaleBy(-1.0)
                    except:
                        pass
                th.normalize()
                # Ensure right-handed local frame so the airfoil isn't mirrored
                try:
                    _chk = ch.crossProduct(th)
                    if _chk.dotProduct(nrm) < 0:
                        th.scaleBy(-1)
                except:
                    pass

                # Update continuity vectors for next section.
                prev_wlt_ch = adsk.core.Vector3D.create(ch.x, ch.y, ch.z)
                prev_wlt_th = adsk.core.Vector3D.create(th.x, th.y, th.z)
                prev_wlt_nrm = adsk.core.Vector3D.create(nrm.x, nrm.y, nrm.z)
# Build a deterministic section plane so sketch axes don't randomly flip between sections.
                # Use 3 points: LE, TE, and a small offset in thickness direction.
                le_w = adsk.core.Point3D.create(ref_pt.x - ch.x * (ref_frac * chord_len_sec),
                                               ref_pt.y - ch.y * (ref_frac * chord_len_sec),
                                               ref_pt.z - ch.z * (ref_frac * chord_len_sec))
                te_w = adsk.core.Point3D.create(le_w.x + ch.x * chord_len_sec,
                                               le_w.y + ch.y * chord_len_sec,
                                               le_w.z + ch.z * chord_len_sec)

                off = max(1e-3, chord_len_sec * 0.05)
                p3 = adsk.core.Point3D.create(le_w.x + th.x * off,
                                             le_w.y + th.y * off,
                                             le_w.z + th.z * off)

                # Use SketchPoints as plane seeds (Point3D seeds can trip InternalValidationError on some Fusion builds)
                try:
                    sp_a = wlt_axis_sk.sketchPoints.add(le_w)
                    sp_b = wlt_axis_sk.sketchPoints.add(te_w)
                    sp_c = wlt_axis_sk.sketchPoints.add(p3)
                except:
                    tmp_seed = sketches.add(comp.xYConstructionPlane)
                    tmp_seed.is3D = True
                    tmp_seed.name = f"Winglet_PlaneSeeds_{ui_idx}"
                    sp_a = tmp_seed.sketchPoints.add(le_w)
                    sp_b = tmp_seed.sketchPoints.add(te_w)
                    sp_c = tmp_seed.sketchPoints.add(p3)

                pln_inp = planes.createInput()
                pln_inp.setByThreePoints(sp_a, sp_b, sp_c)
                pln = planes.add(pln_inp)

                sk = sketches.add(pln)
                sk.name = 'Winglet_Section_' + str(tag)

                constraints = sk.geometricConstraints
                crv = sk.sketchCurves
                pts = adsk.core.ObjectCollection.create()

                # Compute LE & TE in world space for this section from refline.
                le_w = adsk.core.Point3D.create(ref_pt.x - ch.x * (ref_frac * chord_len_sec),
                                               ref_pt.y - ch.y * (ref_frac * chord_len_sec),
                                               ref_pt.z - ch.z * (ref_frac * chord_len_sec))
                te_w = adsk.core.Point3D.create(le_w.x + ch.x * chord_len_sec,
                                               le_w.y + ch.y * chord_len_sec,
                                               le_w.z + ch.z * chord_len_sec)

                model_pts = []
                for (xn, zn) in loop2d:
                    zz = (-zn if p.flipZ else zn)
                    mp = adsk.core.Point3D.create(le_w.x + ch.x * (xn * chord_len_sec) + th.x * (zz * chord_len_sec),
                                                 le_w.y + ch.y * (xn * chord_len_sec) + th.y * (zz * chord_len_sec),
                                                 le_w.z + ch.z * (xn * chord_len_sec) + th.z * (zz * chord_len_sec))
                    model_pts.append(mp)
                    sp = sk.modelToSketchSpace(mp)
                    pts.add(adsk.core.Point3D.create(sp.x, sp.y, 0.0))

                spl = crv.sketchFittedSplines.add(pts)
                # Close the loop at TE.
                spl_end = spl.endSketchPoint.geometry
                spl_start = spl.startSketchPoint.geometry
                te_ln = crv.sketchLines.addByTwoPoints(spl_end, spl_start)

                # Add LE and TE midpoints constrained to the curve for robust rail endpoints.
                sp_le = sk.modelToSketchSpace(model_pts[i_le])
                le_sp = sk.sketchPoints.add(adsk.core.Point3D.create(sp_le.x, sp_le.y, 0.0))
                le_sp.isFixed = False
                try:
                    constraints.addCoincident(le_sp, spl)
                except:
                    pass

                sp_te_a = spl.startSketchPoint.geometry
                sp_te_b = spl.endSketchPoint.geometry
                sp_te_mid = adsk.core.Point3D.create((sp_te_a.x + sp_te_b.x) * 0.5, (sp_te_a.y + sp_te_b.y) * 0.5, 0.0)
                te_mid_sp = sk.sketchPoints.add(sp_te_mid)
                te_mid_sp.isFixed = False
                try:
                    constraints.addCoincident(te_mid_sp, te_ln)
                except:
                    pass
                best = sk.profiles.item(0)
                best_area = -1.0
                for _pi in range(sk.profiles.count):
                    pr = sk.profiles.item(_pi)
                    try:
                        ap = pr.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                        a = abs(ap.area)
                    except:
                        a = 0.0
                    if a > best_area:
                        best_area = a
                        best = pr

                return best, le_sp.worldGeometry, te_mid_sp.worldGeometry

            # Build section profiles and rail points.
            wlt_profiles = []
            le_wlt_pts = adsk.core.ObjectCollection.create()
            te_wlt_pts = adsk.core.ObjectCollection.create()
            # Seed the winglet with the *exact* wing-tip profile from the main wing loft.
            # This guarantees perfect geometric alignment at the joint (no tiny offsets from re-sampling,
            # ref-point drift, or frame math).
            seed_tip_profile = None
            try:
                if profiles and len(profiles) > 0:
                    seed_tip_profile = profiles[-1]
            except:
                seed_tip_profile = None

            if seed_tip_profile is not None:
                try:
                    wlt_profiles.append(seed_tip_profile)
                    # Use the *actual* wing tip LE/TE-mid points used by the main wing rails.
                    le_wlt_pts.add(le_tip)
                    te_wlt_pts.add(te_tip)
                except:
                    pass


            # Winglet spine: use a cubic Hermite curve so the winglet starts tangent to the wing (along +Y)
            # and gradually bends toward the final winglet span direction (span_vec). This removes the harsh
            # "kink" at the root compared to a straight-line spine.
            ref_top = adsk.core.Point3D.create(ref_tip.x + span_vec.x * H + chord_vec.x * dx_ref,
                                              ref_tip.y + span_vec.y * H + chord_vec.y * dx_ref,
                                              ref_tip.z + span_vec.z * H + chord_vec.z * dx_ref)

            tmag = 0.65 * H  # tangent magnitude (tune for how "blended" the winglet looks)
            t0 = adsk.core.Vector3D.create(ny.x * tmag, ny.y * tmag, ny.z * tmag)  # start tangent in wing plane
            t1 = adsk.core.Vector3D.create(span_vec.x * tmag, span_vec.y * tmag, span_vec.z * tmag)  # end tangent

            def _hermite_point(u_h, p0, p1, v0, v1):
                try:
                    u_h = float(u_h)
                except:
                    u_h = 0.0
                if u_h < 0.0:
                    u_h = 0.0
                if u_h > 1.0:
                    u_h = 1.0
                u2 = u_h * u_h
                u3 = u2 * u_h
                h00 = 2.0 * u3 - 3.0 * u2 + 1.0
                h10 = u3 - 2.0 * u2 + u_h
                h01 = -2.0 * u3 + 3.0 * u2
                h11 = u3 - u2
                return adsk.core.Point3D.create(
                    p0.x * h00 + v0.x * h10 + p1.x * h01 + v1.x * h11,
                    p0.y * h00 + v0.y * h10 + p1.y * h01 + v1.y * h11,
                    p0.z * h00 + v0.z * h10 + p1.z * h01 + v1.z * h11
                )


            start_idx = 0
            if seed_tip_profile is not None:
                start_idx = 1  # u=0 already covered by the exact wing-tip profile

                # Seed continuity vectors from the wing-tip frame so the first generated winglet section
                # cannot flip relative to the reused tip profile.
                try:
                    prev_wlt_ch = adsk.core.Vector3D.create(chord_vec.x, chord_vec.y, chord_vec.z)
                    if prev_wlt_ch.length > 1e-9:
                        prev_wlt_ch.normalize()
                except:
                    prev_wlt_ch = None
                try:
                    # normal at u=0 is +Y (wing span direction)
                    prev_wlt_nrm = adsk.core.Vector3D.create(ny.x, ny.y, ny.z)
                    if prev_wlt_nrm.length > 1e-9:
                        prev_wlt_nrm.normalize()
                except:
                    prev_wlt_nrm = None
                try:
                    if prev_wlt_ch is not None and prev_wlt_nrm is not None:
                        prev_wlt_th = prev_wlt_nrm.crossProduct(prev_wlt_ch)
                        if prev_wlt_th.length > 1e-9:
                            prev_wlt_th.normalize()
                    else:
                        prev_wlt_th = None
                except:
                    prev_wlt_th = None

            for ui_idx in range(start_idx, len(us_lin)):

                u_lin = us_lin[ui_idx]
                u = us[ui_idx]
                # Interpolate plane normal from +Y toward the flipped winglet span direction (for outward-facing airfoil).
                nrm_u = adsk.core.Vector3D.create(ny.x * (1.0 - u) + span_n.x * u,
                                                 ny.y * (1.0 - u) + span_n.y * u,
                                                 ny.z * (1.0 - u) + span_n.z * u)
                if nrm_u.length < 1e-9:
                    nrm_u = adsk.core.Vector3D.create(0.0, 1.0, 0.0)
                nrm_u.normalize()

                # Reference point along winglet spine (curved Hermite path).
                ref_u = _hermite_point(u, ref_tip, ref_top, t0, t1)

                # Chord length interpolates from wing-tip chord to winglet-tip chord.
                chord_len_u = chord_len * (1.0 - u) + chord_len_top * u

                # Chord direction: apply winglet twist gradually (about final span axis), then keep it in the section plane.
                ch_u = _rot_vec_about_axis(chord_vec, span_vec, tw_delta * u)
                ch_u.normalize()

                prof_u, le_u, te_u = _make_winglet_section_profile(ref_u, nrm_u, ch_u, chord_len_u, ui_idx)
                wlt_profiles.append(prof_u)
                le_wlt_pts.add(le_u)
                te_wlt_pts.add(te_u)

            # Build 3D rails through the LE and TE points.
            wlt_rail_sk = sketches.add(comp.xYConstructionPlane)
            wlt_rail_sk.is3D = True
            wlt_rail_sk.name = 'Winglet_Rails'
            try:
                le_rail = wlt_rail_sk.sketchCurves.sketchControlPointSplines.add(le_wlt_pts)
                te_rail = wlt_rail_sk.sketchCurves.sketchControlPointSplines.add(te_wlt_pts)
            except:
                le_rail = wlt_rail_sk.sketchCurves.sketchFittedSplines.add(le_wlt_pts)
                te_rail = wlt_rail_sk.sketchCurves.sketchFittedSplines.add(te_wlt_pts)

            # Winglet loft: try rails-first (solid), then no-rails (solid), then rails (surface), then no-rails (surface).
            def _try_winglet_loft(use_rails: bool, solid: bool):
                li = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
                for pr in wlt_profiles:
                    li.loftSections.add(pr)
                try:
                    li.isSolid = bool(solid)
                except:
                    pass
                if use_rails:
                    try:
                        li.centerLineOrRails.addRail(le_rail)
                        li.centerLineOrRails.addRail(te_rail)
                    except:
                        pass
                return lofts.add(li)

            winglet_loft = None
            _errs = []
            for (_use_rails, _solid) in ((True, True), (False, True), (True, False), (False, False)):
                try:
                    winglet_loft = _try_winglet_loft(_use_rails, _solid)
                    break
                except Exception:
                    _errs.append(("rails" if _use_rails else "no-rails") + (" solid" if _solid else " surface") + ":\n" + traceback.format_exc())
                    winglet_loft = None

            if winglet_loft is None:
                raise RuntimeError("Winglet loft failed in all modes:\n\n" + "\n---\n".join(_errs))


            winglet_body = winglet_loft.bodies.item(0)
            winglet_body.name = 'Winglet'
        except Exception:
            ui.messageBox('Winglet failed:\n' + traceback.format_exc())

    if p.mirror:
        try:
            if comp.bRepBodies.count > 0:
                mirrorFeats = comp.features.mirrorFeatures
                objs = adsk.core.ObjectCollection.create()
                for i in range(comp.bRepBodies.count):
                    objs.add(comp.bRepBodies.item(i))
                mirIn = mirrorFeats.createInput(objs, comp.xZConstructionPlane)
                mirIn.isCombine = False
                mirrorFeats.add(mirIn)
        except:
            pass

    return comp

# -----------------------------
# UI helpers
# -----------------------------

def _file_dialog(ui: adsk.core.UserInterface, title='Select airfoil DAT'):
    dlg = ui.createFileDialog()
    dlg.title = title
    dlg.filter = 'Airfoil DAT (*.dat);;All files (*.*)'
    dlg.filterIndex = 0
    dlg.isMultiSelectEnabled = False
    if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.filename

def _child_inputs(container):
    for attr in ('children', 'commandInputs', 'childInputs'):
        try:
            ci = getattr(container, attr)
            if ci is not None:
                return ci
        except:
            pass
    return None

def _find_input(inputs: adsk.core.CommandInputs, input_id: str):
    try:
        it = inputs.itemById(input_id)
        if it is not None:
            return it
    except:
        pass

    try:
        for i in range(inputs.count):
            ci = inputs.item(i)
            try:
                if ci and ci.id == input_id:
                    return ci
            except:
                pass

            try:
                ot = getattr(ci, 'objectType', '')
                if ('TabCommandInput' in ot) or ('GroupCommandInput' in ot):
                    children = _child_inputs(ci)
                    if children is not None:
                        found = _find_input(children, input_id)
                        if found is not None:
                            return found
            except:
                pass
    except:
        pass
    return None

def _get_design_units():
    """Best-effort access to the active Design units manager.
    In some Fusion contexts (command dialogs, special workspaces), app.activeProduct may not cast to Design.
    """
    app = adsk.core.Application.get()

    # Primary path
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design:
            return design.unitsManager
    except:
        pass

    # Fallback: get Design product from the active document
    try:
        doc = app.activeDocument
        if doc:
            prod = doc.products.itemByProductType('DesignProductType')
            design = adsk.fusion.Design.cast(prod)
            if design:
                return design.unitsManager
    except:
        pass

    return None

def _safe_eval(units, expr: str, unit: str):
    try:
        if units:
            return units.evaluateExpression(expr, unit)
    except:
        pass
    try:
        return float(expr)
    except:
        return 0.0

def _format_num(x, digits=4):
    try:
        return f'{x:.{digits}g}'
    except:
        return str(x)

def _compute_outputs_from_inputs(cmdInputs: adsk.core.CommandInputs):
    units = _get_design_units()

    def get(iid):
        return _find_input(cmdInputs, iid)

    spanMode = get('spanMode')
    spanIsTotal = bool(spanMode and spanMode.selectedItem and spanMode.selectedItem.name.startswith('Total'))
    span_cm = _safe_eval(units, get('span').expression, 'cm') if get('span') else 0.0
    semiSpan = 0.5 * span_cm if spanIsTotal else span_cm
    semiSpan = max(1e-9, semiSpan)

    midFrac = float(_safe_eval(units, get('midFrac').expression, '')) if get('midFrac') else 0.5
    midFrac = max(0.01, min(0.99, midFrac))

    rootChord = _safe_eval(units, get('rootChord').expression, 'cm') if get('rootChord') else 0.0
    midChord  = _safe_eval(units, get('midChord').expression, 'cm') if get('midChord') else 0.0
    tipChord  = _safe_eval(units, get('tipChord').expression, 'cm') if get('tipChord') else 0.0

    chordDistName = get('chordDist').selectedItem.name if get('chordDist') else CHORD_DISTS[0]
    polyA0 = float(_safe_eval(units, get('polyA0').expression, '')) if get('polyA0') else 0.0
    polyA1 = float(_safe_eval(units, get('polyA1').expression, '')) if get('polyA1') else 1.0
    polyA2 = float(_safe_eval(units, get('polyA2').expression, '')) if get('polyA2') else 0.0
    polyA3 = float(_safe_eval(units, get('polyA3').expression, '')) if get('polyA3') else 0.0

    sweepDistName = get('sweepDist').selectedItem.name if get('sweepDist') else SWEEP_DISTS[0]
    sweep_rad = get('sweep').value if get('sweep') else 0.0
    sweepInner_rad = get('sweepInner').value if get('sweepInner') else sweep_rad
    sweepOuter_rad = get('sweepOuter').value if get('sweepOuter') else sweep_rad
    sweepBreakFrac = float(_safe_eval(units, get('sweepBreak').expression, '')) if get('sweepBreak') else 0.35
    sweepBreakFrac = max(0.01, min(0.99, sweepBreakFrac))

    sweepRefName = get('sweepRef').selectedItem.name if get('sweepRef') else 'Quarter-chord'
    if sweepRefName.startswith('Leading'):
        sweepRefFrac = 0.0
    elif sweepRefName.startswith('Trailing'):
        sweepRefFrac = 1.0
    else:
        sweepRefFrac = 0.25

    class _TmpP:
        pass
    p = _TmpP()
    p.midFrac = midFrac
    p.rootChord_cm = rootChord
    p.midChord_cm = midChord
    p.tipChord_cm = tipChord
    p.chordDist = chordDistName
    p.polyA0, p.polyA1, p.polyA2, p.polyA3 = polyA0, polyA1, polyA2, polyA3

    class _TmpS:
        pass
    s = _TmpS()
    s.sweepDist = sweepDistName
    s.sweep_rad = sweep_rad
    s.sweepInner_rad = sweepInner_rad
    s.sweepOuter_rad = sweepOuter_rad
    s.sweepBreakFrac = sweepBreakFrac

    def chord_at_t(t):
        return _chord_at(p, t)

    def sweep_x_at_y(y_cm):
        # use the sweep-offset helper but with the tmp sweep struct
        # (same field names as WingParams for these pieces)
        return _x_sweep_offset_at(s, semiSpan, y_cm)

    xRefRoot = sweepRefFrac * rootChord

    def x_refline(y_cm):
        return xRefRoot + sweep_x_at_y(y_cm)

    def x_le(y_cm, c):
        return x_refline(y_cm) - sweepRefFrac * c

    def x_qc(y_cm, c):
        return x_refline(y_cm) + (0.25 - sweepRefFrac) * c

    def x_te(y_cm, c):
        return x_refline(y_cm) + (1.0 - sweepRefFrac) * c

    N = 300
    ys = [semiSpan * i/(N-1) for i in range(N)]
    cs = [max(0.0, chord_at_t(y/semiSpan)) for y in ys]
    dy = ys[1] - ys[0] if N > 1 else semiSpan

    half_area = 0.0
    i2 = 0.0
    i_yc2 = 0.0

    for i in range(N-1):
        c0, c1 = cs[i], cs[i+1]
        y0, y1 = ys[i], ys[i+1]
        half_area += 0.5*(c0 + c1) * dy
        i2 += 0.5*(c0*c0 + c1*c1) * dy
        i_yc2 += 0.5*(y0*c0*c0 + y1*c1*c1) * dy

    total_area = 2.0 * half_area
    total_span = 2.0 * semiSpan
    AR = (total_span * total_span) / max(1e-12, total_area)
    taper = (tipChord / rootChord) if abs(rootChord) > 1e-12 else 0.0

    MAC = (2.0 / max(1e-12, total_area)) * i2
    y_MAC = (2.0 / max(1e-12, total_area * max(1e-12, MAC))) * i_yc2
    c_MAC = chord_at_t(y_MAC / semiSpan)
    xLE_MAC = x_le(y_MAC, c_MAC)

    c_root = chord_at_t(0.0)
    c_tip = chord_at_t(1.0)
    xLE_root, xLE_tip = x_le(0.0, c_root), x_le(semiSpan, c_tip)
    xQC_root, xQC_tip = x_qc(0.0, c_root), x_qc(semiSpan, c_tip)
    xTE_root, xTE_tip = x_te(0.0, c_root), x_te(semiSpan, c_tip)

    sweepLE = math.degrees(math.atan2((xLE_tip - xLE_root), semiSpan))
    sweepQC = math.degrees(math.atan2((xQC_tip - xQC_root), semiSpan))
    sweepTE = math.degrees(math.atan2((xTE_tip - xTE_root), semiSpan))

    return {
        'area_cm2': total_area,
        'AR': AR,
        'taper': taper,
        'MAC_cm': MAC,
        'yMAC_cm': y_MAC,
        'xLE_MAC_cm': xLE_MAC,
        'sweepLE_deg': sweepLE,
        'sweepQC_deg': sweepQC,
        'sweepTE_deg': sweepTE,
    }



def _compute_outputs_from_params(p: WingParams):
    """Compute simple planform outputs from WingParams.

    Notes:
      - Uses the *geometric* half-span. If a tip treatment adds a tip cap (no winglet),
        this will include the cap extension so the area/span/AR reflect the actual outer geometry.
      - Uses the same sweep reference fraction (LE / 1/4 / TE) as the model.
    """
    base_semi = max(1e-9, float(getattr(p, 'span_cm', 0.0)))

    # Tip cap extension (only if enabled and winglet is NOT enabled)
    cap_len = 0.0
    end_ratio = 1.0
    try:
        tip_mode = str(getattr(p, 'tipTreatment', 'Flat cut'))
        if (not getattr(p, 'wingletEnable', False)) and tip_mode and (not tip_mode.lower().startswith('flat')):
            cap_len = max(0.0, float(getattr(p, 'tipCapLength_cm', 0.0)))
            end_ratio = float(getattr(p, 'tipCapEndChordRatio', 0.05))
            end_ratio = max(0.0, min(1.0, end_ratio))
    except:
        cap_len = 0.0
        end_ratio = 1.0

    half_span = base_semi + cap_len

    # Ensure p has the same fields _chord_at/_x_sweep_offset_at expect.
    p.midFrac = max(0.01, min(0.99, float(getattr(p, 'midFrac', 0.5))))

    def chord_main(y_cm):
        # y in [0, base_semi]
        t = 0.0 if base_semi < 1e-9 else max(0.0, min(1.0, y_cm / base_semi))
        return max(0.0, _chord_at(p, t))

    c_root = chord_main(0.0)
    c_tip = chord_main(base_semi)

    # chord along the cap is linear in normalized cap span (0..1) between c_tip and c_end
    c_end = max(0.0, c_tip * end_ratio)

    def chord_y(y_cm):
        if y_cm <= base_semi or cap_len <= 1e-9:
            return chord_main(y_cm)
        ue = max(0.0, min(1.0, (y_cm - base_semi) / cap_len))
        return c_tip + (c_end - c_tip) * ue

    sweepRefFrac = float(getattr(p, 'sweepRefFrac', 0.25))
    xRefRoot = sweepRefFrac * c_root

    def x_refline(y_cm):
        # IMPORTANT: model sweep is based on the original semi-span (base_semi), even for cap stations.
        return xRefRoot + _x_sweep_offset_at(p, base_semi, y_cm)

    def x_le(y_cm, c):
        return x_refline(y_cm) - sweepRefFrac * c

    def x_qc(y_cm, c):
        return x_refline(y_cm) + (0.25 - sweepRefFrac) * c

    def x_te(y_cm, c):
        return x_refline(y_cm) + (1.0 - sweepRefFrac) * c

    # Integrate over half-wing [0, half_span]
    N = 600
    ys = [half_span * i/(N-1) for i in range(N)]
    cs = [chord_y(y) for y in ys]
    dy = ys[1] - ys[0] if N > 1 else half_span

    half_area = 0.0
    i2 = 0.0
    i_yc2 = 0.0
    for i in range(N-1):
        c0, c1 = cs[i], cs[i+1]
        y0, y1 = ys[i], ys[i+1]
        half_area += 0.5*(c0 + c1) * dy
        i2 += 0.5*(c0*c0 + c1*c1) * dy
        i_yc2 += 0.5*(y0*c0*c0 + y1*c1*c1) * dy

    total_area = 2.0 * half_area
    total_span = 2.0 * half_span
    AR = (total_span * total_span) / max(1e-12, total_area)
    taper = (cs[-1] / c_root) if c_root > 1e-12 else 0.0

    MAC = (2.0 / max(1e-12, total_area)) * i2
    y_MAC = (2.0 / max(1e-12, total_area * max(1e-12, MAC))) * i_yc2
    y_MAC = max(0.0, min(half_span, y_MAC))
    c_MAC = chord_y(y_MAC)
    xLE_MAC = x_le(y_MAC, c_MAC)

    # Sweeps from root to actual tip (end of cap if present)
    c_tip_end = chord_y(half_span)
    xLE_root, xLE_tip = x_le(0.0, c_root), x_le(half_span, c_tip_end)
    xQC_root, xQC_tip = x_qc(0.0, c_root), x_qc(half_span, c_tip_end)
    xTE_root, xTE_tip = x_te(0.0, c_root), x_te(half_span, c_tip_end)

    sweepLE = math.degrees(math.atan2((xLE_tip - xLE_root), half_span))
    sweepQC = math.degrees(math.atan2((xQC_tip - xQC_root), half_span))
    sweepTE = math.degrees(math.atan2((xTE_tip - xTE_root), half_span))

    return {
        'area_cm2': total_area,
        'AR': AR,
        'taper': taper,
        'MAC_cm': MAC,
        'yMAC_cm': y_MAC,
        'xLE_MAC_cm': xLE_MAC,
        'sweepLE_deg': sweepLE,
        'sweepQC_deg': sweepQC,
        'sweepTE_deg': sweepTE,
        'span_cm': total_span,
    }


def _write_textconsole(ui: adsk.core.UserInterface, text: str):
    """Write to Fusion's Text Commands palette if available."""
    try:
        pal = ui.palettes.itemById('TextCommands')
        if pal:
            pal.isVisible = True
            pal.writeText(text)
            return True
    except:
        pass
    return False


def _print_planform_outputs(ui: adsk.core.UserInterface, units, out: dict):
    """Print outputs to the Text Commands palette (preferred) or via messageBox."""
    def fmt_len_cm(v_cm):
        try:
            u = units.defaultLengthUnits if units else 'cm'
            if units:
                v = units.convert(v_cm, 'cm', u)
                return f'{_format_num(v, 6)} {u}'
        except:
            pass
        return f'{_format_num(v_cm, 6)} cm'

    def fmt_area_cm2(a_cm2):
        try:
            u = units.defaultLengthUnits if units else 'cm'
            if units:
                v = units.convert(a_cm2, 'cm^2', f'{u}^2')
                return f'{_format_num(v, 8)} {u}^2'
        except:
            pass
        return f'{_format_num(a_cm2, 8)} cm^2'

    msg = (
        '\n=== Wing planform outputs ===\n'
        f'Total span: {fmt_len_cm(out.get("span_cm", 0.0))}\n'
        f'Area:       {fmt_area_cm2(out.get("area_cm2", 0.0))}\n'
        f'AR:         {_format_num(out.get("AR", 0.0), 8)}\n'
        f'Taper:      {_format_num(out.get("taper", 0.0), 8)}\n'
        f'MAC:        {fmt_len_cm(out.get("MAC_cm", 0.0))}\n'
        f'MAC y:      {fmt_len_cm(out.get("yMAC_cm", 0.0))}\n'
        f'MAC LE x:   {fmt_len_cm(out.get("xLE_MAC_cm", 0.0))}\n'
        f'Sweep (LE): {_format_num(out.get("sweepLE_deg", 0.0), 8)} deg\n'
        f'Sweep (1/4):{_format_num(out.get("sweepQC_deg", 0.0), 8)} deg\n'
        f'Sweep (TE): {_format_num(out.get("sweepTE_deg", 0.0), 8)} deg\n'
    )

    if not _write_textconsole(ui, msg):
        try:
            ui.messageBox(msg)
        except:
            pass



def _print_run_params(ui: adsk.core.UserInterface, units, p, foil_meta=None):
    """Dump selected UI parameters to the Text Commands console for debugging/history."""
    try:
        # Helpers
        def _deg(rad):
            try:
                return math.degrees(float(rad))
            except:
                return 0.0

        def _fmt_num(v, n=6):
            try:
                return f"{float(v):.{n}g}"
            except:
                return str(v)

        def _fmt_len_cm(v_cm):
            try:
                if units:
                    u = units.defaultLengthUnits
                    v = units.convert(float(v_cm), 'cm', u)
                    return f"{_fmt_num(v, 6)} {u}"
            except:
                pass
            return f"{_fmt_num(v_cm, 6)} cm"

        def _bool(v):
            return 'Yes' if bool(v) else 'No'

        ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        out = []
        out.append(f"\n===== WingBuilder run @ {ts} =====")
        out.append(f"Span input: {'Total span' if getattr(p,'spanIsTotal',False) else 'Semi-span'}")
        out.append(f"Semi-span: {_fmt_len_cm(getattr(p,'span_cm',0.0))}   Mirror: {_bool(getattr(p,'mirror',False))}")
        out.append(f"Stations: {int(getattr(p,'numStations',0))}   Station spacing: {getattr(p,'stationSpacing','Linear')}")

        out.append("\n-- Planform --")
        out.append(f"Mid station frac: {_fmt_num(getattr(p,'midFrac',0.5),6)}")
        out.append(f"Chord dist: {getattr(p,'chordDist','')}  |  Root/Mid/Tip: {_fmt_len_cm(getattr(p,'rootChord_cm',0.0))} / {_fmt_len_cm(getattr(p,'midChord_cm',0.0))} / {_fmt_len_cm(getattr(p,'tipChord_cm',0.0))}")
        if getattr(p,'chordDist','') == 'Custom polynomial (normalized endpoints)':
            out.append(f"Chord poly: a0={_fmt_num(getattr(p,'polyA0',0.0))}, a1={_fmt_num(getattr(p,'polyA1',1.0))}, a2={_fmt_num(getattr(p,'polyA2',0.0))}, a3={_fmt_num(getattr(p,'polyA3',0.0))}")

        out.append("\n-- Angles --")
        out.append(f"Sweep dist: {getattr(p,'sweepDist','')}  |  Ref frac: {_fmt_num(getattr(p,'sweepRefFrac',0.25),6)}")
        if getattr(p,'sweepDist','') == 'Kinked (inner/outer)':
            out.append(f"Sweep inner/outer: {_fmt_num(_deg(getattr(p,'sweepInner_rad',0.0)),6)} / {_fmt_num(_deg(getattr(p,'sweepOuter_rad',0.0)),6)} deg  | break: {_fmt_num(getattr(p,'sweepBreakFrac',0.35),6)}")
        else:
            out.append(f"Sweep: {_fmt_num(_deg(getattr(p,'sweep_rad',0.0)),6)} deg")

        out.append(f"Dihedral dist: {getattr(p,'dihedralDist','')}")
        if getattr(p,'dihedralDist','') == 'Kinked (inner/outer)':
            out.append(f"Dihedral inner/outer: {_fmt_num(_deg(getattr(p,'dihedralInner_rad',0.0)),6)} / {_fmt_num(_deg(getattr(p,'dihedralOuter_rad',0.0)),6)} deg  | break: {_fmt_num(getattr(p,'dihedralBreakFrac',0.35),6)}")
        else:
            out.append(f"Dihedral: {_fmt_num(_deg(getattr(p,'dihedral_rad',0.0)),6)} deg")

        out.append(f"Twist dist: {getattr(p,'twistDist','')}  | axis: {getattr(p,'twistAxisMode','')}  (frac={_fmt_num(getattr(p,'twistAxisFrac',0.25),6)})")
        if getattr(p,'twistDist','') == 'Piecewise linear (root-mid-tip)':
            out.append(f"Twist root/mid/tip: {_fmt_num(_deg(getattr(p,'twistRoot_rad',0.0)),6)} / {_fmt_num(_deg(getattr(p,'twistMid_rad',0.0)),6)} / {_fmt_num(_deg(getattr(p,'twistTip_rad',0.0)),6)} deg")
        else:
            out.append(f"Twist root/tip: {_fmt_num(_deg(getattr(p,'twistRoot_rad',0.0)),6)} / {_fmt_num(_deg(getattr(p,'twistTip_rad',0.0)),6)} deg")

        out.append("\n-- Airfoil processing --")
        out.append(f"Resample N: {int(getattr(p,'resampleN',0))}   Flip Z: {_bool(getattr(p,'flipZ',True))}")
        out.append(f"Thickness dist: {getattr(p,'thicknessDist','Constant')}  |  root/mid/tip: {_fmt_num(getattr(p,'thicknessRootScale',1.0),6)} / {_fmt_num(getattr(p,'thicknessMidScale',1.0),6)} / {_fmt_num(getattr(p,'thicknessTipScale',1.0),6)}")
        out.append(f"Camber dist: {getattr(p,'camberDist','Constant')}  |  root/mid/tip: {_fmt_num(getattr(p,'camberRootScale',1.0),6)} / {_fmt_num(getattr(p,'camberMidScale',1.0),6)} / {_fmt_num(getattr(p,'camberTipScale',1.0),6)}")

        # Tip treatment
        if hasattr(p, 'tipTreatment'):
            out.append("\n-- Tip treatment --")
            out.append(f"Mode: {getattr(p,'tipTreatment','Flat cut')}  |  Cap thickness: {_fmt_len_cm(getattr(p,'tipCapThickness_cm',0.0))}")

        # Winglet
        if getattr(p, 'wingletEnable', False):
            out.append("\n-- Winglet --")
            out.append(f"Enable: Yes  |  Height: {_fmt_len_cm(getattr(p,'wingletHeight_cm',0.0))}  |  Cant: {_fmt_num(_deg(getattr(p,'wingletCant_rad',0.0)),6)} deg")
            out.append(f"Sweep: {_fmt_num(_deg(getattr(p,'wingletSweep_rad',0.0)),6)} deg  |  Tip chord ratio: {_fmt_num(getattr(p,'wingletTipChordRatio',1.0),6)}")
            out.append(f"Tip twist delta: {_fmt_num(_deg(getattr(p,'wingletTwistTipDelta_rad',0.0)),6)} deg  |  Use rails: {_bool(getattr(p,'wingletUseRails',True))}")
            out.append(f"Use tip airfoil: {_bool(getattr(p,'wingletUseTipAirfoil',True))}")
            if not getattr(p,'wingletUseTipAirfoil',True):
                out.append(f"Winglet DAT: {getattr(p,'wingletPath','')}")
            if hasattr(p,'wingletRootBlendFrac'):
                out.append(f"Root blend frac: {_fmt_num(getattr(p,'wingletRootBlendFrac',0.25),6)}")

        # Airfoil station meta
        if foil_meta:
            out.append("\n-- Airfoil stations (DAT inputs) --")
            for st in sorted(foil_meta, key=lambda d: d.get('t', 0.0)):
                t = float(st.get('t', 0.0))
                path = st.get('path', '') or ''
                nm = st.get('name', '') or ''
                base = os.path.basename(path) if path else ''
                label = nm if nm and nm != base else base
                if label:
                    out.append(f"t={_fmt_num(t,6)}  {label}  ({path})")
                else:
                    out.append(f"t={_fmt_num(t,6)}  {path}")
        _write_textconsole(ui, "\n".join(out) + "\n")
    except:
        # Never fail the run due to debug printing.
        pass




def _set_text(cmdInputs, iid, text):
    it = _find_input(cmdInputs, iid)
    if it is None:
        return
    try:
        it.text = text
    except:
        try:
            it.value = text
        except:
            pass

def _update_outputs(cmdInputs: adsk.core.CommandInputs):
    try:
        out = _compute_outputs_from_inputs(cmdInputs)
        units = _get_design_units()

        def fmt_len_cm(v_cm):
            if units:
                try:
                    u = units.defaultLengthUnits
                    v = units.convert(v_cm, 'cm', u)
                    return f'{_format_num(v, 6)} {u}'
                except:
                    pass
            return f'{_format_num(v_cm, 6)} cm'

        def fmt_area_cm2(a_cm2):
            if units:
                try:
                    u = units.defaultLengthUnits
                    v = units.convert(a_cm2, 'cm^2', f'{u}^2')
                    return f'{_format_num(v, 8)} {u}^2'
                except:
                    pass
            return f'{_format_num(a_cm2, 8)} cm^2'

        _set_text(cmdInputs, 'outArea', fmt_area_cm2(out['area_cm2']))
        _set_text(cmdInputs, 'outAR', f'{_format_num(out["AR"], 6)}')
        _set_text(cmdInputs, 'outTaper', f'{_format_num(out["taper"], 6)}')
        _set_text(cmdInputs, 'outMAC', fmt_len_cm(out['MAC_cm']))
        _set_text(cmdInputs, 'outMACy', fmt_len_cm(out['yMAC_cm']))
        _set_text(cmdInputs, 'outMACx', fmt_len_cm(out['xLE_MAC_cm']))
        _set_text(cmdInputs, 'outSweepLE', f'{_format_num(out["sweepLE_deg"], 6)} deg')
        _set_text(cmdInputs, 'outSweepQC', f'{_format_num(out["sweepQC_deg"], 6)} deg')
        _set_text(cmdInputs, 'outSweepTE', f'{_format_num(out["sweepTE_deg"], 6)} deg')
    except Exception:
        try:
            _write_textconsole(adsk.core.Application.get().userInterface, "Run parameter dump failed:\n" + traceback.format_exc())
        except:
            pass

def _apply_ui_enable_states(cmdInputs: adsk.core.CommandInputs):
    try:
        def get(iid):
            return _find_input(cmdInputs, iid)

        chordMode = get('chordDist').selectedItem.name if get('chordDist') else CHORD_DISTS[0]
        is_piecewise_chord = (chordMode == CHORD_DISTS[0])
        is_poly = (chordMode == CHORD_DISTS[3])

        if get('midChord'):
            get('midChord').isEnabled = is_piecewise_chord
        for pid in ['polyA0', 'polyA1', 'polyA2', 'polyA3']:
            it = get(pid)
            if it:
                it.isEnabled = is_poly

        dihedralMode = get('dihedralDist').selectedItem.name if get('dihedralDist') else DIHEDRAL_DISTS[0]
        is_kink_dih = (dihedralMode == DIHEDRAL_DISTS[1])
        if get('dihedral'): get('dihedral').isEnabled = not is_kink_dih
        if get('dihInner'): get('dihInner').isEnabled = is_kink_dih
        if get('dihOuter'): get('dihOuter').isEnabled = is_kink_dih
        if get('dihBreak'): get('dihBreak').isEnabled = is_kink_dih

        sweepMode = get('sweepDist').selectedItem.name if get('sweepDist') else SWEEP_DISTS[0]
        is_kink_sw = (sweepMode == SWEEP_DISTS[1])
        if get('sweep'): get('sweep').isEnabled = not is_kink_sw
        if get('sweepInner'): get('sweepInner').isEnabled = is_kink_sw
        if get('sweepOuter'): get('sweepOuter').isEnabled = is_kink_sw
        if get('sweepBreak'): get('sweepBreak').isEnabled = is_kink_sw

        twistMode = get('twistDist').selectedItem.name if get('twistDist') else TWIST_DISTS[0]
        if get('twMid'):
            get('twMid').isEnabled = (twistMode == TWIST_DISTS[0])

        axisMode = get('twistAxis').selectedItem.name if get('twistAxis') else TWIST_AXIS[1]
        if get('twistAxisFrac'):
            get('twistAxisFrac').isEnabled = (axisMode == TWIST_AXIS[3])

        cnt = int(get('airfoilCount').value) if get('airfoilCount') else 3
        cnt = max(2, min(MAX_AIRFOILS, cnt))
        for i in range(MAX_AIRFOILS):
            visible = (i < cnt)
            grp = get(f'foilGrp{i}')
            if grp:
                grp.isVisible = visible
                grp.isEnabled = visible
            # (Group children are typically hidden with the group, but also set explicitly)
            path = get(f'foilPath{i}')
            if path:
                path.isVisible = visible
                path.isEnabled = visible
            browse = get(f'browseFoil{i}')
            if browse:
                browse.isVisible = visible
                browse.isEnabled = visible
            frac = get(f'foilFrac{i}')
            if frac:
                frac.isVisible = visible
                frac.isEnabled = (visible and i != 0 and i != (cnt-1))

        # Winglet UI
        w_en = get('wingletEnable').value if get('wingletEnable') else False
        if get('wingletHeight'): get('wingletHeight').isEnabled = w_en
        if get('wingletCant'):   get('wingletCant').isEnabled   = w_en
        if get('wingletSweep'):  get('wingletSweep').isEnabled  = w_en
        if get('wingletChordRatio'): get('wingletChordRatio').isEnabled = w_en
        if get('wingletTwist'):  get('wingletTwist').isEnabled  = w_en
        if get('wingletRails'):  get('wingletRails').isEnabled  = w_en
        if get('wingletUseTip'):
            get('wingletUseTip').isEnabled = w_en
            use_tip = get('wingletUseTip').value
        else:
            use_tip = True
        if get('wingletPath'):   get('wingletPath').isEnabled   = (w_en and (not use_tip))
        if get('browseWinglet'): get('browseWinglet').isEnabled = (w_en and (not use_tip))


        # Station spacing is always enabled (no gating)

        # Thickness / camber distribution enable states
        try:
            thDist = cmdInputs.itemById('thicknessDist').selectedItem.name
            thMid = cmdInputs.itemById('thicknessMidScale')
            thTip = cmdInputs.itemById('thicknessTipScale')
            if thDist == SCALE_DISTS[0]:  # Constant
                thMid.isEnabled = False
                thTip.isEnabled = False
            elif thDist == SCALE_DISTS[2]:  # Piecewise
                thMid.isEnabled = True
                thTip.isEnabled = True
            else:
                thMid.isEnabled = False
                thTip.isEnabled = True
        except:
            pass

        try:
            cbDist = cmdInputs.itemById('camberDist').selectedItem.name
            cbMid = cmdInputs.itemById('camberMidScale')
            cbTip = cmdInputs.itemById('camberTipScale')
            if cbDist == SCALE_DISTS[0]:  # Constant
                cbMid.isEnabled = False
                cbTip.isEnabled = False
            elif cbDist == SCALE_DISTS[2]:  # Piecewise
                cbMid.isEnabled = True
                cbTip.isEnabled = True
            else:
                cbMid.isEnabled = False
                cbTip.isEnabled = True
        except:
            pass

        # Tip cap controls
        try:
            tipMode = cmdInputs.itemById('tipMode').selectedItem.name
            capEnable = (tipMode != TIP_TREATMENTS[0])

            # Tip treatments are meant for non-winglet tips. Disable them when winglet is enabled
            # to avoid creating degenerate "cap" profiles that the winglet tries to loft from.
            if w_en:
                capEnable = False
                try:
                    cmdInputs.itemById('tipMode').isEnabled = False
                except:
                    pass
            else:
                try:
                    cmdInputs.itemById('tipMode').isEnabled = True
                except:
                    pass

            cmdInputs.itemById('tipCapLength').isEnabled = capEnable
            cmdInputs.itemById('tipEndChordRatio').isEnabled = capEnable
            cmdInputs.itemById('tipCapStations').isEnabled = capEnable
            cmdInputs.itemById('tipCapEase').isEnabled = capEnable
        except:
            pass

        # Winglet scaling multipliers only matter when winglet is enabled
        try:
            cmdInputs.itemById('wingletThicknessMult').isEnabled = w_en
            cmdInputs.itemById('wingletCamberMult').isEnabled = w_en
        except:
            pass

        _update_outputs(cmdInputs)

    except:
        pass

# -----------------------------
# Fusion command handlers
# -----------------------------

class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            tab_airfoils = inputs.addTabCommandInput('tabAirfoils', 'Airfoils')
            tab_planform = inputs.addTabCommandInput('tabPlanform', 'Planform')
            tab_chord    = inputs.addTabCommandInput('tabChord', 'Chord')
            tab_angles   = inputs.addTabCommandInput('tabAngles', 'Angles')
            tab_winglet  = inputs.addTabCommandInput('tabWinglets', 'Winglets')
            tab_adv      = inputs.addTabCommandInput('tabAdvanced', 'Advanced')

            air = _child_inputs(tab_airfoils)
            pln = _child_inputs(tab_planform)
            chd = _child_inputs(tab_chord)
            ang = _child_inputs(tab_angles)
            wlt = _child_inputs(tab_winglet)
            adv = _child_inputs(tab_adv)

            # Airfoils
            air.addIntegerSpinnerCommandInput('airfoilCount', 'Airfoil stations', 2, MAX_AIRFOILS, 1, 3)

            for i in range(MAX_AIRFOILS):
                grp = air.addGroupCommandInput(f'foilGrp{i}', f'Airfoil {i+1}')
                try:
                    grp.isVisible = (i < 3)
                    grp.isEnabled = (i < 3)
                except:
                    pass
                g = _child_inputs(grp)
                g.addStringValueInput(f'foilPath{i}', 'DAT file', '')
                g.addBoolValueInput(f'browseFoil{i}', 'Browse…', False, '', False)
                default_frac = '0' if i == 0 else ('0.5' if i == 1 else ('1' if i == 2 else f'{(i)/(MAX_AIRFOILS-1):.3f}'))
                g.addValueInput(f'foilFrac{i}', 'Span fraction (0..1)', '', adsk.core.ValueInput.createByString(default_frac))

            # Planform
            spanMode = pln.addDropDownCommandInput('spanMode', 'Span input', adsk.core.DropDownStyles.TextListDropDownStyle)
            spanMode.listItems.add('Semi-span (one side)', True, '')
            spanMode.listItems.add('Total span (both sides)', False, '')

            pln.addValueInput('span', 'Span', 'cm', adsk.core.ValueInput.createByString('30 cm'))
            pln.addValueInput('midFrac', 'Mid station (0..1)', '', adsk.core.ValueInput.createByString('0.5'))
            pln.addIntegerSpinnerCommandInput('numStations', 'Spanwise profiles', 3, 35, 1, 7)
            stationSpacing = pln.addDropDownCommandInput('stationSpacing', 'Station spacing', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(STATION_SPACING):
                stationSpacing.listItems.add(name, j == 0, '')

            pln.addSeparatorCommandInput('sepTip')
            tipMode = pln.addDropDownCommandInput('tipMode', 'Tip treatment', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(TIP_TREATMENTS):
                tipMode.listItems.add(name, j == 0, '')
            pln.addValueInput('tipCapLength', 'Tip cap length', 'cm', adsk.core.ValueInput.createByString('2 cm'))
            pln.addValueInput('tipEndChordRatio', 'Tip end chord ratio (to tip chord)', '', adsk.core.ValueInput.createByString('0.2'))
            pln.addIntegerSpinnerCommandInput('tipCapStations', 'Tip cap profiles (extra)', 1, 20, 1, 3)
            tipEase = pln.addDropDownCommandInput('tipCapEase', 'Tip cap easing', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(TIP_CAP_EASE):
                tipEase.listItems.add(name, j == 1, '')

            pln.addBoolValueInput('mirror', 'Mirror other half', True, '', True)

            # Chord
            chordDist = chd.addDropDownCommandInput('chordDist', 'Chord distribution', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(CHORD_DISTS):
                chordDist.listItems.add(name, j == 0, '')

            chd.addValueInput('rootChord', 'Root chord', 'cm', adsk.core.ValueInput.createByString('12 cm'))
            chd.addValueInput('midChord',  'Mid chord',  'cm', adsk.core.ValueInput.createByString('10 cm'))
            chd.addValueInput('tipChord',  'Tip chord',  'cm', adsk.core.ValueInput.createByString('6 cm'))

            chd.addSeparatorCommandInput('sepChordPoly')
            chd.addValueInput('polyA0', 'Poly a0', '', adsk.core.ValueInput.createByString('0'))
            chd.addValueInput('polyA1', 'Poly a1', '', adsk.core.ValueInput.createByString('1'))
            chd.addValueInput('polyA2', 'Poly a2', '', adsk.core.ValueInput.createByString('0'))
            chd.addValueInput('polyA3', 'Poly a3', '', adsk.core.ValueInput.createByString('0'))

            # Angles
            sweepDist = ang.addDropDownCommandInput('sweepDist', 'Sweep distribution', adsk.core.DropDownStyles.TextListDropDownStyle)
            sweepDist.listItems.add(SWEEP_DISTS[0], True, '')
            sweepDist.listItems.add(SWEEP_DISTS[1], False, '')

            ang.addValueInput('sweep', 'Sweep', 'deg', adsk.core.ValueInput.createByString('15 deg'))
            ang.addValueInput('sweepInner', 'Inner sweep', 'deg', adsk.core.ValueInput.createByString('15 deg'))
            ang.addValueInput('sweepOuter', 'Outer sweep', 'deg', adsk.core.ValueInput.createByString('25 deg'))
            ang.addValueInput('sweepBreak', 'Sweep break (0..1)', '', adsk.core.ValueInput.createByString('0.35'))

            sweepRef = ang.addDropDownCommandInput('sweepRef', 'Sweep reference', adsk.core.DropDownStyles.TextListDropDownStyle)
            sweepRef.listItems.add('Leading edge', False, '')
            sweepRef.listItems.add('Quarter-chord', True, '')
            sweepRef.listItems.add('Trailing edge', False, '')

            ang.addSeparatorCommandInput('sepDihedral')

            dihedralDist = ang.addDropDownCommandInput('dihedralDist', 'Dihedral distribution', adsk.core.DropDownStyles.TextListDropDownStyle)
            dihedralDist.listItems.add(DIHEDRAL_DISTS[0], True, '')
            dihedralDist.listItems.add(DIHEDRAL_DISTS[1], False, '')

            ang.addValueInput('dihedral', 'Dihedral', 'deg', adsk.core.ValueInput.createByString('3 deg'))
            ang.addValueInput('dihInner', 'Inner dihedral', 'deg', adsk.core.ValueInput.createByString('3 deg'))
            ang.addValueInput('dihOuter', 'Outer dihedral', 'deg', adsk.core.ValueInput.createByString('6 deg'))
            ang.addValueInput('dihBreak', 'Dihedral break (0..1)', '', adsk.core.ValueInput.createByString('0.35'))

            ang.addSeparatorCommandInput('sepTwist')

            ang.addValueInput('twRoot', 'Twist root', 'deg', adsk.core.ValueInput.createByString('0 deg'))
            ang.addValueInput('twMid',  'Twist mid',  'deg', adsk.core.ValueInput.createByString('-2 deg'))
            ang.addValueInput('twTip',  'Twist tip',  'deg', adsk.core.ValueInput.createByString('-4 deg'))

            twistDist = ang.addDropDownCommandInput('twistDist', 'Twist distribution', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(TWIST_DISTS):
                twistDist.listItems.add(name, j == 0, '')

            twistAxis = ang.addDropDownCommandInput('twistAxis', 'Twist about', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(TWIST_AXIS):
                twistAxis.listItems.add(name, j == 1, '')

            ang.addValueInput('twistAxisFrac', 'Elastic axis (% chord)', '', adsk.core.ValueInput.createByString('0.25'))

            # Winglets
            wlt.addBoolValueInput('wingletEnable', 'Enable winglet', True, '', False)
            wlt.addValueInput('wingletHeight', 'Winglet height', 'cm', adsk.core.ValueInput.createByString('5 cm'))
            wlt.addValueInput('wingletCant', 'Winglet cant (0=flat, 90=vertical)', 'deg', adsk.core.ValueInput.createByString('90 deg'))
            wlt.addValueInput('wingletSweep', 'Winglet sweep', 'deg', adsk.core.ValueInput.createByString('0 deg'))
            wlt.addValueInput('wingletChordRatio', 'Winglet tip chord ratio', '', adsk.core.ValueInput.createByString('0.7'))
            wlt.addValueInput('wingletThicknessMult', 'Winglet thickness multiplier', '', adsk.core.ValueInput.createByString('1'))
            wlt.addValueInput('wingletCamberMult', 'Winglet camber multiplier', '', adsk.core.ValueInput.createByString('1'))
            wlt.addValueInput('wingletTwist', 'Winglet tip twist delta', 'deg', adsk.core.ValueInput.createByString('0 deg'))
            wlt.addBoolValueInput('wingletRails', 'Use guide rails (LE/TE)', True, '', True)
            wlt.addBoolValueInput('wingletUseTip', 'Use tip airfoil (no DAT)', True, '', True)
            wlt.addStringValueInput('wingletPath', 'Winglet DAT', '')
            wlt.addBoolValueInput('browseWinglet', 'Browse winglet…', False, '', False)

            # Advanced
            adv.addBoolValueInput('flipZ', 'Flip airfoil vertically', True, '', True)
            adv.addIntegerSpinnerCommandInput('resampleN', 'Resample points per surface', 60, 250, 10, 140)
            adv.addBoolValueInput('logParams', 'Log parameters to console', True, '', True)
            adv.addSeparatorCommandInput('sepScale')
            adv.addValueInput('thicknessScale', 'Thickness scale', '', adsk.core.ValueInput.createByString('1'))
            thicknessDist = adv.addDropDownCommandInput('thicknessDist', 'Thickness distribution', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(SCALE_DISTS):
                thicknessDist.listItems.add(name, j == 0, '')
            adv.addValueInput('thicknessMidScale', 'Thickness scale (mid)', '', adsk.core.ValueInput.createByString('1'))
            adv.addValueInput('thicknessTipScale', 'Thickness scale (tip)', '', adsk.core.ValueInput.createByString('1'))
            adv.addValueInput('camberScale', 'Camber scale', '', adsk.core.ValueInput.createByString('1'))
            camberDist = adv.addDropDownCommandInput('camberDist', 'Camber distribution', adsk.core.DropDownStyles.TextListDropDownStyle)
            for j, name in enumerate(SCALE_DISTS):
                camberDist.listItems.add(name, j == 0, '')
            adv.addValueInput('camberMidScale', 'Camber scale (mid)', '', adsk.core.ValueInput.createByString('1'))
            adv.addValueInput('camberTipScale', 'Camber scale (tip)', '', adsk.core.ValueInput.createByString('1'))

            adv.addSeparatorCommandInput('sepOutputs')
            adv.addTextBoxCommandInput('outArea', 'Area', '', 1, True)
            adv.addTextBoxCommandInput('outAR', 'Aspect ratio', '', 1, True)
            adv.addTextBoxCommandInput('outTaper', 'Taper ratio', '', 1, True)
            adv.addTextBoxCommandInput('outMAC', 'MAC', '', 1, True)
            adv.addTextBoxCommandInput('outMACy', 'MAC spanwise y', '', 1, True)
            adv.addTextBoxCommandInput('outMACx', 'MAC LE x', '', 1, True)
            adv.addTextBoxCommandInput('outSweepLE', 'Sweep (LE)', '', 1, True)
            adv.addTextBoxCommandInput('outSweepQC', 'Sweep (1/4)', '', 1, True)
            adv.addTextBoxCommandInput('outSweepTE', 'Sweep (TE)', '', 1, True)

            _apply_ui_enable_states(inputs)

            onInputChanged = InputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)

            onExecute = ExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)

        except:
            adsk.core.Application.get().userInterface.messageBox(traceback.format_exc())

class InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            ui = adsk.core.Application.get().userInterface
            rootInputs = args.command.commandInputs if getattr(args, 'command', None) else args.inputs
            changed = args.input

            if changed and changed.id and changed.id.startswith('browseFoil'):
                idx = int(changed.id.replace('browseFoil', ''))
                path = _file_dialog(ui, title=f'Select airfoil DAT for station {idx+1}')
                if path:
                    _find_input(rootInputs, f'foilPath{idx}').value = path
                try:
                    changed.value = False
                except:
                    pass

            if changed and changed.id == 'browseWinglet':
                path = _file_dialog(ui, title='Select WINGLET airfoil DAT')
                if path:
                    _find_input(rootInputs, 'wingletPath').value = path
                try:
                    changed.value = False
                except:
                    pass


            _apply_ui_enable_states(rootInputs)
        except:
            pass

class ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = None
        try:
            app = adsk.core.Application.get()
            ui = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox('No active Fusion design.')
                return

            units = design.unitsManager
            inputs = args.command.commandInputs

            def get(iid):
                return _find_input(inputs, iid)

            p = WingParams()

            spanMode = get('spanMode')
            p.spanIsTotal = bool(spanMode and spanMode.selectedItem and spanMode.selectedItem.name.startswith('Total'))

            span_cm = units.evaluateExpression(get('span').expression, 'cm')
            if p.spanIsTotal:
                span_cm *= 0.5
            p.span_cm = span_cm

            p.midFrac = float(units.evaluateExpression(get('midFrac').expression, ''))
            p.midFrac = max(0.01, min(0.99, p.midFrac))
            p.numStations = int(get('numStations').value)
            try:
                p.stationSpacing = get('stationSpacing').selectedItem.name
            except:
                p.stationSpacing = STATION_SPACING[0]

            # Tip treatment
            try:
                p.tipTreatment = get('tipMode').selectedItem.name
                p.tipCapLength_cm = units.evaluateExpression(get('tipCapLength').expression, 'cm')
                p.tipEndChordRatio = float(units.evaluateExpression(get('tipEndChordRatio').expression, ''))
                p.tipCapStations = int(get('tipCapStations').value)
                p.tipCapEase = get('tipCapEase').selectedItem.name
            except:
                pass

            p.mirror = bool(get('mirror').value)

            p.rootChord_cm = units.evaluateExpression(get('rootChord').expression, 'cm')
            p.midChord_cm  = units.evaluateExpression(get('midChord').expression, 'cm')
            p.tipChord_cm  = units.evaluateExpression(get('tipChord').expression, 'cm')
            p.chordDist = get('chordDist').selectedItem.name
            p.polyA0 = float(units.evaluateExpression(get('polyA0').expression, ''))
            p.polyA1 = float(units.evaluateExpression(get('polyA1').expression, ''))
            p.polyA2 = float(units.evaluateExpression(get('polyA2').expression, ''))
            p.polyA3 = float(units.evaluateExpression(get('polyA3').expression, ''))

            p.sweepDist = get('sweepDist').selectedItem.name
            p.sweep_rad = get('sweep').value
            p.sweepInner_rad = get('sweepInner').value
            p.sweepOuter_rad = get('sweepOuter').value
            p.sweepBreakFrac = float(units.evaluateExpression(get('sweepBreak').expression, ''))
            p.sweepBreakFrac = max(0.01, min(0.99, p.sweepBreakFrac))

            sweepRef = get('sweepRef').selectedItem.name
            if sweepRef.startswith('Leading'):
                p.sweepRefFrac = 0.0
            elif sweepRef.startswith('Trailing'):
                p.sweepRefFrac = 1.0
            else:
                p.sweepRefFrac = 0.25

            p.dihedralDist = get('dihedralDist').selectedItem.name
            p.dihedral_rad = get('dihedral').value
            p.dihedralInner_rad = get('dihInner').value
            p.dihedralOuter_rad = get('dihOuter').value
            p.dihedralBreakFrac = float(units.evaluateExpression(get('dihBreak').expression, ''))
            p.dihedralBreakFrac = max(0.01, min(0.99, p.dihedralBreakFrac))

            p.twistRoot_rad = get('twRoot').value
            p.twistMid_rad  = get('twMid').value
            p.twistTip_rad  = get('twTip').value
            p.twistDist = get('twistDist').selectedItem.name

            p.twistAxisMode = get('twistAxis').selectedItem.name
            p.twistAxisFrac = float(units.evaluateExpression(get('twistAxisFrac').expression, ''))
            p.twistAxisFrac = max(0.0, min(1.0, p.twistAxisFrac))

            p.flipZ = bool(get('flipZ').value)
            p.resampleN = int(get('resampleN').value)
            p.thicknessScale = float(units.evaluateExpression(get('thicknessScale').expression, ''))
            try:
                p.thicknessDist = get('thicknessDist').selectedItem.name
                p.thicknessMidScale = float(units.evaluateExpression(get('thicknessMidScale').expression, ''))
                p.thicknessTipScale = float(units.evaluateExpression(get('thicknessTipScale').expression, ''))
            except:
                pass

            p.camberScale = float(units.evaluateExpression(get('camberScale').expression, ''))
            try:
                p.camberDist = get('camberDist').selectedItem.name
                p.camberMidScale = float(units.evaluateExpression(get('camberMidScale').expression, ''))
                p.camberTipScale = float(units.evaluateExpression(get('camberTipScale').expression, ''))
            except:
                pass
            p.thicknessScale = max(0.1, min(3.0, p.thicknessScale))
            p.camberScale = max(0.0, min(3.0, p.camberScale))

            count = int(get('airfoilCount').value)
            count = max(2, min(MAX_AIRFOILS, count))

            stations = []
            foil_meta = []
            for i in range(count):
                path = get(f'foilPath{i}').value
                if not path:
                    ui.messageBox(f'Please select DAT file for Airfoil {i+1}.')
                    return

                if i == 0:
                    t = 0.0
                elif i == count - 1:
                    t = 1.0
                else:
                    t = float(units.evaluateExpression(get(f'foilFrac{i}').expression, ''))
                    t = max(0.0, min(1.0, t))

                _, pts = load_dat_airfoil(path)
                pts = normalize_airfoil(pts)
                loop = resample_selig(pts, n=p.resampleN)
                loop = ensure_airfoil_up(loop)
                stations.append((t, loop, path))

            stations.sort(key=lambda x: x[0])
            dedup = []
            last_t = -1.0
            eps = 1e-4
            for t, loop, path in stations:
                if last_t is None or abs(t - last_t) > 1e-6:
                    dedup.append((t, loop, path))
                    last_t = t

            p.foilStations = [{'t': t, 'loop': loop, 'path': path} for (t, loop, path) in dedup]
            foil_meta = [{'t': st.get('t', 0.0), 'path': st.get('path', ''), 'name': os.path.basename(st.get('path', '') or '')} for st in p.foilStations]


            # Winglet
            try:
                p.wingletEnable = inputs.itemById('wingletEnable').value
            except:
                p.wingletEnable = False

            if p.wingletEnable:
                p.wingletHeight_cm = units.evaluateExpression(inputs.itemById('wingletHeight').expression, 'cm')
                p.wingletCant_rad = inputs.itemById('wingletCant').value
                p.wingletSweep_rad = inputs.itemById('wingletSweep').value
                p.wingletTipChordRatio = float(units.evaluateExpression(inputs.itemById('wingletChordRatio').expression, ''))
                p.wingletTwistTipDelta_rad = inputs.itemById('wingletTwist').value
                p.wingletUseRails = inputs.itemById('wingletRails').value
                p.wingletUseTipAirfoil = inputs.itemById('wingletUseTip').value
                p.wingletPath = inputs.itemById('wingletPath').value if inputs.itemById('wingletPath') else ''
                if (not p.wingletUseTipAirfoil) and (not p.wingletPath):
                    ui.messageBox('Winglet is enabled but no winglet DAT file is selected.')
                    return

            # Winglet vs Tip Treatment compatibility:
            # Tip treatments add extra tapered "cap" profiles beyond the tip. The winglet code expects the
            # last main-wing profile to be the true tip, so combining them can produce degenerate loft inputs.
            # For now, automatically disable tip treatments when winglet is enabled.
            try:
                if getattr(p, 'wingletEnable', False):
                    if getattr(p, 'tipTreatment', TIP_TREATMENTS[0]) != TIP_TREATMENTS[0]:
                        ui.messageBox('Note: Tip treatment is disabled when a winglet is enabled (to prevent loft failures).')
                        p.tipTreatment = TIP_TREATMENTS[0]
                        p.tipCapLength_cm = 0.0
            except:
                pass

            if get('logParams') and get('logParams').value:
                _print_run_params(ui, units, p, foil_meta)
            build_wing(design, ui, p)
            # Print planform outputs to the Text Commands palette after successful build.
            try:
                out = _compute_outputs_from_params(p)
                _print_planform_outputs(ui, design.unitsManager if design else None, out)
            except:
                pass


        except:
            if ui:
                ui.messageBox('Failed:\n' + traceback.format_exc())

# -----------------------------
# Add-in lifecycle
# -----------------------------

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        cmdDef = ui.commandDefinitions.itemById(CMD_ID)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC)

        onCreated = CommandCreatedHandler()
        cmdDef.commandCreated.add(onCreated)
        _handlers.append(onCreated)

        ws = ui.workspaces.itemById(WORKSPACE_ID)
        panel = ws.toolbarPanels.itemById(PANEL_ID)

        ctrl = panel.controls.itemById(CMD_ID)
        if not ctrl:
            panel.controls.addCommand(cmdDef)

    except:
        if ui:
            ui.messageBox('Add-in start failed:\n' + traceback.format_exc())

def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        ws = ui.workspaces.itemById(WORKSPACE_ID)
        panel = ws.toolbarPanels.itemById(PANEL_ID)

        ctrl = panel.controls.itemById(CMD_ID)
        if ctrl:
            ctrl.deleteMe()

        cmdDef = ui.commandDefinitions.itemById(CMD_ID)
        if cmdDef:
            cmdDef.deleteMe()

    except:
        if ui:
            ui.messageBox('Add-in stop failed:\n' + traceback.format_exc())