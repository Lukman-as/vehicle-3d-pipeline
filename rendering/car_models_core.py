"""
Core (OpenGL-free) logic for per-vehicle-type car modelling.

Pipeline per detected car:
  1. conditional_params(type, OW, WB, OH, OL)  -> 10 shape params from that
     type's Gaussian model  (Stage 1: per-type proportions)
  2. build_geometry(params, type)              -> a type-specific mesh
     (Stage 2: sedan=3-box/trunk, suv & van=2-box, truck=cab+bed)
  3. fit_geometry(geom, points, heading)       -> rigid-align the mesh to the
     point cloud and return the final residual (mean squared point->surface dist)
  4. select_best_fit(...)                       -> do 1-3 for all four types and
     pick the one with the smallest residual.

This module is pure numpy/torch so it can be unit-tested without a display.
The renderer (optimize_render_car_multimodel.py) imports from here.
"""

import os
import numpy as np
import torch   # type: ignore

# Order of the 10 geometry params used everywhere downstream.
PARAM_NAMES = ['A', 'C', 'D', 'E', 'F', 'G', 'OH', 'OL', 'OW', 'TW']
VEHICLE_TYPES = ['sedan', 'suv', 'truck', 'van']

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(os.path.dirname(_THIS_DIR), 'priors', 'models')


def model_path(vehicle_type):
    return os.path.join(MODEL_DIR, f'gaussian_model_{vehicle_type}_OW_WB_OH_OL.npz')


# --------------------------------------------------------------------------
# Per-type SHAPE constants.  All longitudinal fractions are of L = OL - A
# (the length budget behind the windshield base).  These are the knobs you
# tune visually after a first run -- changing them never breaks the pipeline.
#   ws_top     : windshield-top position (smaller = steeper windshield)
#   roof_back  : where the roof ends (suv/van two-box)
#   cabin_back : where the sedan roof ends before the rear window
#   trunk_base : sedan rear-window base / start of the trunk deck
#   trunk_h    : sedan trunk-deck height as a fraction of (roof - belt) above belt
#   cab_back   : truck cab rear (short cab => long bed)
#   bed_h      : truck bed-rail height as a fraction of (roof - belt) above belt
#   tire_r     : fixed tire radius in cm (the "ground-clearance" decision)
# --------------------------------------------------------------------------
#   hood_drop  : how far the hood nose drops below the beltline, as a fraction
#                of the body-box height d (gives a raked front instead of a box)
SHAPE = {
    'sedan': dict(ws_top=0.20, cabin_back=0.55, trunk_base=0.72, trunk_h=0.30, tire_r=43.18, hood_drop=0.50),
    'suv':   dict(ws_top=0.18, roof_back=0.92,                                 tire_r=46.0,  hood_drop=0.38),
    'van':   dict(ws_top=0.10, roof_back=0.97,                                 tire_r=46.0,  hood_drop=0.18),
    'truck': dict(ws_top=0.16, cab_back=0.40, bed_h=0.0,                       tire_r=55.0,  hood_drop=0.45),
}


# ===========================  Gaussian model  =============================
def load_gaussian_model(filename):
    data = np.load(filename, allow_pickle=True)
    return (data['mu_a'], data['cov_ab'], data['cov_bb_inv'], data['mu_b'],
            data['a_part'].tolist(), data['b_part'].tolist())


def as_scalar(value):
    return float(np.asarray(value).reshape(-1)[0])


def conditional_params(vehicle_type, ow, wb, oh, ol):
    """Condition the type's Gaussian on the 4 measured exterior dims and return
    a 10-vector ordered by PARAM_NAMES (cm)."""
    mu_a, cov_ab, cov_bb_inv, mu_b, _a_part, b_part = load_gaussian_model(model_path(vehicle_type))
    cv = {'OW': float(ow), 'WB': float(wb), 'OH': float(oh), 'OL': float(ol)}
    b_vector = np.array([cv[name] for name in b_part], dtype=float)
    pred = mu_a + cov_ab @ cov_bb_inv @ (b_vector - mu_b)   # -> A, C, D, E, FH, RH, TW

    out = np.zeros(len(PARAM_NAMES))
    out[0:4] = pred[0:4]          # A, C, D, E
    out[4] = pred[4]             # F  (= FH)
    out[5] = pred[5]             # G  (= RH)
    out[6] = cv['OH']            # OH (passed through)
    out[7] = cv['OL']            # OL (passed through)
    out[8] = cv['OW']            # OW (passed through)
    out[9] = pred[6]             # TW
    return out


# ===========================  Geometry  ==================================
def _compute_normal(verts):
    edge1 = verts[1] - verts[0]
    edge2 = verts[3] - verts[0]
    normal = np.cross(edge1, edge2)
    n = np.linalg.norm(normal)
    return normal / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])


def _silhouette(vehicle_type, params):
    """Return the top side-profile polyline as a list of (z, y, off) from the
    front (z=0) to the rear (z=-OL).  `off` is the per-point x-inset: 0 for
    full-body-width points, `ins` for the (narrower) greenhouse points.
    Also returns y_sill (body floor) and rear_y (height of the rear top edge).
    """
    a, c, d, e, f, g, oh, ol, ow, tw = params
    y_sill = oh - c - d          # bottom of the body box
    y_belt = oh - c              # beltline / bottom of side glass
    y_roof = oh                  # roof top
    L = ol - a
    ins = float(np.clip((ow - e) / 2.0, 0.0, 0.45 * ow))   # greenhouse inset
    s = SHAPE[vehicle_type]

    prof = [(0.0, y_belt, 0.0),     # grille top / hood front
            (-a,  y_belt, 0.0)]     # windshield base (end of hood)

    if vehicle_type in ('suv', 'van'):
        z_ws = -(a + s['ws_top'] * L)
        z_rb = -(a + s['roof_back'] * L)
        prof += [(z_ws, y_roof, ins),     # windshield top
                 (z_rb, y_roof, ins),     # roof back
                 (-ol,  y_belt, 0.0)]     # rear window down to beltline at rear
        rear_y = y_belt

    elif vehicle_type == 'sedan':
        z_ws = -(a + s['ws_top'] * L)
        z_cb = -(a + s['cabin_back'] * L)
        z_tb = -(a + s['trunk_base'] * L)
        y_tr = y_belt + s['trunk_h'] * (y_roof - y_belt)   # trunk-lid height
        prof += [(z_ws, y_roof, ins),     # windshield top
                 (z_cb, y_roof, ins),     # cabin / roof back
                 (z_tb, y_tr,   0.0),     # rear-window base = trunk start
                 (-ol,  y_tr,   0.0)]     # trunk-deck rear
        rear_y = y_tr

    elif vehicle_type == 'truck':
        z_ws = -(a + s['ws_top'] * L)
        z_cb = -(a + s['cab_back'] * L)
        y_bed = y_belt + s['bed_h'] * (y_roof - y_belt)    # bed-rail height
        prof += [(z_ws, y_roof, ins),     # windshield top
                 (z_cb, y_roof, ins),     # cab roof back
                 (z_cb, y_bed,  0.0),     # back-of-cab vertical wall
                 (-ol,  y_bed,  0.0)]     # bed rear
        rear_y = y_bed
    else:
        raise ValueError(f'unknown vehicle_type {vehicle_type!r}')

    return prof, y_sill, rear_y


def build_geometry(params, vehicle_type):
    """Build a watertight-ish quad+cylinder mesh for one vehicle type.
    Returns {'quads', 'cylinders', 'center_car', 'vehicle_type'} in the same
    centred frame the original renderer used (x in [-OW/2, OW/2], z in
    [-OL/2, OL/2], y up from 0)."""
    a, c, d, e, f, g, oh, ol, ow, tw = params
    prof, y_sill, rear_y = _silhouette(vehicle_type, params)

    quads = []

    def add_quad(p0, p1, p2, p3):
        verts = [np.array(p, dtype=float) for p in (p0, p1, p2, p3)]
        quads.append({'vertices': verts, 'normal': _compute_normal(verts)})

    s = SHAPE[vehicle_type]
    y_belt = oh - c                              # beltline: top of the full-width lower body
    hood_drop = s.get('hood_drop', 0.4) * d
    y_nose = max(y_sill, y_belt - hood_drop)     # raked hood front edge

    # ---- Front clip: raked hood + dropped nose (z in [0, -a]) ----
    add_quad((0, y_sill, 0),  (ow, y_sill, 0),  (ow, y_nose, 0),  (0, y_nose, 0))     # front fascia
    add_quad((0, y_nose, 0),  (ow, y_nose, 0),  (ow, y_belt, -a), (0, y_belt, -a))    # hood (raked down to nose)
    add_quad((0, y_sill, 0),  (0, y_nose, 0),   (0, y_belt, -a),  (0, y_sill, -a))    # hood left side
    add_quad((ow, y_sill, 0), (ow, y_nose, 0),  (ow, y_belt, -a), (ow, y_sill, -a))   # hood right side

    # ---- Lower body / doors: FULL width, beltline down to sill (z in [-a, -ol]) ----
    # Fix for the exposed-wheel "missing door": the greenhouse inset is applied
    # only ABOVE the belt; the doors always reach the full body width over the wheels.
    add_quad((0, y_sill, -a),  (0, y_belt, -a),  (0, y_belt, -ol),  (0, y_sill, -ol))    # left door/body
    add_quad((ow, y_sill, -a), (ow, y_belt, -a), (ow, y_belt, -ol), (ow, y_sill, -ol))   # right door/body

    # ---- Greenhouse + decks: profile above the belt; inset lives only here ----
    upper = prof[1:]                             # from windshield base (-a, y_belt) rearwards
    for (z0, y0, o0), (z1, y1, o1) in zip(upper[:-1], upper[1:]):
        add_quad((o0, y0, z0), (ow - o0, y0, z0), (ow - o1, y1, z1), (o1, y1, z1))         # top / glass
        add_quad((o0, y_belt, z0), (o0, y0, z0), (o1, y1, z1), (o1, y_belt, z1))           # left glass/pillar
        add_quad((ow - o0, y_belt, z0), (ow - o0, y0, z0),
                 (ow - o1, y1, z1), (ow - o1, y_belt, z1))                                  # right glass/pillar
        if o0 > 1e-6 or o1 > 1e-6:               # shoulder ledge: body edge -> inset greenhouse
            add_quad((0, y_belt, z0), (o0, y_belt, z0), (o1, y_belt, z1), (0, y_belt, z1))
            add_quad((ow, y_belt, z0), (ow - o0, y_belt, z0),
                     (ow - o1, y_belt, z1), (ow, y_belt, z1))

    # ---- caps (front is already faced by the front clip) ----
    add_quad((0, y_sill, -ol), (ow, y_sill, -ol), (ow, rear_y, -ol), (0, rear_y, -ol))     # rear face
    add_quad((0, y_sill, 0),   (ow, y_sill, 0),   (ow, y_sill, -ol), (0, y_sill, -ol))      # underbody

    # ---- tires (4 cylinders, axes along x) ----
    s = SHAPE[vehicle_type]
    tire_r = s['tire_r']
    tire_w = ow - tw
    offset = 5.0
    z_front_axle = -f
    z_rear_axle = -(ol - g)
    v19 = np.array([ow - (ow - tw) / 2 + offset, tire_r, z_front_axle])   # right front
    v20 = np.array([(ow - tw) / 2 - offset,      tire_r, z_front_axle])   # left  front
    v21 = np.array([ow - (ow - tw) / 2 + offset, tire_r, z_rear_axle])    # right rear
    v22 = np.array([(ow - tw) / 2 - offset,      tire_r, z_rear_axle])    # left  rear
    axis_right = np.array([1.0, 0.0, 0.0])
    axis_left = np.array([-1.0, 0.0, 0.0])
    cylinders = [
        {'center': v19, 'radius': tire_r, 'height': tire_w, 'axis': axis_right},
        {'center': v20, 'radius': tire_r, 'height': tire_w, 'axis': axis_left},
        {'center': v21, 'radius': tire_r, 'height': tire_w, 'axis': axis_right},
        {'center': v22, 'radius': tire_r, 'height': tire_w, 'axis': axis_left},
    ]

    # ---- centre everything (x by OW/2, z by +OL/2) ----
    x_shift, z_shift = ow / 2.0, ol / 2.0
    for q in quads:
        for v in q['vertices']:
            v[0] -= x_shift
            v[2] += z_shift
        q['normal'] = _compute_normal(q['vertices'])
    for cyl in cylinders:
        cyl['center'][0] -= x_shift
        cyl['center'][2] += z_shift

    all_pts = np.vstack([v for q in quads for v in q['vertices']]
                        + [c['center'] for c in cylinders])
    center_car = np.mean(all_pts, axis=0)
    return {'quads': quads, 'cylinders': cylinders,
            'center_car': center_car, 'vehicle_type': vehicle_type}


# ===========================  Distance fns  ==============================
def ud_quad(p, a, b, c, d, eps=1e-5):
    def dot(u, v): return torch.sum(u * v, dim=-1)
    def dot2(v): return torch.sum(v * v, dim=-1)
    def cross(u, v): return torch.cross(u, v, dim=-1)
    ba = b - a; pa = p - a
    cb = c - b; pb = p - b
    dc = d - c; pc = p - c
    ad = a - d; pd = p - d
    nor = cross(ba, ad)
    s = (torch.sign(dot(cross(ba, nor), pa)) +
         torch.sign(dot(cross(cb, nor), pb)) +
         torch.sign(dot(cross(dc, nor), pc)) +
         torch.sign(dot(cross(ad, nor), pd)))

    def seg_dist2(pa_, ba_):
        denom = dot2(ba_) + eps
        t = torch.clamp(dot(ba_, pa_) / denom, 0.0, 1.0)
        v = ba_ * t[..., None] - pa_
        return dot2(v)

    e0 = seg_dist2(pa, ba); e1 = seg_dist2(pb, cb)
    e2 = seg_dist2(pc, dc); e3 = seg_dist2(pd, ad)
    min_edge = torch.min(torch.stack([e0, e1, e2, e3], axis=-1), axis=-1)[0]
    numer = dot(nor, pa)
    plane_d2 = (numer * numer) / (dot2(nor) + eps)
    d2 = torch.where(s < 3.0, min_edge, plane_d2)
    return torch.sqrt(d2.abs() + eps)


def distance_to_cylinder(point, center, radius, height, axis):
    bottom_center = center.clone()
    bottom_center[1] = 0
    bottom_center = bottom_center + (height / 2) * axis
    radial_vec = point - bottom_center
    return torch.abs(torch.norm(radial_vec))


# ===========================  Fit + selection  ===========================
# Tire term default weight.  The four templates share almost identical tires
# (only tire_r differs slightly), so tires carry little type information and the
# huge contact-point distances can swamp the body signal -- hence a low default.
DEFAULT_TIRE_WEIGHT = 1.0

# Fix A -- robust residual: drop this fraction of the worst-fitting body points
# before averaging, so a few outliers (e.g. a pickup's tall cargo rack, which
# the low-bed truck template cannot reach) don't dominate the type decision.
TRIM_FRAC = 0.20

# Fix B -- exterior-dimension prior: weight of the Gaussian NLL of (OW,WB,OH,OL)
# under each type, added to the geometric residual.  >0 lets "vans are tall /
# pickups are long" break the close geometric calls.  Calibrated on the 3 sample
# cars: any value in ~[12, 50] classifies all three correctly; 16 sits inside
# that band with margin.  NOTE: this is tuned on only 3 vehicles -- recalibrate
# against the CATEGORY-labelled data for a statistically sound value.
PRIOR_WEIGHT = 16.0


def exterior_nll(vehicle_type, ow, wb, oh, ol):
    """Negative log-likelihood of the exterior dims under the type's Gaussian
    b-marginal N(mu_b, cov_bb).  Lower = these dims are more typical of the type.
    The constant 0.5*k*log(2*pi) is dropped (identical across types)."""
    _mu_a, _cov_ab, cov_bb_inv, mu_b, _a, b_part = load_gaussian_model(model_path(vehicle_type))
    cv = {'OW': float(ow), 'WB': float(wb), 'OH': float(oh), 'OL': float(ol)}
    d = np.array([cv[n] for n in b_part], dtype=float) - mu_b
    maha2 = float(d @ cov_bb_inv @ d)
    logdet_cov_bb = -float(np.linalg.slogdet(cov_bb_inv)[1])   # log|cov_bb| = -log|cov_bb_inv|
    return 0.5 * maha2 + 0.5 * logdet_cov_bb


def _trimmed_residual(body_dists, trim_frac):
    """Mean of squared body-point distances after discarding the worst
    `trim_frac` (always keeps at least one point)."""
    d = np.asarray(body_dists, dtype=float)
    if d.size == 0:
        return float('inf')
    d2 = np.sort(d ** 2)
    keep = max(1, int(round(d2.size * (1.0 - trim_frac))))
    return float(np.mean(d2[:keep]))


def _eval_loss(quad_verts, cyls, pts, translation, angle, tire_weight):
    """Return (total_loss, body_mean, body_dists) for one pose.  total includes
    the down-weighted tire term; body_dists is the per-body-point distance tensor
    (used for the trimmed selection residual)."""
    cos = torch.cos(torch.deg2rad(angle)); sin = torch.sin(torch.deg2rad(angle))
    z = torch.zeros_like(cos); o = torch.ones_like(cos)
    rot = torch.stack([torch.stack([cos, z, sin]),
                       torch.stack([z, o, z]),
                       torch.stack([-sin, z, cos])])
    all_d, body_d = [], []
    for p in pts:
        p_t = rot.T @ (p - translation)
        if p[1] > 0.5:                          # body point -> nearest quad
            md = torch.tensor(float('inf'))
            for vs in quad_verts:
                md = torch.min(md, ud_quad(p_t, *vs))
            body_d.append(md); all_d.append(md)
        else:                                   # ground point -> nearest tire
            md = torch.tensor(float('inf'))
            for (ce, ax, r, h) in cyls:
                md = torch.min(md, distance_to_cylinder(p_t, ce, r, h, ax) * tire_weight)
            all_d.append(md)
    total = torch.mean(torch.abs(torch.stack(all_d)) ** 2)
    body_stack = torch.stack(body_d) if body_d else torch.stack(all_d)
    body_mean = torch.mean(torch.abs(body_stack) ** 2)
    return total, body_mean, body_stack


def fit_geometry(geom, points, init_angle, iters=50, lr=0.8,
                 tire_weight=DEFAULT_TIRE_WEIGHT):
    """Rigidly align `geom` (translation + heading only) to `points`.
    Returns (total_residual, body_mean, body_dists[np], translation[3], angle_deg).
    Shape is NOT optimised; body_dists feeds the trimmed selection residual."""
    points = np.asarray(points, dtype=float)
    translation = torch.tensor(points.mean(axis=0) - geom['center_car'],
                               requires_grad=True, dtype=torch.float)
    angle = torch.tensor(float(init_angle), requires_grad=True, dtype=torch.float)
    optimizer = torch.optim.RMSprop([translation, angle], lr=lr, momentum=0.5)
    pts = torch.from_numpy(points).float()

    quad_verts = [[torch.tensor(v, dtype=torch.float) for v in q['vertices']]
                  for q in geom['quads']]
    cyls = [(torch.tensor(c['center'], dtype=torch.float),
             torch.tensor(c['axis'], dtype=torch.float),
             c['radius'], c['height']) for c in geom['cylinders']]

    for _ in range(iters):
        optimizer.zero_grad()
        total, _, _ = _eval_loss(quad_verts, cyls, pts, translation, angle, tire_weight)
        total.backward()
        optimizer.step()

    with torch.no_grad():
        total_f, body_mean, body_dists = _eval_loss(quad_verts, cyls, pts, translation, angle, tire_weight)
    return (total_f.item(), body_mean.item(), body_dists.numpy(),
            translation.detach().numpy(), float(angle.detach()))


def select_best_fit(points, ow, wb, oh, ol, init_angle,
                    types=VEHICLE_TYPES, iters=50, heading_starts=(0.0, 180.0),
                    tire_weight=DEFAULT_TIRE_WEIGHT, trim_frac=TRIM_FRAC,
                    prior_weight=PRIOR_WEIGHT):
    """Fit all four type models and pick the smallest combined score
        score = trimmed_body_residual + prior_weight * exterior_nll
    Each type is fitted from every heading offset (the 180-deg flip removes the
    front/back ambiguity) and the best geometric start is kept.
    Returns (best_type, results); each results[type] holds residual (trimmed) /
    nll / score / body_mean / total_residual / params / geom / translation /
    angle."""
    results = {}
    for vt in types:
        params = conditional_params(vt, ow, wb, oh, ol)
        geom = build_geometry(params, vt)
        best_c = None
        for off in heading_starts:
            total, body_mean, body_dists, translation, angle = fit_geometry(
                geom, points, init_angle + off, iters=iters, tire_weight=tire_weight)
            trimmed = _trimmed_residual(body_dists, trim_frac)
            if best_c is None or trimmed < best_c['residual']:
                best_c = dict(residual=trimmed, body_mean=body_mean,
                              total_residual=total, translation=translation, angle=angle)
        nll = exterior_nll(vt, ow, wb, oh, ol)
        best_c['nll'] = nll
        best_c['score'] = best_c['residual'] + prior_weight * nll
        best_c.update(params=params, geom=geom)
        results[vt] = best_c
    best = min(results, key=lambda k: results[k]['score'])
    return best, results
