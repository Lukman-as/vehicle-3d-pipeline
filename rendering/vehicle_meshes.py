"""
Real low-poly vehicle meshes (Kenney Car Kit, CC0) for the multi-model renderer.

The fitting / type-selection pipeline in car_models_core.py still uses the fast
procedural box mesh.  THIS module is display-only: once a type is chosen and a
pose is fitted, we load the matching real model, stretch it to the predicted
OL/OW/OH bounding box, and hand back a triangle soup that the OpenGL widget
draws in place of the box.

Kenney Car Kit convention (verified against the .glb files):
  * Y up, wheels resting on y = 0 (after baking the scene-graph transforms)
  * length along +Z, width along X, NOSE points +Z
  * this matches our box frame exactly (grille at +OL/2), so no axis flip needed
Colours come from the kit's palette texture, baked to per-vertex RGB via
trimesh's `to_color()` (gives painted body + dark wheels/glass for free).

Models live in   ./models/<type>.glb   (sedan, suv, van, truck).
Download: https://kenney.nl/assets/car-kit  ->  GLB format  ->  copy those four.
"""

import os
import numpy as np
import trimesh   # type: ignore

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(_THIS_DIR, 'models')
# Kenney kit lives here separately so it isn't clobbered by photoreal swaps in
# models/.  reconstruct() (parametric) reads from here by default.
KENNEY_DIR = os.path.join(_THIS_DIR, 'models', 'kenney')

# Map the four selectable types to a Kenney .glb.  (Swap in sedan-sports,
# suv-luxury, delivery, etc. here if you prefer a different silhouette.)
MODEL_FILES = {
    'sedan': 'sedan.glb',
    'suv':   'suv.glb',
    'van':   'van.glb',
    'truck': 'truck.glb',
}

# Brightness lift for the baked vertex colours.  >1 lifts dark PBR bakes (e.g. a
# charcoal car body) while barely touching already-bright parts; 1.0 = off.
COLOR_GAMMA = 1.5

_BASE_CACHE = {}   # type -> baked Trimesh in Kenney frame (loaded once)


_FALLBACK_RGB = np.array([160, 160, 165], dtype=np.uint8)   # neutral grey

# Fallback tire radii (cm) if car_models_core.SHAPE can't be imported.
_DEFAULT_TIRE_R = {'sedan': 43.18, 'suv': 46.0, 'van': 46.0, 'truck': 55.0}


def _tire_radius(vehicle_type):
    """Per-type tire radius (cm) from car_models_core.SHAPE, with a static
    fallback so this module never hard-depends on the (torch-importing) core."""
    try:
        import car_models_core as core
        return float(core.SHAPE[vehicle_type]['tire_r'])
    except Exception:
        return _DEFAULT_TIRE_R.get(vehicle_type, 45.0)


def _part_colors(geom):
    """Per-vertex RGB (uint8, len == len(geom.vertices)) for one part, baking its
    texture/material if possible, else a neutral grey.  Guarantees alignment so
    multi-material models don't desync colours from vertices."""
    n = len(geom.vertices)
    try:
        vis = geom.visual.to_color() if hasattr(geom.visual, 'to_color') else geom.visual
        vc = getattr(vis, 'vertex_colors', None)
        if vc is not None and len(vc) == n:
            return np.asarray(vc)[:, :3].astype(np.uint8)
    except Exception:
        pass
    return np.tile(_FALLBACK_RGB, (n, 1))


def _bake_scene(path):
    """Load a .glb and return ONE Trimesh in world coords with per-vertex colours.

    We stack vertices/faces/colours ourselves (instead of trimesh.concatenate) so
    the colour array ALWAYS matches the vertex count.  This is what makes the
    loader survive arbitrary models: the Kenney kit shares one palette texture,
    but a photoreal model can have dozens of separate materials, which trimesh's
    concatenate cannot merge into a single coherent vertex-colour array."""
    scene = trimesh.load(path)
    if not isinstance(scene, trimesh.Scene):
        g = scene
        return trimesh.Trimesh(vertices=g.vertices, faces=g.faces,
                               vertex_colors=_part_colors(g), process=False)

    V, F, C, offset = [], [], [], 0
    for node in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph[node]
        g = scene.geometry[geom_name].copy()
        g.apply_transform(transform)            # place wheels / body in world
        V.append(np.asarray(g.vertices, dtype=float))
        F.append(np.asarray(g.faces) + offset)
        C.append(_part_colors(g))
        offset += len(g.vertices)

    return trimesh.Trimesh(vertices=np.vstack(V), faces=np.vstack(F),
                           vertex_colors=np.vstack(C), process=False)


def _load_base(vehicle_type):
    if vehicle_type not in MODEL_FILES:
        raise KeyError(f'no Kenney model mapped for type {vehicle_type!r}')
    if vehicle_type not in _BASE_CACHE:
        path = os.path.join(MODEL_DIR, MODEL_FILES[vehicle_type])
        if not os.path.exists(path):
            raise FileNotFoundError(
                f'{path} not found.  Download the Kenney Car Kit (CC0) from '
                f'https://kenney.nl/assets/car-kit and copy the GLB into models/.')
        _BASE_CACHE[vehicle_type] = _bake_scene(path)
    return _BASE_CACHE[vehicle_type]


def render_arrays(vehicle_type, ow, oh, ol):
    """Stretch the real model to the predicted box (OW x OH x OL, cm, in the
    centred frame x in [-OW/2,OW/2], y in [0,OH], z in [-OL/2,OL/2]) and return a
    flat triangle soup ready for glBegin(GL_TRIANGLES):

        positions (3F,3) float, normals (3F,3) float, colors (3F,3) float in 0..1

    where F = number of triangles, every consecutive 3 rows are one triangle."""
    base = _load_base(vehicle_type)
    v = base.vertices.astype(float).copy()
    mn = base.bounds[0]
    size = np.maximum(base.bounds[1] - base.bounds[0], 1e-6)

    # Non-uniform scale: map the model's AABB exactly onto the predicted box.
    v[:, 0] = (v[:, 0] - mn[0]) / size[0] * ow - ow / 2.0   # width, centred
    v[:, 1] = (v[:, 1] - mn[1]) / size[1] * oh              # height, ground at 0
    v[:, 2] = (v[:, 2] - mn[2]) / size[2] * ol - ol / 2.0   # length, centred (nose +Z)

    # Recompute normals on the scaled mesh (non-uniform scale changes them).
    scaled = trimesh.Trimesh(vertices=v, faces=base.faces,
                             vertex_colors=base.visual.vertex_colors, process=False)
    vn = scaled.vertex_normals
    rgb = (base.visual.vertex_colors[:, :3].astype(float)) / 255.0
    if COLOR_GAMMA != 1.0:
        rgb = np.clip(rgb, 0.0, 1.0) ** (1.0 / COLOR_GAMMA)   # lift dark PBR bakes

    f = base.faces.reshape(-1)
    return v[f], vn[f], rgb[f]


# ===========================  Parametric reconstruction  =================
def _load_parts(path):
    """Load a .glb into a list of separate parts (so wheels can be moved on their
    own).  Each part: {name, vertices(world,Nx3), faces, colors(Nx3 uint8),
    is_wheel}.  Kenney names parts 'body', 'wheel-front-left', etc."""
    scene = trimesh.load(path)
    if not isinstance(scene, trimesh.Scene):
        g = scene
        return [{'name': 'mesh', 'vertices': np.asarray(g.vertices, float),
                 'faces': np.asarray(g.faces), 'colors': _part_colors(g),
                 'is_wheel': False}]
    parts = []
    for node in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph[node]
        g = scene.geometry[geom_name].copy()
        g.apply_transform(transform)            # bake node transform -> world
        name = f'{node} {geom_name}'.lower()
        parts.append({'name': name, 'vertices': np.asarray(g.vertices, float),
                      'faces': np.asarray(g.faces), 'colors': _part_colors(g),
                      'is_wheel': ('wheel' in name)})
    return parts


def reconstruct(vehicle_type, params, model_dir=None, resize_wheels=True):
    """Parametric Kenney reconstruction.

    1. scale the whole body to the predicted OW x OH x OL box, then
    2. relocate each NAMED wheel group to the derived wheelbase (front axle at
       OL/2 - F, rear axle at -OL/2 + G), snap it to the track half-width TW/2,
       and size it to tire_r.

    So the rendered mesh now flexes with OW, OH, OL, F, G, TW (+tire_r) -- 6 of the
    predicted params, vs only 3 for the rigid render_arrays().  The body-shape
    params (A, C, D, E) need non-rigid deformation and remain the box's job.

    Returns the same (positions, normals, colors) triangle soup as render_arrays.
    Models without named wheel parts gracefully degrade to the OW/OH/OL stretch."""
    A, C, D, E, F, G, OH, OL, OW, TW = (float(x) for x in params)
    model_dir = model_dir or KENNEY_DIR
    if vehicle_type not in MODEL_FILES:
        raise KeyError(f'no model mapped for type {vehicle_type!r}')
    path = os.path.join(model_dir, MODEL_FILES[vehicle_type])
    if not os.path.exists(path):
        raise FileNotFoundError(f'{path} not found (need the Kenney kit in {model_dir}).')

    parts = _load_parts(path)

    # ---- 1. global scale: whole-vehicle AABB -> predicted OW x OH x OL ----
    allv = np.vstack([p['vertices'] for p in parts])
    mn, mx = allv.min(0), allv.max(0)
    size = np.maximum(mx - mn, 1e-6)
    cx, cz = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[2] + mx[2])
    sx, sy, sz = OW / size[0], OH / size[1], OL / size[2]
    for p in parts:
        v = p['vertices']
        p['vertices'] = np.column_stack([(v[:, 0] - cx) * sx,      # width, centred
                                         (v[:, 1] - mn[1]) * sy,    # height, ground=0
                                         (v[:, 2] - cz) * sz])      # length, centred

    # ---- 2. wheels -> derived wheelbase (F,G), track (TW), radius (tire_r) ----
    front_z = OL / 2.0 - F            # front axle z (nose at +Z)
    rear_z = -OL / 2.0 + G            # rear axle z
    track_half = TW / 2.0
    tire_r = _tire_radius(vehicle_type)
    for p in parts:
        if not p['is_wheel']:
            continue
        v = p['vertices']
        c = v.mean(0)
        if 'front' in p['name']:
            tz = front_z
        elif 'back' in p['name'] or 'rear' in p['name']:
            tz = rear_z
        else:
            tz = front_z if c[2] >= 0 else rear_z          # classify by position
        tx = c[0] if abs(c[0]) < 0.05 * OW else np.sign(c[0]) * track_half
        v = v + np.array([tx - c[0], 0.0, tz - c[2]])       # rigid move in x,z
        if resize_wheels and tire_r:                        # match radius (clamped)
            cur_r = 0.5 * (v[:, 1].max() - v[:, 1].min())
            if cur_r > 1e-3:
                k = float(np.clip(tire_r / cur_r, 0.7, 1.4))
                zc = v[:, 2].mean()
                v[:, 1] = v[:, 1] * k                        # scale about ground
                v[:, 2] = (v[:, 2] - zc) * k + zc            # scale about wheel z
        p['vertices'] = v

    # ---- 3. assemble -> triangle soup ----
    V, faces_list, C_, off = [], [], [], 0
    for p in parts:
        V.append(p['vertices'])
        faces_list.append(p['faces'] + off)
        C_.append(p['colors'])
        off += len(p['vertices'])
    verts, faces, cols = np.vstack(V), np.vstack(faces_list), np.vstack(C_)

    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    vn = mesh.vertex_normals
    rgb = cols[:, :3].astype(float) / 255.0
    if COLOR_GAMMA != 1.0:
        rgb = np.clip(rgb, 0.0, 1.0) ** (1.0 / COLOR_GAMMA)
    f = faces.reshape(-1)
    return verts[f], vn[f], rgb[f]


if __name__ == '__main__':
    # Sanity: parametrically reconstruct each Kenney type and confirm the wheels
    # actually land at the derived wheelbase / track.  Run: python vehicle_meshes.py
    #         A    C    D    E     F     G    OH   OL   OW    TW
    demo = [110, 22, 70, 30,  95, 100, 175, 480, 190, 165]
    A, C, D, E, F, G, OH, OL, OW, TW = demo
    want_front_z, want_rear_z, want_track = OL/2 - F, -OL/2 + G, TW/2
    print(f'targets: front_z={want_front_z:.0f}  rear_z={want_rear_z:.0f}  track/2={want_track:.0f}')
    for vt in MODEL_FILES:
        try:
            parts = _load_parts(os.path.join(KENNEY_DIR, MODEL_FILES[vt]))
            n_wheels = sum(p['is_wheel'] for p in parts)
            pos, nrm, col = reconstruct(vt, demo)
            print(f'{vt:6}: tris={len(pos)//3:5d}  parts={len(parts)} '
                  f'wheels={n_wheels}  finite={np.isfinite(pos).all()}  '
                  f'colors[{col.min():.2f},{col.max():.2f}]')
        except Exception as e:
            print(f'{vt:6}: ERROR {type(e).__name__}: {e}')
