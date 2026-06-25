"""
Side-by-side comparison sheets: annotated camera image vs reconstructed 3D car.

For every car in a MATLAB results file this script writes
    comparisons/<cam>_<frame>_obj<id>.png
        left  = camera frame cropped to the car, with the 2D annotation drawn
                (tire contacts, extremal/non-extremal symmetry pairs, centers)
        right = the symmetry point cloud with the predicted OLxOWxOH box
                (default, headless-safe) or the real textured GL render (--gl)
plus comparisons/index.html, a scrollable contact sheet of all cars.

Run:
    python make_comparisons.py                      # all cars, matplotlib panel
    python make_comparisons.py --limit 10           # first 10 rows only
    python make_comparisons.py --cam sc1 --frame 0035 --obj 16
    python make_comparisons.py --gl                 # capture the OpenGL render
                                                    # (needs a display + torch)
"""
import argparse
import html
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

import render_io

ROOT = render_io.ROOT
DEFAULT_BATCH = "23_short_cams_thuy_20250329"
SEGMENT = "23"

# Overlay colours (BGR)
COL_TIRE = (0, 215, 255)      # yellow-orange circles
COL_EXTREMAL = (0, 220, 0)    # green
COL_NONEXTREMAL = (0, 140, 255)  # orange
COL_MIRROR = (255, 0, 255)    # magenta
COL_CENTER = (0, 0, 255)      # red


def load_annotation(batch, camera, target, obj_id):
    file = ROOT / "data/gen/formatted_anno" / batch / camera / "annotation" / f"{target}.json"
    for entry in json.loads(file.read_text()):
        if int(entry["obj_id"]) == int(obj_id):
            return entry["annotations"]
    raise KeyError(f"obj_id {obj_id} not in {file}")


def crop_area(batch, camera, target, obj_id):
    """Pixel area (w*h) of the per-object crop in the formatted_anno image/ dir.

    This is the on-screen size of the car the user sees; larger crops are the
    'biggest' cars whose annotations are most distinctively visible. Returns 0
    if the crop is missing so such rows sort to the bottom."""
    crop = (ROOT / "data/gen/formatted_anno" / batch / camera / "image"
            / f"{target}_{int(obj_id)}.jpg")
    if not crop.exists():
        return 0
    with Image.open(crop) as im:
        w, h = im.size
    return w * h


def annotated_image_panel(batch, camera, target, obj_id):
    """Left panel taken straight from the pre-made annotated_images/ red-dot crop
    (data/gen/formatted_anno/<batch>/<cam>/annotated_images/<frame>_<obj>.jpg)."""
    crop = (ROOT / "data/gen/formatted_anno" / batch / camera / "annotated_images"
            / f"{target}_{int(obj_id)}.jpg")
    img = cv2.imread(str(crop))
    if img is None:
        raise FileNotFoundError(f"missing annotated crop {crop} "
                                "(run overlay_annotations.py first)")
    return img


def annotated_crop(batch, camera, target, obj_id, margin=0.30):
    """Camera frame cropped to the car with the annotation drawn on top (BGR)."""
    img_file = ROOT / "data/raw/image" / f"Seg{SEGMENT}" / camera / f"{target}.png"
    img = cv2.imread(str(img_file))
    if img is None:
        raise FileNotFoundError(f"missing raw frame {img_file} "
                                "(restore it from the Google Drive archive)")
    anno = load_annotation(batch, camera, target, obj_id)

    xs, ys = [], []

    def pt(p):
        xs.append(p[0]); ys.append(p[1])
        return int(round(p[0])), int(round(p[1]))

    pairs = list(anno.get("extremal_pairs", []))
    non_ext = list(anno.get("non_extremal_pairs", []))
    # process_annotation.py appends the mirror pair (if any) last in non_extremal
    mirror = non_ext.pop() if anno.get("has_mirror") and non_ext else None

    for x1, y1, x2, y2 in pairs:
        cv2.line(img, pt((x1, y1)), pt((x2, y2)), COL_EXTREMAL, 3)
    for x1, y1, x2, y2 in non_ext:
        cv2.line(img, pt((x1, y1)), pt((x2, y2)), COL_NONEXTREMAL, 3)
    if mirror is not None:
        x1, y1, x2, y2 = mirror
        cv2.line(img, pt((x1, y1)), pt((x2, y2)), COL_MIRROR, 3)
    for cx, cy in anno.get("center_points", []):
        cv2.circle(img, pt((cx, cy)), 6, COL_CENTER, -1)
    for name, p in anno.get("tire_points", {}).items():
        if p[0] < 0:           # [-1,-1] = not annotated
            continue
        c = pt(p)
        cv2.circle(img, c, 9, COL_TIRE, 3)
        cv2.putText(img, name, (c[0] + 10, c[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, COL_TIRE, 2)

    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    mx, my = (x1 - x0) * margin, (y1 - y0) * margin
    h, w = img.shape[:2]
    a = max(0, int(y0 - my)); b = min(h, int(y1 + my))
    c = max(0, int(x0 - mx)); d = min(w, int(x1 + mx))
    return img[a:b, c:d]


def transform_points(raw):
    """Same frame change the renderers use: x=width, y=up, z=length (cm)."""
    t = np.zeros_like(raw, dtype=float)
    t[:, 0] = -raw[:, 2]
    t[:, 1] = -raw[:, 1]
    t[:, 2] = -raw[:, 0]
    t *= 100.0
    t[:, 0] -= np.mean(t[:, 0])
    t[:, 1] -= np.min(t[:, 1])
    t[:, 2] -= np.min(t[:, 2])
    return t


def box_wireframe(ow, oh, ol, heading_deg, center_xz):
    """12 edges of the predicted bounding box, rotated about the up axis."""
    x, y, z = ow / 2, oh, ol / 2
    corners = np.array([[sx * x, sy, sz * z]
                        for sx in (-1, 1) for sy in (0, y) for sz in (-1, 1)], dtype=float)
    th = np.deg2rad(heading_deg)
    rot = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])
    corners = corners @ rot.T
    corners[:, 0] += center_xz[0]
    corners[:, 2] += center_xz[1]
    edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    return corners, edges


def pointcloud_panel(points_raw, row, out_file, dpi=110):
    """Matplotlib 3D view of the point cloud + predicted-dimension box."""
    pts = transform_points(points_raw)
    ow, oh, ol = row.PRED_WWOM * 100, row.PRED_OH * 100, row.PRED_OL * 100
    center = (np.mean(pts[:, 0]), np.mean(pts[:, 2]))
    corners, edges = box_wireframe(ow, oh, ol, row.pred_heading_angle + 180, center)

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection="3d")
    ax.scatter(pts[:, 0], pts[:, 2], pts[:, 1], c="red", s=40, depthshade=False)
    for i, j in edges:
        ax.plot(*zip(corners[i][[0, 2, 1]], corners[j][[0, 2, 1]]),
                c="steelblue", lw=1.5)
    allp = np.vstack([pts, corners])
    mid = allp.mean(axis=0)
    r = max(np.ptp(allp[:, 0]), np.ptp(allp[:, 1]), np.ptp(allp[:, 2])) / 2 + 30
    ax.set_xlim(mid[0] - r, mid[0] + r)
    ax.set_ylim(mid[2] - r, mid[2] + r)
    ax.set_zlim(0, 2 * r)
    ax.set_xlabel("x (cm)"); ax.set_ylabel("z (cm)"); ax.set_zlabel("y (cm)")
    ax.set_title(f"point cloud + predicted box  "
                 f"OL={row.PRED_OL:.2f} OW={row.PRED_WWOM:.2f} OH={row.PRED_OH:.2f} m")
    fig.tight_layout()
    fig.savefig(out_file, dpi=dpi)
    plt.close(fig)


def gl_panel(points_raw, row, out_file, app, render_module):
    """Capture the GL render of one car from `render_module`'s GLWidget.

    Passes the labeled vehicle_type when the renderer's GLWidget accepts it, so
    it fits that type's model; otherwise the renderer falls back to fitting all
    four types and keeping the best."""
    widget = _build_widget(points_raw, row, render_module)
    widget.show()
    for _ in range(10):
        app.processEvents()
    widget.grabFramebuffer().save(str(out_file))
    widget.close()
    app.processEvents()


def _build_widget(points_raw, row, render_module):
    """Construct a renderer GLWidget, passing vehicle_type if it accepts one."""
    import inspect
    args = [points_raw, row["PRED_WWOM"], row["PRED_WB"], row["PRED_OL"],
            row["PRED_OH"], row["pred_heading_angle"], row["dist_to_move"]]
    if "vehicle_type" in inspect.signature(render_module.GLWidget).parameters:
        args.append(render_io.canonical_type(row.get(render_io.VEHICLE_TYPE_COLUMN)))
    return render_module.GLWidget(*args)


def _activate_macos_app():
    """Force this (non-bundled) Python process to become the active macOS app, so
    its GL window comes to the front and receives keyboard/mouse focus instead of
    opening hidden behind the terminal. No-op if AppKit isn't available."""
    try:
        from AppKit import NSApplication, NSApplicationActivationPolicyRegular
        nsapp = NSApplication.sharedApplication()
        nsapp.setActivationPolicy_(NSApplicationActivationPolicyRegular)
        nsapp.activateIgnoringOtherApps_(True)
    except Exception:
        pass


def gl_panel_interactive(points_raw, row, out_file, app, render_module, title):
    """Pop the GL window, let the user orbit it, capture on ENTER.

    Controls (handled by the renderer's GLWidget): drag = rotate, arrow keys =
    pan, W/S = zoom.  ENTER grabs the current framebuffer to out_file; ESC (or
    closing the window) skips this car.  Returns True if a frame was captured."""
    from PyQt5.QtCore import QObject, QEvent, QEventLoop, Qt

    widget = _build_widget(points_raw, row, render_module)
    widget.setWindowTitle(f"{title}   [drag=rotate  arrows=pan  W/S=zoom  "
                          f"ENTER=capture  ESC=skip]")
    loop = QEventLoop()
    state = {"saved": False}

    class Filter(QObject):
        def eventFilter(self, obj, ev):
            t = ev.type()
            if t == QEvent.KeyPress and ev.key() in (Qt.Key_Return, Qt.Key_Enter):
                widget.grabFramebuffer().save(str(out_file))
                state["saved"] = True
                loop.quit()
                return True
            if t == QEvent.KeyPress and ev.key() == Qt.Key_Escape:
                loop.quit()
                return True
            if t == QEvent.Close:
                loop.quit()
            return False

    filt = Filter()
    widget.installEventFilter(filt)
    widget.resize(1000, 1000)
    # keep the window above the terminal and force it frontmost so drag/keys land
    widget.setWindowFlag(Qt.WindowStaysOnTopHint, True)
    widget.show()
    widget.raise_()
    widget.activateWindow()
    widget.setFocus()
    _activate_macos_app()
    app.processEvents()
    widget.raise_()
    widget.activateWindow()
    loop.exec_()
    widget.removeEventFilter(filt)
    widget.close()
    app.processEvents()
    return state["saved"]


def compose(left_bgr, right_file, out_file, header):
    right = cv2.imread(str(right_file))
    h = 760
    def fit(im):
        s = h / im.shape[0]
        return cv2.resize(im, (int(im.shape[1] * s), h))
    left, right = fit(left_bgr), fit(right)
    bar = np.full((52, left.shape[1] + right.shape[1], 3), 30, np.uint8)
    cv2.putText(bar, header, (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (255, 255, 255), 2)
    cv2.imwrite(str(out_file), np.vstack([bar, np.hstack([left, right])]))


def write_index(out_dir, items):
    rows = []
    for name, meta in items:
        rows.append(
            f'<div class="item"><h3>{html.escape(meta)}</h3>'
            f'<a href="{name}"><img src="{name}" loading="lazy"></a></div>')
    out = (out_dir / "index.html")
    out.write_text(
        "<!doctype html><meta charset='utf-8'><title>annotation vs 3D fit</title>"
        "<style>body{font-family:sans-serif;background:#222;color:#eee}"
        ".item{margin:24px 0}img{max-width:100%;border:1px solid #555}"
        "h3{margin:4px 0}</style>"
        f"<h1>Annotated image vs reconstructed 3D car ({len(items)} cars)</h1>"
        + "\n".join(rows))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--results", default=str(render_io.DEFAULT_RESULTS))
    ap.add_argument("--pointcloud", default=str(render_io.DEFAULT_POINT_CLOUD_DIR))
    ap.add_argument("--batch", default=DEFAULT_BATCH,
                    help="formatted_anno batch the results were computed from")
    ap.add_argument("--out", default=str(ROOT / "comparisons"))
    ap.add_argument("--limit", type=int, help="only the first N results rows")
    ap.add_argument("--top", type=int,
                    help="only the N biggest cars by crop area (most visible)")
    ap.add_argument("--cam"); ap.add_argument("--frame")
    ap.add_argument("--obj", type=int)
    ap.add_argument("--gl", action="store_true",
                    help="right panel = real OpenGL render (needs display + torch)")
    ap.add_argument("--renderer", default="originalrender",
                    help="GL render module to use (default: originalrender, the "
                         "parametric polygon model). e.g. optimize_render_car_"
                         "multimodel, rendertooriginal, optimize_render_car")
    ap.add_argument("--left", choices=("reddots", "overlay"), default="reddots",
                    help="left panel source: 'reddots' = the pre-made "
                         "annotated_images/ crops (default), 'overlay' = draw the "
                         "colored symmetry overlay on the raw frame")
    ap.add_argument("--interactive", action="store_true",
                    help="pop each GL window to orbit, press ENTER to capture "
                         "the angle you want (ESC skips). Implies --gl.")
    ap.add_argument("--force", action="store_true",
                    help="re-render even if the output png already exists")
    args = ap.parse_args()
    if args.interactive:
        args.gl = True

    df = render_io.load_results(args.results)
    if args.cam and args.frame and args.obj is not None:
        df = df[(df.camera == args.cam) & (df.target == str(args.frame).zfill(4))
                & (df.annotated_car_id == args.obj)]
    if args.top:
        df = df.assign(_area=[crop_area(args.batch, r.camera, r.target,
                                        r.annotated_car_id)
                              for _, r in df.iterrows()])
        df = df.sort_values("_area", ascending=False).head(args.top)
        print(f"selected {len(df)} biggest cars "
              f"(crop area {int(df._area.min())}..{int(df._area.max())} px)")
    if args.limit:
        df = df.iloc[:args.limit]

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    app = render_module = None
    if args.gl:
        import importlib
        from PyQt5.QtWidgets import QApplication
        app = QApplication([])
        render_module = importlib.import_module(args.renderer)
        if args.interactive:
            _activate_macos_app()
        print(f"GL renderer: {args.renderer}")

    total = len(df)
    items, skipped = [], 0
    for i, (_, row) in enumerate(df.iterrows(), 1):
        cam, target, obj = row.camera, row.target, int(row.annotated_car_id)
        name = f"{cam}_{target}_obj{obj}.png"
        meta = (f"{cam} frame {target} obj {obj} | "
                f"OL={row.PRED_OL:.2f} OW={row.PRED_WWOM:.2f} "
                f"OH={row.PRED_OH:.2f} WB={row.PRED_WB:.2f} m | "
                f"heading={row.pred_heading_angle:.1f} deg | iou={row.iou:.2f}")
        if not args.force and (out_dir / name).exists():
            items.append((name, meta))
            print(f"[{i}/{total}] resume: keep existing {name}")
            continue
        try:
            pts = render_io.load_points(cam, target, obj, args.pointcloud)
            if args.left == "overlay":
                left = annotated_crop(args.batch, cam, target, obj)
            else:
                left = annotated_image_panel(args.batch, cam, target, obj)
            panel = out_dir / f"_panel_{name}"
            if args.interactive:
                print(f"\n[{i}/{total}] {cam} {target} obj {obj}  (iou={row.iou:.2f})\n"
                      f"          window is frontmost: DRAG=rotate  ARROWS=pan  "
                      f"W/S=zoom  ENTER=capture  ESC=skip")
                if not gl_panel_interactive(pts, row, panel, app, render_module, meta):
                    skipped += 1
                    print(f"[{i}/{total}] skipped (no capture) {name}")
                    continue
            elif args.gl:
                gl_panel(pts, row, panel, app, render_module)
            else:
                pointcloud_panel(pts, row, panel)
            compose(left, panel, out_dir / name, meta)
            panel.unlink()
            items.append((name, meta))
            print(f"[{i}/{total}] wrote {name}")
        except (FileNotFoundError, KeyError) as e:
            skipped += 1
            print(f"[{i}/{total}] skip {cam} {target} obj {obj}: {e}")

    index = write_index(out_dir, items)
    print(f"\n{len(items)} comparisons written, {skipped} skipped")
    print(f"open {index}")


if __name__ == "__main__":
    main()
