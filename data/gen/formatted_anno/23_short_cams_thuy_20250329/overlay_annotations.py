#!/usr/bin/env python3
"""Overlay annotation points as red dots on each per-object vehicle crop.

The crops in each ``<scene>/image/`` folder are named ``<frame>_<obj_id>.jpg``
and are tight crops of a single vehicle taken from the full camera frame
``data/raw/image/Seg<seg>/<scene>/0001.png``. The annotation points stored in
``<scene>/annotation/<frame>.json`` are in FULL-FRAME coordinates, so they must
be shifted by the crop's upper-left corner before they line up with the crop.

That corner is reproduced exactly from the source label box (the same math the
pipeline used to make the crop, see annotation/process_annotation.py):

    box      = data/raw/label/Seg<seg>/<scene>/<frame>.json[obj_id - 1]['2d_box']
    x_min    = max(0, int(box.xmin - BUFFER * (box.xmax - box.xmin)))
    y_min    = max(0, int(box.ymin - BUFFER * (box.ymax - box.ymin)))
    crop_x   = global_x - x_min
    crop_y   = global_y - y_min

For every scene the script writes annotated copies to
``<scene>/annotated_images/<frame>_<obj_id>.jpg``.

Usage:
    python3 overlay_annotations.py [base_folder]
"""
import argparse
import json
import os
import sys

from PIL import Image, ImageDraw

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tif", ".tiff", ".webp"}
IMAGE_DIRNAME = "image"
ANNO_DIRNAME = "annotation"
OUT_DIRNAME = "annotated_images"

BUFFER = 0.15            # crop padding fraction, matches process_annotation.py
DOT_RADIUS = 4          # red dot radius in pixels
DOT_COLOR = (255, 0, 0)  # red


def find_repo_root(start):
    """Walk up from `start` until a folder containing data/raw is found."""
    cur = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(cur, "data", "raw")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


def crop_origin(box):
    """Reproduce the crop's upper-left (x_min, y_min) from a 2d_box."""
    x_range = box["xmax"] - box["xmin"]
    y_range = box["ymax"] - box["ymin"]
    x_min = max(0, int(box["xmin"] - BUFFER * x_range))
    y_min = max(0, int(box["ymin"] - BUFFER * y_range))
    return x_min, y_min


def collect_points(ann):
    """Return a flat list of (x, y) global points from one object's annotations."""
    points = []
    tire = ann.get("tire_points", {})
    for key in ("DF", "PF", "DR", "PR"):
        pt = tire.get(key)
        if pt and len(pt) == 2 and not (pt[0] == -1 and pt[1] == -1):
            points.append((pt[0], pt[1]))
    for key in ("extremal_pairs", "non_extremal_pairs"):
        for pair in ann.get(key, []):
            if len(pair) >= 4:
                points.append((pair[0], pair[1]))
                points.append((pair[2], pair[3]))
    for pt in ann.get("center_points", []):
        if len(pt) == 2:
            points.append((pt[0], pt[1]))
    return points


def parse_frame_obj(filename):
    """'<frame>_<obj_id>.ext' -> ('<frame>', obj_id). Returns None on miss."""
    stem = os.path.splitext(filename)[0]
    if "_" not in stem:
        return None
    frame, obj_str = stem.rsplit("_", 1)
    try:
        return frame, int(obj_str)
    except ValueError:
        return None


def draw_dots(image_path, points, out_path):
    with Image.open(image_path) as im:
        im = im.convert("RGB")
        draw = ImageDraw.Draw(im)
        for x, y in points:
            draw.ellipse(
                [x - DOT_RADIUS, y - DOT_RADIUS, x + DOT_RADIUS, y + DOT_RADIUS],
                fill=DOT_COLOR,
                outline=DOT_COLOR,
            )
        im.save(out_path)


def process_scene(scene_dir, label_root):
    scene = os.path.basename(scene_dir.rstrip(os.sep))
    image_dir = os.path.join(scene_dir, IMAGE_DIRNAME)
    anno_dir = os.path.join(scene_dir, ANNO_DIRNAME)
    out_dir = os.path.join(scene_dir, OUT_DIRNAME)
    label_dir = os.path.join(label_root, scene)

    if not os.path.isdir(image_dir) or not os.path.isdir(anno_dir):
        return None

    os.makedirs(out_dir, exist_ok=True)
    anno_cache = {}
    label_cache = {}
    stats = dict(written=0, missing_anno=0, missing_obj=0,
                 missing_label=0, dim_mismatch=0, skipped=0)

    for filename in sorted(os.listdir(image_dir)):
        if os.path.splitext(filename)[1].lower() not in IMAGE_EXTS:
            continue
        parsed = parse_frame_obj(filename)
        if parsed is None:
            print(f"    ! cannot parse {filename}")
            stats["skipped"] += 1
            continue
        frame, obj_id = parsed

        # annotation (formatted) -------------------------------------------
        if frame not in anno_cache:
            ap = os.path.join(anno_dir, frame + ".json")
            anno_cache[frame] = json.load(open(ap)) if os.path.exists(ap) else None
        objects = anno_cache[frame]
        if objects is None:
            print(f"    ! no annotation file for frame {frame} ({filename})")
            stats["missing_anno"] += 1
            continue
        match = next((o for o in objects if o.get("obj_id") == obj_id), None)
        if match is None:
            print(f"    ! obj_id {obj_id} not in {frame}.json ({filename})")
            stats["missing_obj"] += 1
            continue

        # source label box -> crop origin ----------------------------------
        if frame not in label_cache:
            lp = os.path.join(label_dir, frame + ".json")
            label_cache[frame] = json.load(open(lp)) if os.path.exists(lp) else None
        labels = label_cache[frame]
        raw_idx = obj_id - 1  # JSON obj_id == raw label index + 1
        if labels is None or raw_idx < 0 or raw_idx >= len(labels):
            print(f"    ! no source label for {filename} (idx {raw_idx})")
            stats["missing_label"] += 1
            continue
        ox, oy = crop_origin(labels[raw_idx]["2d_box"])

        in_path = os.path.join(image_dir, filename)
        with Image.open(in_path) as im:
            cw, ch = im.size
        # sanity check: points should land inside the crop
        pts = [(x - ox, y - oy) for (x, y) in collect_points(match.get("annotations", {}))]
        if pts:
            bx = max(0, -min(p[0] for p in pts)) + max(0, max(p[0] for p in pts) - cw)
            by = max(0, -min(p[1] for p in pts)) + max(0, max(p[1] for p in pts) - ch)
            if bx > 5 or by > 5:
                print(f"    ! {filename}: points fall outside crop "
                      f"(overshoot x={bx:.0f} y={by:.0f}); offset may be wrong")
                stats["dim_mismatch"] += 1

        draw_dots(in_path, pts, os.path.join(out_dir, filename))
        stats["written"] += 1

    stats["out_dir"] = out_dir
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_folder", nargs="?",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()
    base = os.path.abspath(args.base_folder)

    # segment number from the leading digits of the dataset folder name
    folder_name = os.path.basename(base)
    seg_digits = ""
    for ch in folder_name:
        if ch.isdigit():
            seg_digits += ch
        else:
            break
    if not seg_digits:
        print(f"Could not infer Seg<number> from folder name '{folder_name}'")
        sys.exit(1)

    repo_root = find_repo_root(base)
    if repo_root is None:
        print("Could not locate repo root containing data/raw")
        sys.exit(1)
    label_root = os.path.join(repo_root, "data", "raw", "label", f"Seg{seg_digits}")
    if not os.path.isdir(label_root):
        print(f"Source label folder not found: {label_root}")
        sys.exit(1)
    print(f"Using source labels from {label_root}\n")

    scenes = sorted(d for d in os.listdir(base)
                    if os.path.isdir(os.path.join(base, d, IMAGE_DIRNAME)))
    if not scenes:
        print(f"No scene folders with an '{IMAGE_DIRNAME}/' dir in {base}")
        sys.exit(1)

    grand = 0
    for scene in scenes:
        print(f"== {scene} ==")
        r = process_scene(os.path.join(base, scene), label_root)
        if r is None:
            print("    (skipped: missing image/ or annotation/)")
            continue
        grand += r["written"]
        print(f"    wrote {r['written']} -> {r['out_dir']}\n"
              f"      missing_anno={r['missing_anno']}, missing_obj={r['missing_obj']}, "
              f"missing_label={r['missing_label']}, outside_crop={r['dim_mismatch']}, "
              f"skipped={r['skipped']}")
    print(f"\nDone. {grand} annotated images written.")


if __name__ == "__main__":
    main()
