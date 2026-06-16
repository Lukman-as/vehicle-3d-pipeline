#!/usr/bin/env python3
"""Apply vehicle-type labels from each scene's CSV into the annotation JSONs.

For every scene folder (sc1, sc2, ...) under the base folder this script reads
``<scene>/vehicle_labels.csv``. Each CSV row is ``<frame>_<obj_id>.jpg,<type>``.
The matching annotation is ``<scene>/annotation/<frame>.json``, a list of
objects each with an ``obj_id``. The script sets ``vehicle_type`` on the object
whose ``obj_id`` matches the id in the image filename.

Usage:
    python3 apply_labels.py [base_folder]      # default: script's own folder
"""
import csv
import json
import os
import sys

CSV_NAME = "vehicle_labels.csv"
ANNO_DIRNAME = "annotation"


def parse_image_name(name):
    """'0001_42.jpg' -> ('0001', 42). Returns (frame, obj_id) or None."""
    stem = os.path.splitext(name)[0]
    if "_" not in stem:
        return None
    frame, obj = stem.rsplit("_", 1)
    try:
        return frame, int(obj)
    except ValueError:
        return None


def apply_scene(scene_dir):
    scene = os.path.basename(scene_dir.rstrip(os.sep))
    csv_path = os.path.join(scene_dir, CSV_NAME)
    anno_dir = os.path.join(scene_dir, ANNO_DIRNAME)
    if not os.path.isfile(csv_path):
        print(f"[{scene}] no {CSV_NAME}, skipping.")
        return

    # Group labels by frame so each JSON file is read/written once.
    by_frame = {}  # frame -> {obj_id: vehicle_type}
    with open(csv_path, newline="") as f:
        for row in csv.reader(f):
            if not row or row[0] == "image":
                continue
            parsed = parse_image_name(row[0])
            if parsed is None:
                print(f"[{scene}] cannot parse image name '{row[0]}', skipping row.")
                continue
            frame, obj_id = parsed
            by_frame.setdefault(frame, {})[obj_id] = row[1]

    updated = matched = missing_files = missing_ids = 0
    for frame, labels in sorted(by_frame.items()):
        json_path = os.path.join(anno_dir, f"{frame}.json")
        if not os.path.isfile(json_path):
            print(f"[{scene}] missing annotation file {frame}.json")
            missing_files += 1
            continue
        with open(json_path) as f:
            data = json.load(f)

        found = {}
        changed = False
        for entry in data:
            oid = entry.get("obj_id")
            if oid in labels:
                if entry.get("vehicle_type") != labels[oid]:
                    entry["vehicle_type"] = labels[oid]
                    changed = True
                found[oid] = True
                matched += 1

        for oid in labels:
            if oid not in found:
                print(f"[{scene}] {frame}.json has no obj_id {oid}")
                missing_ids += 1

        if changed:
            with open(json_path, "w") as f:
                json.dump(data, f, indent=2)
            updated += 1

    print(f"[{scene}] matched {matched} objects, wrote {updated} json file(s)"
          + (f", {missing_files} missing file(s)" if missing_files else "")
          + (f", {missing_ids} unmatched id(s)" if missing_ids else ""))


def main():
    base = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 \
        else os.path.dirname(os.path.abspath(__file__))
    scenes = sorted(
        os.path.join(base, d) for d in os.listdir(base)
        if os.path.isdir(os.path.join(base, d, ANNO_DIRNAME))
    )
    if not scenes:
        sys.exit(f"No scene folders with an '{ANNO_DIRNAME}' subfolder in {base}")
    for scene_dir in scenes:
        apply_scene(scene_dir)
    print("Done.")


if __name__ == "__main__":
    main()
