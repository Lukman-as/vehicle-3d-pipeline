#!/usr/bin/env python3
"""Label vehicle types for the images in each scene folder.

The script looks at its own folder (by default), finds every scene subfolder
that contains an ``image`` directory (e.g. sc1/image, sc2/image, ...), and for
each scene:

  * opens every image in turn in your default viewer,
  * asks you to type the vehicle type in the terminal,
  * writes the result to a ``vehicle_labels.csv`` *inside that scene folder*,
    appending after every entry so progress is saved and runs are resumable.

Usage:
    python3 label_vehicles.py [base_folder]

Controls while labeling:
    <text> + Enter   record the vehicle type for the current image
    (empty) + Enter  skip this image (not written to the CSV)
    s + Enter        skip this image
    n + Enter        skip the rest of this scene, move to the next one
    q + Enter        quit (progress already saved)
"""
import argparse
import csv
import os
import subprocess
import sys

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tif", ".tiff", ".webp"}
CSV_NAME = "vehicle_labels.csv"
IMAGE_DIRNAME = "image"


def open_image(path):
    """Open the image in the OS default viewer."""
    try:
        if sys.platform == "darwin":
            subprocess.run(["open", path], check=False)
        elif sys.platform.startswith("win"):
            os.startfile(path)  # type: ignore[attr-defined]
        else:
            subprocess.run(["xdg-open", path], check=False)
    except Exception as e:
        print(f"  (could not open image automatically: {e})")


def load_done(csv_path):
    """Return the set of image filenames already recorded in the CSV."""
    done = set()
    if os.path.exists(csv_path):
        with open(csv_path, newline="") as f:
            for row in csv.reader(f):
                if row and row[0] != "image":
                    done.add(row[0])
    return done


def label_scene(scene_dir):
    """Label every image in scene_dir/image, writing scene_dir/vehicle_labels.csv.

    Returns "quit" if the user asked to quit, otherwise None.
    """
    image_dir = os.path.join(scene_dir, IMAGE_DIRNAME)
    csv_path = os.path.join(scene_dir, CSV_NAME)
    scene_name = os.path.basename(scene_dir.rstrip(os.sep))

    images = sorted(
        f for f in os.listdir(image_dir)
        if os.path.splitext(f)[1].lower() in IMAGE_EXTS
    )
    if not images:
        print(f"\n[{scene_name}] no images, skipping.")
        return None

    done = load_done(csv_path)
    remaining = [f for f in images if f not in done]

    print(f"\n=== {scene_name} === {len(images)} images "
          f"({len(done)} already labeled, {len(remaining)} to go)")
    print(f"    CSV: {csv_path}")

    if not remaining:
        print("    all done, skipping.")
        return None

    new_file = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(["image", "vehicle_type"])
            f.flush()

        for i, name in enumerate(remaining, 1):
            open_image(os.path.join(image_dir, name))
            try:
                label = input(f"[{scene_name} {i}/{len(remaining)}] {name} -> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nStopped. Progress saved.")
                return "quit"

            low = label.lower()
            if low == "q":
                print("Quit. Progress saved.")
                return "quit"
            if low == "n":
                print(f"  skipping rest of {scene_name}")
                return None
            if label == "" or low == "s":
                print("  skipped")
                continue

            writer.writerow([name, label])
            f.flush()  # write immediately so progress is never lost

    return None


def main():
    ap = argparse.ArgumentParser(description="Label vehicle types per scene folder.")
    ap.add_argument("folder", nargs="?", default=os.path.dirname(os.path.abspath(__file__)),
                    help="Base folder holding the scene subfolders "
                         "(default: the script's own folder)")
    args = ap.parse_args()

    base = os.path.abspath(args.folder)
    if not os.path.isdir(base):
        sys.exit(f"Not a folder: {base}")

    # A scene is any direct subfolder that contains an `image` directory.
    scenes = sorted(
        os.path.join(base, d) for d in os.listdir(base)
        if os.path.isdir(os.path.join(base, d, IMAGE_DIRNAME))
    )
    if not scenes:
        sys.exit(f"No scene folders with an '{IMAGE_DIRNAME}' subfolder found in {base}")

    print(f"Found {len(scenes)} scene(s): {', '.join(os.path.basename(s) for s in scenes)}")
    print("Type the vehicle type and press Enter. "
          "'s'/empty skip image, 'n' next scene, 'q' quit.")

    for scene_dir in scenes:
        if label_scene(scene_dir) == "quit":
            break

    print("\nFinished.")


if __name__ == "__main__":
    main()
