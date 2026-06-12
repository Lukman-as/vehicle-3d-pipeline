"""
Bridge between the MATLAB stage and the Python renderers.

The MATLAB script (matlab/hw7_exp_official.m) writes, per run:
    data/gen/matlab_output/hw7_results_<annotator>_<algo>_<bounds>.txt
        one whitespace-separated row per reconstructed car (35 columns)
    data/gen/matlab_output/point_cloud/<camera>/<frame>.json
        [{"obj_id": <id>, "points": [[x,y,z], ...]}, ...]

This module reads both directly, so renderers no longer need hand-copied
sample.txt / pointsN.csv files.  A car is identified everywhere by the
triple (camera, frame, obj_id) == (camera, target, annotated_car_id).

Renderer CLI (shared via select_car()):
    python rendertooriginal.py                 # first row of the results file
    python rendertooriginal.py 5               # row 5 of the results file
    python rendertooriginal.py --cam sc1 --frame 0035 --obj 16
    python rendertooriginal.py --results path/to/other_results.txt 3
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
MATLAB_OUT = ROOT / "data" / "gen" / "matlab_output"
DEFAULT_RESULTS = MATLAB_OUT / "hw7_results_thuy_avg_bounds.txt"
DEFAULT_POINT_CLOUD_DIR = MATLAB_OUT / "point_cloud"

# Column layout of hw7_results_*.txt (written by hw7_exp_official.m).
RESULTS_COLUMNS = [
    "camera", "target", "annotated_car_id", "num_sym_pairs", "bbox_2D_height",
    "reproj_error", "gt_heading_angle", "pred_heading_angle", "angle_difference",
    "dist_base_gt_bbox", "dist_base_pred_bbox", "dist_base_bbox_diff",
    "dist_nearest_corner_gt_bbox", "dist_nearest_corner_pred_bbox",
    "dist_nearest_corner_diff", "iou", "iou_bev", "mounting_height", "ds",
    "PRED_OL", "PRED_OW", "PRED_OH", "PRED_WB", "LD_OL", "LD_OW", "LD_OH",
    "PRED_WWOM", "tire_both_sides", "has_mirrors", "dist_to_move",
    "LD_OW_NON", "LD_OH_NON", "LD_OL_NON", "LENGTH_BY_GAUSSIAN", "NUM_TIRES",
]


def load_results(results_file=DEFAULT_RESULTS):
    """Read a MATLAB results txt into a DataFrame with named columns.
    'target' is kept as a zero-padded string ('0035') to match file names."""
    df = pd.read_csv(results_file, sep=r"\s+", header=None, dtype={1: str})
    df.columns = RESULTS_COLUMNS
    df["target"] = df["target"].str.zfill(4)
    return df


def load_points(camera, target, obj_id, point_cloud_dir=DEFAULT_POINT_CLOUD_DIR):
    """Return the (N,3) symmetry point cloud for one car."""
    target = str(target).zfill(4)
    file = Path(point_cloud_dir) / str(camera) / f"{target}.json"
    for obj in json.loads(file.read_text()):
        if int(obj["obj_id"]) == int(obj_id):
            return np.asarray(obj["points"], dtype=float)
    raise KeyError(f"obj_id {obj_id} not found in {file}")


def select_car(argv=None, description="Render one reconstructed car."):
    """Shared renderer CLI.  Returns (results_row, points ndarray)."""
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("index", nargs="?", type=int, default=0,
                    help="row index in the results file (default 0)")
    ap.add_argument("--cam", help="camera, e.g. sc1")
    ap.add_argument("--frame", help="frame/target, e.g. 0035")
    ap.add_argument("--obj", type=int, help="annotated car id, e.g. 16")
    ap.add_argument("--results", default=str(DEFAULT_RESULTS),
                    help="hw7_results_*.txt to read")
    ap.add_argument("--pointcloud", default=str(DEFAULT_POINT_CLOUD_DIR),
                    help="point_cloud directory to read")
    # parse_known_args so Qt flags (e.g. -platform) pass through untouched
    args, _ = ap.parse_known_args(argv)

    df = load_results(args.results)
    if args.cam or args.frame or args.obj is not None:
        if not (args.cam and args.frame and args.obj is not None):
            ap.error("--cam, --frame and --obj must be given together")
        sel = df[(df.camera == args.cam)
                 & (df.target == str(args.frame).zfill(4))
                 & (df.annotated_car_id == args.obj)]
        if sel.empty:
            ap.error(f"no results row for {args.cam} frame {args.frame} obj {args.obj}")
        row = sel.iloc[0]
    else:
        row = df.iloc[args.index]

    points = load_points(row.camera, row.target, row.annotated_car_id, args.pointcloud)
    print(f"[render_io] {row.camera} frame {row.target} obj {int(row.annotated_car_id)} "
          f"({len(points)} points)  OLxOWxOH={row.PRED_OL:.2f}x{row.PRED_OW:.2f}x{row.PRED_OH:.2f} m")
    return row, points
