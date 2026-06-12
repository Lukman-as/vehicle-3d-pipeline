"""Headless validation of car_models_core on the real sample data."""
import os
import numpy as np
import pandas as pd
import car_models_core as core

SAMPLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "samples")


def load_points(points_file):
    raw = pd.read_csv(points_file, header=None).values
    t = np.zeros_like(raw, dtype=float)
    t[:, 0] = -raw[:, 2]
    t[:, 1] = -raw[:, 1]
    t[:, 2] = -raw[:, 0]
    t *= 100.0
    t[:, 0] -= np.mean(t[:, 0])
    t[:, 1] -= np.min(t[:, 1])
    t[:, 2] -= np.min(t[:, 2])
    return t


# Read sample.txt exactly like the renderer's __main__.
col_names = ["camera", "target", "annotated_car_id", "num_sym_pairs", "bbox_2D_height",
             "reproj_error", "gt_heading_angle", "pred_heading_angle", "angle_difference",
             "dist_base_gt_bbox", "dist_base_pred_bbox", "dist_base_bbox_diff",
             "dist_nearest_corner_gt_bbox", "dist_nearest_corner_pred_bbox",
             "dist_nearest_corner_diff", "iou", "iou_bev", "mounting_height", "ds",
             "PRED_OL", "PRED_OW", "PRED_OH", "PRED_WB", "LD_OL", "LD_OW", "LD_OH",
             "PRED_WWOM", "tire_both_sides", "has_mirrors", "dist_to_move",
             "LD_OW_NON", "LD_OH_NON", "LD_OL_NON", "LENGTH_BY_GAUSSIAN", "NUM_TIRES"]
df = pd.read_csv(os.path.join(SAMPLES, "sample.txt"), sep=" ", header=None)
df.columns = col_names

# quick geometry sanity: every template builds and has >0 quads/4 tires
print("=== geometry build sanity (mid-size car) ===")
for vt in core.VEHICLE_TYPES:
    p = core.conditional_params(vt, 185, 280, 160, 480)
    g = core.build_geometry(p, vt)
    pts = np.vstack([v for q in g['quads'] for v in q['vertices']])
    print(f"  {vt:6} quads={len(g['quads']):2d} tires={len(g['cylinders'])} "
          f"bbox x[{pts[:,0].min():.0f},{pts[:,0].max():.0f}] "
          f"y[{pts[:,1].min():.0f},{pts[:,1].max():.0f}] z[{pts[:,2].min():.0f},{pts[:,2].max():.0f}]")

print("\n=== best-fit selection on the 3 real cars ===")
for i in range(3):
    row = df.iloc[i]
    pts = load_points(os.path.join(SAMPLES, f"points{i}.csv"))
    ow = row["PRED_WWOM"] * 100
    wb = row["PRED_WB"] * 100
    ol = row["PRED_OL"] * 100
    oh = row["PRED_OH"] * 100
    init_angle = core.as_scalar(row["pred_heading_angle"]) + 180
    best, results = core.select_best_fit(pts, ow, wb, oh, ol, init_angle, iters=50)
    print(f"\n car {i}: n_pts={len(pts)}  dims(cm) OW={ow:.0f} WB={wb:.0f} OH={oh:.0f} OL={ol:.0f}")
    ranked = sorted(results.items(), key=lambda kv: kv[1]['residual'])
    for vt, r in ranked:
        mark = "  <-- BEST" if vt == best else ""
        print(f"     {vt:6} residual={r['residual']:10.3f}  angle={r['angle']:7.1f}{mark}")
