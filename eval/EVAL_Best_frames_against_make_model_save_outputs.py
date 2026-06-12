from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Iterable

import pandas as pd

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
PROJECT_DIR = Path(__file__).resolve().parents[1]
MAKE_MODEL_DIR = Path(__file__).resolve().parent / "hw7_saved_make_model"
TRACKS_DIR = PROJECT_DIR / "data" / "gen" / "saved_tracks_json" / "Seg23"
OUTPUT_DIR = PROJECT_DIR / "data" / "gen" / "eval_outputs"

ALL_CAMS = ["sc1", "sc2", "sc3", "sc4"]

BEST_FRAMES = False
ANNO = "thuy" if BEST_FRAMES else "aime"
ANGLE = "avg"
BOUNDS = "bounds"
FIXTIRE_SUFFIX = ""

REFINE_DIM = False
DIST_BASE_GT_MAX = 150

WIDTH_USED = "WWOM"          # Evaluates PRED_WWOM as width.
MAKE_MODEL_WIDTH_COL = "GT_OW"
USE_WHAT_LIDAR = ""          # Set to "_NON" to use LD_*_NON columns where available.

RESULT_FILE = PROJECT_DIR / "data" / "gen" / "matlab_output" / f"hw7_results_{ANNO}_{ANGLE}_{BOUNDS}{FIXTIRE_SUFFIX}.txt"
RUN_NAME = RESULT_FILE.stem

SORT_COLS = ["camera", "target", "annotated_car_id"]
ID_COLS = ["camera", "target", "annotated_car_id", "track_name", "model", "class", "type"]

RAW_COL_NAMES = [
    "camera", "target", "annotated_car_id",
    "num_sym_pairs", "bbox_2D_height",
    "reproj_error", "gt_heading_angle",
    "pred_heading_angle", "angle_difference",
    "dist_base_gt_bbox", "dist_base_pred_bbox",
    "dist_base_bbox_diff", "dist_nearest_corner_gt_bbox",
    "dist_nearest_corner_pred_bbox", "dist_nearest_corner_diff",
    "iou", "iou_bev", "mounting_height", "ds",
    "PRED_OL", "PRED_OW", "PRED_OH", "PRED_WB",
    "LD_OL", "LD_OW", "LD_OH", "PRED_WWOM",
    "tire_both_sides", "has_mirrors", "dist_to_move",
    "LD_OW_NON", "LD_OH_NON", "LD_OL_NON", "LENGTH_BY_GAUSSIAN", "NUM_TIRES",
]

# Preserve the old notebook's slightly odd column reordering.
COL_NAMES = ["camera", "target", "annotated_car_id"] + [
    c for c in RAW_COL_NAMES if c not in {"camera", "target", "annotated_car_id"}
]

VEHICLE_CLASSIFICATION = {
    "Porsche Macan": "Vans and SUVs",
    "Hyundai Elantra": "Sedans",
    "Acura MDX": "Vans and SUVs",
    "Toyota Camry": "Sedans",
    "Lexus ES": "Sedans",
    "Honda Pilot": "Vans and SUVs",
    "Kia Sportage": "Vans and SUVs",
    "Ford Edge": "Vans and SUVs",
    "Mercedes-Benz GLC": "Vans and SUVs",
    "Nissan Altima": "Sedans",
    "Tesla Model 3": "Sedans",
    "Honda CR-V": "Vans and SUVs",
    "Toyota Matrix": "Hatchbacks and Station Wagons",
    "Mercedes-Benz C300 4MATIC": "Sedans",
    "BMW X5": "Vans and SUVs",
    "Ford Escape": "Vans and SUVs",
    "Toyota 4Runner": "Vans and SUVs",
    "Toyota RAV4": "Vans and SUVs",
    "BMW X3": "Vans and SUVs",
    "Toyota RAV4 Hybrid": "Vans and SUVs",
    "BMW X3 xDrive28i": "Vans and SUVs",
    "Honda Civic": "Sedans",
    "Mercedes-Benz C-Class": "Sedans",
    "Toyota Corolla": "Sedans",
    "Ford Fusion": "Sedans",
}


def critical_dims_for(lidar: bool, width_used: str = WIDTH_USED) -> list[str]:
    """Dimensions evaluated by each source.

    Lidar has no wheelbase column in this file, so WB is only evaluated for the
    prediction method.
    """
    return ["OH", "OL", width_used] if lidar else ["OH", "OL", width_used, "WB"]


def build_column_labels(width_used: str = WIDTH_USED) -> dict[str, str]:
    return {
        "DIFF_OH": "DIFF Height (m)",
        "DIFF_OL": "DIFF Length (m)",
        f"DIFF_{width_used}": "DIFF Width (m)",
        "DIFF_WB": "DIFF Wheelbase (m)",
        "ABS_DIFF_OH": "MAE Height (m)",
        "ABS_DIFF_OL": "MAE Length (m)",
        f"ABS_DIFF_{width_used}": "MAE Width (m)",
        "ABS_DIFF_WB": "MAE Wheelbase (m)",
        "ABS_DIFF_PERCENT_OH": "Mean percentage of Absolute Error over groundtruth vehicle's Height",
        "ABS_DIFF_PERCENT_OL": "Mean percentage of Absolute Error over groundtruth vehicle's Length",
        f"ABS_DIFF_PERCENT_{width_used}": "Mean percentage of Absolute Error over groundtruth vehicle's Width",
        "ABS_DIFF_PERCENT_WB": "Mean percentage of Absolute Error over groundtruth vehicle's Wheelbase",
        "iou": r"$\mathbf{IoU}$",
        "dist_base_bbox_diff_abs": "MAE location (m)",
        "angle_difference": "Mean heading angle error (deg)",
        "N": "Number of vehicles",
    }


def load_make_model(make_model_dir: Path = MAKE_MODEL_DIR, all_cams: Iterable[str] = ALL_CAMS) -> dict:
    all_cams_make_model: dict[str, dict] = {}
    for cam in all_cams:
        with open(make_model_dir / f"seg23_{cam}_make_model_20240319.json", "r") as f:
            one_cam = json.load(f)
        all_cams_make_model[cam] = one_cam[next(iter(one_cam.keys()))]
    return all_cams_make_model


def load_track_lookup(tracks_dir: Path = TRACKS_DIR, all_cams: Iterable[str] = ALL_CAMS) -> dict:
    car_track_lookup: dict[str, dict[str, str]] = {}
    for cam in all_cams:
        with open(tracks_dir / cam / f"Seg23_{cam}_20231011.json", "r") as f:
            tracks = json.load(f)
        car_to_track: dict[str, str] = {}
        for track_name, cars in tracks.items():
            for car_id in cars:
                car_to_track[car_id] = track_name
        car_track_lookup[cam] = car_to_track
    return car_track_lookup


def load_results(result_file: Path = RESULT_FILE) -> pd.DataFrame:
    df = pd.read_csv(result_file, sep=r"\s+", header=None)
    if len(RAW_COL_NAMES) != df.shape[-1]:
        raise ValueError(
            f"Expected {len(RAW_COL_NAMES)} columns in {result_file}, got {df.shape[-1]}"
        )

    df.columns = COL_NAMES
    df = df[(df.camera != "lc1") & (df.camera != "lc2")].copy()

    df["dist_base_bbox_diff_abs"] = df["dist_base_bbox_diff"].abs()
    df["dist_nearest_corner_diff_abs"] = df["dist_nearest_corner_diff"].abs()
    df = df[df.dist_base_gt_bbox <= DIST_BASE_GT_MAX].copy()

    if REFINE_DIM:
        for metric_to_check in ["PRED_OW", "PRED_WWOM"]:
            thres_to_check = 1.56 * 0.5  # From DB, preserved from original notebook.
            print(len(df[df[metric_to_check] < thres_to_check]), "violate dimension", metric_to_check)
            df = df[~(df[metric_to_check] < thres_to_check)].copy()

    return df.sort_values(by=SORT_COLS).reset_index(drop=True)


def get_gt_dims(row: pd.Series, car_track_lookup: dict, all_cams_make_model: dict) -> pd.Series:
    lookup_str = f"{str(int(row.target)).zfill(4)}_{int(row.annotated_car_id - 1)}"
    cam_lookup = car_track_lookup.get(row.camera, {})

    if lookup_str in cam_lookup:
        track_name = cam_lookup[lookup_str]
        cam_make_model = all_cams_make_model.get(row.camera, {})
        if track_name in cam_make_model:
            model_detect = cam_make_model[track_name]
            model = next(iter(model_detect.keys()))
            row["model"] = model
            row["class"] = VEHICLE_CLASSIFICATION.get(model, "")
            for k, v in model_detect[model].items():
                row[f"GT_{k}"] = v / 100  # Convert cm to m.
        row["track_name"] = track_name

    return row


def add_type(row: pd.Series) -> pd.Series:
    if row.tire_both_sides:
        vehicle_type = 1
    elif row.has_mirrors:
        vehicle_type = 2
    elif row.PRED_WB > 0:
        vehicle_type = 3
    else:
        vehicle_type = 4
    row["type"] = vehicle_type
    return row


def build_target_df(df: pd.DataFrame, car_track_lookup: dict, all_cams_make_model: dict) -> pd.DataFrame:
    target_df = df.apply(
        lambda row: get_gt_dims(row, car_track_lookup, all_cams_make_model),
        axis=1,
    )

    # This intentionally preserves the old notebook behavior. A narrower
    # dropna(subset=["GT_OH", "GT_OL", "GT_OW", "GT_WB"]) may be safer later,
    # but it could change the reported numbers.
    target_df = target_df.dropna()
    target_df = target_df.apply(add_type, axis=1)
    return target_df.sort_values(by=SORT_COLS).reset_index(drop=True)


def compute_dimension_errors(
    target_df: pd.DataFrame,
    *,
    lidar: bool,
    width_used: str = WIDTH_USED,
    make_model_width_col: str = MAKE_MODEL_WIDTH_COL,
    lidar_suffix: str = USE_WHAT_LIDAR,
    adjusting_for_tw: float = 0.0,
) -> tuple[pd.DataFrame, list[str]]:
    """Return a copy of target_df with DIFF_*, ABS_DIFF_*, and percent columns.

    This is a vectorized replacement for the notebook's row-wise compare_dims +
    eval_diff_percent steps.
    """
    evaluated = target_df.copy()
    dims = critical_dims_for(lidar, width_used)

    for dim in dims:
        if lidar:
            pred_col = f"LD_OW{lidar_suffix}" if dim == width_used else f"LD_{dim}{lidar_suffix}"
            gt_col = make_model_width_col if dim == width_used else f"GT_{dim}"
            adjustment = 0.0
        else:
            pred_col = f"PRED_{dim}"
            gt_col = make_model_width_col if dim == width_used else f"GT_{dim}"
            adjustment = adjusting_for_tw if dim == width_used else 0.0

        if pred_col not in evaluated.columns:
            raise KeyError(f"Missing prediction/source column: {pred_col}")
        if gt_col not in evaluated.columns:
            raise KeyError(f"Missing ground-truth column: {gt_col}")

        pred = pd.to_numeric(evaluated[pred_col], errors="coerce") - adjustment
        gt = pd.to_numeric(evaluated[gt_col], errors="coerce")

        evaluated[f"DIFF_{dim}"] = pred - gt
        evaluated[f"ABS_DIFF_{dim}"] = evaluated[f"DIFF_{dim}"].abs()
        evaluated[f"ABS_DIFF_PERCENT_{dim}"] = (evaluated[f"ABS_DIFF_{dim}"] / gt * 100).round(3)

    metrics = [f"DIFF_{d}" for d in dims] + [f"ABS_DIFF_{d}" for d in dims] + [
        f"ABS_DIFF_PERCENT_{d}" for d in dims
    ]
    return evaluated, metrics


def build_summary_df(
    evaluated_df: pd.DataFrame,
    metrics: list[str],
    *,
    method_name: str,
    width_used: str = WIDTH_USED,
) -> pd.DataFrame:
    labels = build_column_labels(width_used)
    summary_df = pd.DataFrame(evaluated_df[metrics].mean()).T
    summary_df.columns = [labels.get(c, c) for c in summary_df.columns]
    summary_df["Number of vehicles"] = int(len(evaluated_df))
    summary_df.index = [method_name]
    summary_df.index.name = "method"
    return summary_df


def generate_out_df(df: pd.DataFrame, metrics: list[str], *, method_name: str = "pred") -> pd.DataFrame:
    """Backward-compatible name for building the summary DataFrame.

    The old notebook function returned only a LaTeX string. This version returns
    the actual DataFrame so it can be saved, tested, and re-used by analysis.
    Use `summary_to_latex(generate_out_df(...))` when LaTeX is needed.
    """
    return build_summary_df(df, metrics, method_name=method_name)


def summary_to_latex(summary_df: pd.DataFrame) -> str:
    with pd.option_context("display.max_colwidth", 1000):
        return summary_df.T.to_latex(
            index=True,
            formatters={"name": str.upper},
            float_format="{:.5f}".format,
            multirow=True,
            multicolumn=True,
            multicolumn_format="c",
            position="h",
            bold_rows=True,
        )


def export_method_artifacts(
    *,
    evaluated_df: pd.DataFrame,
    metrics: list[str],
    method_name: str,
    output_dir: Path,
    run_name: str,
) -> dict[str, str]:
    summary_df = build_summary_df(evaluated_df, metrics, method_name=method_name)
    latex = summary_to_latex(summary_df)

    id_cols = [c for c in ID_COLS if c in evaluated_df.columns]
    metrics_export_df = evaluated_df[id_cols + metrics].copy()

    eval_csv = output_dir / f"{run_name}_{method_name}_eval_df.csv"
    metrics_csv = output_dir / f"{run_name}_{method_name}_metrics.csv"
    summary_csv = output_dir / f"{run_name}_{method_name}_summary.csv"
    latex_tex = output_dir / f"{run_name}_{method_name}_summary.tex"

    evaluated_df.to_csv(eval_csv, index=False)
    metrics_export_df.to_csv(metrics_csv, index=False)
    summary_df.to_csv(summary_csv, index=True)
    latex_tex.write_text(latex, encoding="utf-8")

    return {
        "eval_csv": str(eval_csv),
        "metrics_csv": str(metrics_csv),
        "summary_csv": str(summary_csv),
        "summary_tex": str(latex_tex),
    }


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    all_cams_make_model = load_make_model()
    car_track_lookup = load_track_lookup()
    df1 = load_results()

    print(f"Rows after raw filters: {len(df1)}")
    target_df = build_target_df(df1, car_track_lookup, all_cams_make_model)
    print(f"Rows with make/model GT: {len(target_df)}")

    target_csv = OUTPUT_DIR / f"{RUN_NAME}_target_df.csv"
    target_df.to_csv(target_csv, index=False)

    pred_eval_df, pred_metrics = compute_dimension_errors(target_df, lidar=False)
    lidar_eval_df, lidar_metrics = compute_dimension_errors(target_df, lidar=True)

    pred_artifacts = export_method_artifacts(
        evaluated_df=pred_eval_df,
        metrics=pred_metrics,
        method_name="pred",
        output_dir=OUTPUT_DIR,
        run_name=RUN_NAME,
    )
    lidar_artifacts = export_method_artifacts(
        evaluated_df=lidar_eval_df,
        metrics=lidar_metrics,
        method_name="lidar",
        output_dir=OUTPUT_DIR,
        run_name=RUN_NAME,
    )

    latest_artifacts = {
        "target_csv": OUTPUT_DIR / "latest_eval_target_df.csv",
        "pred_eval_csv": OUTPUT_DIR / "latest_eval_pred_eval_df.csv",
        "pred_metrics_csv": OUTPUT_DIR / "latest_eval_pred_metrics.csv",
        "pred_summary_csv": OUTPUT_DIR / "latest_eval_pred_summary.csv",
        "pred_summary_tex": OUTPUT_DIR / "latest_eval_pred_summary.tex",
        "lidar_eval_csv": OUTPUT_DIR / "latest_eval_lidar_eval_df.csv",
        "lidar_metrics_csv": OUTPUT_DIR / "latest_eval_lidar_metrics.csv",
        "lidar_summary_csv": OUTPUT_DIR / "latest_eval_lidar_summary.csv",
        "lidar_summary_tex": OUTPUT_DIR / "latest_eval_lidar_summary.tex",
    }

    shutil.copyfile(target_csv, latest_artifacts["target_csv"])
    shutil.copyfile(pred_artifacts["eval_csv"], latest_artifacts["pred_eval_csv"])
    shutil.copyfile(pred_artifacts["metrics_csv"], latest_artifacts["pred_metrics_csv"])
    shutil.copyfile(pred_artifacts["summary_csv"], latest_artifacts["pred_summary_csv"])
    shutil.copyfile(pred_artifacts["summary_tex"], latest_artifacts["pred_summary_tex"])
    shutil.copyfile(lidar_artifacts["eval_csv"], latest_artifacts["lidar_eval_csv"])
    shutil.copyfile(lidar_artifacts["metrics_csv"], latest_artifacts["lidar_metrics_csv"])
    shutil.copyfile(lidar_artifacts["summary_csv"], latest_artifacts["lidar_summary_csv"])
    shutil.copyfile(lidar_artifacts["summary_tex"], latest_artifacts["lidar_summary_tex"])

    manifest = {
        "run_name": RUN_NAME,
        "result_file": str(RESULT_FILE),
        "best_frames": BEST_FRAMES,
        "anno": ANNO,
        "angle": ANGLE,
        "bounds": BOUNDS,
        "fixtire_suffix": FIXTIRE_SUFFIX,
        "refine_dim": REFINE_DIM,
        "dist_base_gt_max": DIST_BASE_GT_MAX,
        "width_used": WIDTH_USED,
        "make_model_width_col": MAKE_MODEL_WIDTH_COL,
        "use_what_lidar": USE_WHAT_LIDAR,
        "critical_dims": {
            "pred": critical_dims_for(False),
            "lidar": critical_dims_for(True),
        },
        "row_counts": {
            "raw_filtered": int(len(df1)),
            "target_with_gt": int(len(target_df)),
        },
        "metrics": {
            "pred": pred_metrics,
            "lidar": lidar_metrics,
        },
        "artifacts": {
            "target_csv": str(target_csv),
            "pred": pred_artifacts,
            "lidar": lidar_artifacts,
        },
        "latest_artifacts": {key: str(value) for key, value in latest_artifacts.items()},
    }

    manifest_path = OUTPUT_DIR / f"{RUN_NAME}_manifest.json"
    latest_manifest_path = OUTPUT_DIR / "latest_eval_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    latest_manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print("\nPrediction summary:\n", summary_to_latex(pd.read_csv(pred_artifacts["summary_csv"], index_col=0)))
    print("\nLidar summary:\n", summary_to_latex(pd.read_csv(lidar_artifacts["summary_csv"], index_col=0)))
    print(f"\nSaved artifacts under: {OUTPUT_DIR.resolve()}")
    print(f"Latest manifest: {latest_manifest_path}")


if __name__ == "__main__":
    main()
