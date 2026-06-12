# vehicle-3d-pipeline

Symmetry-based monocular 3D vehicle groundtruthing (WACV 2025 method — see
`report/papers/`) plus vehicle-type fitting and 3D rendering.

From a single highway camera frame, the pipeline reconstructs each annotated
car in 3D: its dimensions (length / width / height / wheelbase), heading and
position, a sparse 3D "symmetry point cloud", and finally a rendered 3D car
model fitted over that point cloud.

```
 raw frames + tracking labels
        │
        ▼
 [1] annotation/prepare_data_for_annotation.py   → car crops for the annotators
        │
        ▼
 [2] CVAT manual annotation                      → data/raw/anno/annotations.xml
        │
        ▼
 [3] annotation/process_annotation.py            → data/gen/formatted_anno/<batch>/
        │
        ▼
 [4] matlab/hw7_exp_official.m                   → data/gen/matlab_output/
        │                                            hw7_results_*.txt  (dimensions per car)
        │                                            point_cloud/<cam>/<frame>.json
        ▼
 [5] rendering/*.py  (via render_io.py)          → interactive 3D fit on screen
 [6] rendering/make_comparisons.py               → comparisons/index.html
                                                    (annotated image vs 3D render, side by side)
```

A car is identified everywhere by the triple **(camera, frame, obj_id)** —
called `(camera, target, annotated_car_id)` in the results file.

---

## Setup

Python (3.10+):

```bash
pip install -r requirements.txt
```

MATLAB: any recent release; stage [4] also needs no toolboxes beyond what the
`.m` files in `matlab/` use. **Run MATLAB with `matlab/` as the current
folder** — all paths in `hw7_exp_official.m` are relative to it.

---

## Quick start (outputs are already included)

The repo ships with the processed annotations and MATLAB outputs for batch
`23_short_cams_thuy_20250329` (Seg23, cameras sc1–sc4, 111 cars), so you can
see results immediately without running MATLAB:

```bash
# Interactive 3D fit of one car (photoreal model stretched to predicted dims):
cd rendering
python rendertooriginal.py --cam sc1 --frame 0035 --obj 16

# Or by row number of the results file:
python rendertooriginal.py 5

# Side-by-side sheets for ALL cars -> ../comparisons/index.html:
python make_comparisons.py
open ../comparisons/index.html
```

Renderer variants (all share the same CLI via `render_io.py`):

| script | what the 3D car looks like |
|---|---|
| `rendertooriginal.py` | original photoreal `.glb` model, rigidly scaled to predicted OW/OH/OL |
| `optimize_render_car_multimodel.py` | Kenney low-poly kit, wheels relocated to the fitted wheelbase/track |
| `originalrender.py` | parametric polygon car driven by all 11 fitted dimensions |
| `optimize_render_car.py` | single-template fit (the original optimizer) |

Each fits all four type templates (sedan / suv / truck / van) to the point
cloud, conditioned on Gaussian dimension priors, and keeps the best.
Mouse-drag rotates, arrows/W/S move the camera.

---

## Running the full pipeline

### [1] Prepare crops for annotation  *(only for a new annotation round)*

Needs the **full** raw frames (see Data storage below).

```bash
python annotation/prepare_data_for_annotation.py
# writes data/gen/saved_tracks_image/   (crops to upload to CVAT)
#        data/gen/saved_tracks_json/    (track membership per camera)
```

### [2] Annotate in CVAT

Per car: 4 tire-contact points (DF/PF/DR/PR), center points, and symmetry
line pairs labelled *Extremal*, *Non-extremal* or *Mirror* (each drawn
driver-side → passenger-side). Export as CVAT-XML to
`data/raw/anno/annotations.xml`.

### [3] Validate + format the annotation

```bash
python annotation/process_annotation.py
# writes data/gen/formatted_anno/<batch>/<cam>/annotation/<frame>.json
```

Set `annotation_folder` at the top of the script to name the new batch, and
`official_task_ids` / `task_to_camera` to match the CVAT task ids.

### [4] MATLAB 3D reconstruction  →  point clouds + dimensions

Open MATLAB **in the `matlab/` folder** and run:

```matlab
hw7_exp_official
```

Inputs it reads (all relative): `../data/gen/formatted_anno/<batch>/`,
`../data/raw/image/Seg23/<cam>/<frame>.png`, intrinsics/extrinsics, labels,
the terrain raster in `raster/`, and LiDAR ground truth in
`../data/raw/image/Seg23/lidar_local_gta/`.

Outputs (these feed the renderers directly):

```
data/gen/matlab_output/hw7_results_<annotator>_avg_bounds.txt   # 1 row per car, 35 cols
data/gen/matlab_output/hw7_fails_<annotator>_avg_bounds.txt     # rejected cars + reason
data/gen/matlab_output/point_cloud/<cam>/<frame>.json           # [{obj_id, points Nx3}, ...]
```

The `annotator` / `date` variables at the top of the script select the batch
(`23_short_cams_<annotator>_<date>` must exist under `data/gen/formatted_anno/`).

### [5] Render the 3D fit

```bash
cd rendering
python rendertooriginal.py --cam sc1 --frame 0035 --obj 16
```

`render_io.py` reads the MATLAB outputs directly — no manual copying of
points/results between stages.

### [6] Compare annotation vs 3D result

```bash
python rendering/make_comparisons.py            # all cars
python rendering/make_comparisons.py --gl       # right panel = real GL render
open comparisons/index.html
```

### Evaluation against make/model ground truth (optional)

`eval/` compares predicted dimensions against spec-sheet dimensions of the
reverse-image-searched make/model of each car:

```bash
python eval/EVAL_Best_frames_against_make_model_save_outputs.py
# writes data/gen/eval_outputs/
```

Dimension priors themselves are rebuilt with the notebooks in `priors/`
(`gaussian_per_vehicle.ipynb` → `priors/models/*.npz`) from the
`priors/cars_database/` spec CSVs (2011–2023).

---

## Repository layout

```
annotation/        stages 1+3 (Python)
matlab/            stage 4 — symmetry-based 3D reconstruction (+ raster/ camera rasters)
priors/            cars database CSVs, Gaussian dimension priors (notebooks + .npz)
rendering/         stage 5+6 — render_io bridge, 4 renderers, make_comparisons, models/ (.glb)
eval/              make/model evaluation scripts + notebooks
report/            LaTeX report, design doc PDF, WACV papers
data/raw/          inputs:  image/ label/ calib/ anno/ terrain/
data/gen/          generated: formatted_anno/ matlab_output/ saved_tracks_json/ ...
comparisons/       output of make_comparisons.py (gitignored)
```

## Data storage policy

Large raw data lives in **Google Drive (`LURA_data/`)**; the repo keeps only
what the pipeline actually reads:

| data | local | git | Drive |
|---|---|---|---|
| annotated frames (~94 PNGs, 1.1 GB) | yes | no (gitignored) | yes (full set) |
| full Seg23 frames (600 PNGs, 6.7 GB) | no | no | yes |
| LiDAR csvs (215 MB) | yes | no | yes |
| terrain rasters (82 MB) | yes | no | yes |
| labels, calib, annotations.xml | yes | yes | yes |
| all `data/gen/` outputs (a few MB) | yes | yes | — |

To work on frames that are not local (e.g. the older
`23_all_cams_aime_20240301` batch, or a new annotation round): download
`LURA_data/HW7_raw_data/Seg23/<cam>/<frame>.png` from Google Drive into
`data/raw/image/Seg23/<cam>/`.

## Notes / known quirks

- `rendering/car_models_core_new.py` is a newer fork of `car_models_core.py`
  used only by `originalrender.py`; the other renderers use the original.
  They should eventually be merged.
- `rendering/render_car.py` and `visualize.py` are early prototypes kept for
  reference; `rendering/samples/` holds the legacy hand-extracted inputs that
  `_test_core.py` (headless fit sanity check) still uses.
- The older annotation batch `23_all_cams_aime_20240301` (cameras lc1/lc2 +
  sc1–sc4) is included in `data/gen/formatted_anno/`, and its results file
  `hw7_results_aime_avg_bounds.txt` is in `data/gen/matlab_output/`, but its
  raw frames are only in the Drive archive.
- `eval/get_multivariate_model_and_make_model.py` expects reverse-search
  spreadsheets exported to `data/raw/make_model_data/<seg_cam>.csv`; that
  folder is produced manually from the reverse-image-search workflow
  (`eval/reverse_search_track_model/`).
- Other `hw7_*.m` variants in `matlab/` (`hw7_exp.m`, `hw7_debug.m`,
  `hw7_proof.m`) are older experiments with legacy absolute paths;
  `hw7_exp_official.m` is the maintained entry point.
