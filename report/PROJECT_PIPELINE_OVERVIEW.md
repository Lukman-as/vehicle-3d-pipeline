# Symmetry-Based Monocular 3D Vehicle Ground Truthing — Full Pipeline Overview

> **Paper:** *Symmetry-based monocular 3D vehicle ground truthing for traffic analytics* — WACV 2025 (Paper #457)
> **Dataset Paper:** *A Multi-Camera/LiDAR, All-Weather, Day/Night Intersection Traffic Dataset* — WACV 2025 (#1454)
> **Codebase roots:** `3D_GroundTruths/Groundtruth Matlab - Share/` (core algorithm) · `CAD_model_to_sent/` (shape model + renderer)

---

## Table of Contents

1. [Hardware & Sensor Setup](#1-hardware--sensor-setup)
2. [Raw Data Acquisition & Storage](#2-raw-data-acquisition--storage)
3. [Calibration](#3-calibration)
4. [Vehicle Detection & Tracking](#4-vehicle-detection--tracking)
5. [Terrain Model (DTM) Construction](#5-terrain-model-dtm-construction)
6. [Initial Ground-Truth JSON Labels](#6-initial-ground-truth-json-labels)
7. [Human Annotation of Symmetry Points](#7-human-annotation-of-symmetry-points)
8. [3D Inference — Core Algorithm](#8-3d-inference--core-algorithm)
9. [Gaussian Vehicle Shape Prior](#9-gaussian-vehicle-shape-prior)
10. [3D Bounding Box Extraction & Output](#10-3d-bounding-box-extraction--output)
11. [Evaluation Against Ground Truth](#11-evaluation-against-ground-truth)
12. [CAD Parametric Car Model & OpenGL Rendering](#12-cad-parametric-car-model--opengl-rendering)
13. [PyTorch Optimization — Fitting CAD to Point Cloud](#13-pytorch-optimization--fitting-cad-to-point-cloud)
14. [Full File Architecture Map](#14-full-file-architecture-map)

---

## 1. Hardware & Sensor Setup

### 1.1 Physical Location

The dataset was captured at a **major North American intersection** with 8 east-west lanes and 6 north-south lanes, including dedicated bus, bicycle, and pedestrian infrastructure. All sensors were mounted on **10m poles**.

### 1.2 Camera Suite — 6 × AXIS Q1798LE (4K, Global Shutter)

| Camera Group | Count | FOV (H × V) | Purpose |
|---|---|---|---|
| Short-range (SC 1–4) | 4 | 90° × 49° | One per corner, covers intersection |
| Long-range (LC 1–2) | 2 | 21° × 12° | Western approach queue sensing |

- **Resolution:** 3840 × 2160 (4K)
- **Frame rate:** 30 Hz
- **Sensor size:** 4/3"
- **Lens:** Varifocal, large sensor for night vision

### 1.3 LiDAR Suite

| Sensor | Range | FOV | Rate | Role |
|---|---|---|---|---|
| Cepton Vista-P60 (LL) | 200 m | 60° H × 22° V | ~315K pts/s @ 10 Hz | Intersection coverage (SE corner) |
| Ouster OS1-128 (SL) | 90 m | 360° H × 45° V | ~5.2M pts/s @ 10 Hz | Western approach (SW corner) |

### 1.4 Time Synchronisation

- Cameras synced via **local NTP server** → all within **33 ms** of each other
- LiDARs synced via **PTP software** → within **100 ms** of cameras
- Frames timestamped and the first frame of each second used as sync anchor

---

## 2. Raw Data Acquisition & Storage

### 2.1 Recording Sessions

- **19 recordings**, each **10 minutes** at 30 fps camera / 10 fps LiDAR
- Covers: sunny, cloudy, night, dusk/dawn, fog, rain, snow
- Videos **anonymised** (license plates + faces blurred) by third party then human-verified

### 2.2 Segment Extraction for Annotation

- **30 short segments** (26 s each, 20 s annotated → 100 frames per segment at 5 fps)
- **1 long segment** (2 min 20 s → 700 annotated frames)
- Final annotation: **3,694 frames** per sensor; **1.25M 2D** + **250K 3D** object labels

### 2.3 Storage Structure on Disk

```
3D_GroundTruths/to_zip/
    Seg23/
        sc1/   ← short-range camera 1 frames: 0001.png, 0002.png ...
        sc2/
        sc3/
        sc4/
        lc1/   ← long-range camera 1 frames
        lc2/
        lidar_local_gta/   ← LiDAR CSV point clouds per frame
    label/
        Seg23/
            sc1/  ← one JSON per frame: 0001.json, 0002.json ...
            lc1/
            ...
    saved_tracks/
        Seg23/{camera}/Seg23_{camera}_{date}.json  ← vehicle track IDs
    saved_tracks_model_make/
        Seg23/{camera}/{UUID}/   ← make/model lookup per track
```

---

## 3. Calibration

### 3.1 Intrinsic Calibration (Lab)

- Performed using **MATLAB calibration toolbox**
- Short-range cameras: **fisheye model** (wide 90° FOV)
- Long-range cameras: **standard pinhole model**
- Mean reprojection errors: SC1=0.197 px, SC2=0.211 px, SC4=0.178 px, LC1=0.333 px

### 3.2 Extrinsic Calibration (In Situ)

1. **70 tie points** hand-selected on a geo-referenced orthophoto of the intersection
2. 3D coordinates obtained from a **municipal digital terrain model (DTM)** at 0.5 m resolution
3. At least 15 tie points per sensor FOV
4. Camera-to-LiDAR extrinsics computed by matching tie points across sensors
5. Result stored in:
   - `Seg23/extrinsic_calibrations_23.json` — 4×4 transform matrices `T_{camera}_localgta`
   - `intrinsic_calibrations.json` — K matrix per camera

**Key extrinsic transform names (from `hw7_exp.m`):**
```matlab
extrinsics_data = extrinsic_allcams.("T_sc1_localgta");   % camera→local GTA
T_velo_utm      = extrinsic_allcams.("T_localgta_gta");   % local→global
M_cam_velo      = [rot, t; 0 0 0 1];                      % 4×4 homogeneous
```

### 3.3 Video Stabilisation

- Wind-induced camera sway corrected using **SIFT-based optical flow + RANSAC**
- Coarse affine transformation estimated per frame relative to the first frame of each segment
- Each segment independently re-calibrated after stabilisation

---

## 4. Vehicle Detection & Tracking

### 4.1 Automatic Detection & Tracking

- **Detector:** YOLOv8 (Ultralytics)
- **Tracker:** StrongSORT (DeepSORT successor)
- Detections run on the 4K camera frames

### 4.2 Track UUID Assignment

Each unique vehicle instance across frames is given a **persistent UUID** (e.g., `041DB766-2BC6-4BE4-8507-4E445AB1DBA9`). This UUID links the same physical vehicle across:
- Multiple frames within one camera view
- Across different cameras at the same intersection

Track data stored as:
```json
{
  "trackUUID": ["0001_0", "0001_1", "0002_3", ...]
}
```

File: `saved_tracks/Seg23/{cam}/Seg23_{cam}_{date}.json`
→ Maps `frame_carIndex` strings → track UUID

### 4.3 Make/Model Reverse Search

- Google Image reverse search used to identify vehicle make & model for a subset of tracks
- Make/model + CVS database dimensions stored under `saved_tracks_model_make/Seg23/{cam}/{UUID}/`
- Used by evaluation notebooks to get absolute ground-truth dimensions

---

## 5. Terrain Model (DTM) Construction

The terrain model is essential because it provides the **ground-level Z coordinate** for tire-contact points, which is the scale anchor for the entire 3D reconstruction.

### 5.1 DTM Sources

| Dataset | Source |
|---|---|
| Dataset [1] (this project) | Municipal government DTM, 0.5 m resolution |
| DAIR-V2X | LiDAR accumulation (geo-coordinates not provided) |

### 5.2 LiDAR-Based Raster Construction (`hw7_dtm.m`, `get_height_raster.m`)

When a pre-existing DTM is unavailable:
1. Accumulate all LiDAR point clouds across all frames
2. Exclude areas occupied by vehicles (using ground-truth bounding boxes)
3. Apply **k-nearest neighbour interpolation** to fill gaps
4. Output: a per-pixel raster → for each camera pixel (u,v), stores terrain height Z in world coordinates

**Files:**
- `Seg23/lidar_local_gta/*.csv` — raw LiDAR point clouds (X,Y,Z in local GTA frame)
- `23_sc1_final_raster.mat` … `23_lc2_final_raster.mat` — precomputed height rasters per camera
- `23_sc1_roi_maskings.mat` … — masking regions per camera (exclude vehicles/buildings)

### 5.3 Height Kernel (`get_height_kernel_mean_hw7.m`)

For each pixel, the terrain height is estimated as the **mean of k nearest LiDAR neighbours** within the road region mask. The raster is precomputed offline and loaded at inference time.

---

## 6. Initial Ground-Truth JSON Labels

### 6.1 What the JSON Represents

Each file `label/Seg23/{camera}/{frame}.json` is a **frame-level annotation** listing every vehicle visible in that frame. These are the **final output labels** produced after the full pipeline runs.

### 6.2 JSON Schema (one entry per vehicle)

```json
{
  "uuid":           "041DB766-...",        // persistent track ID across frames
  "type":           "Car",                 // Car | Truck | Pedestrian
  "3d_dimensions": {
    "x": 1.92,                             // overall width (metres)
    "y": 4.90,                             // overall length (metres)
    "z": 1.90                              // overall height (metres)
  },
  "3d_location": {
    "x": 345.669,                          // world X in local GTA frame (metres)
    "y": 335.496,                          // world Y
    "z": 203.105                           // world Z (centre elevation; bottom = z - height/2)
  },
  "rotation":  -1.192,                     // heading angle (radians, yaw about vertical)
  "2d_box": {
    "xmin": 930.8,                         // pixel bounding box in this camera frame
    "ymin": 912.3,
    "xmax": 1013.3,
    "ymax": 982.8
  }
}
```

### 6.3 How `read_cars_hw7.m` Reads These

```matlab
% Converts JSON → Matlab struct for each detected object
num = [obj.x2d_box.xmin, obj.x2d_box.ymin, obj.x2d_box.xmax, obj.x2d_box.ymax,
       obj.x3d_dimensions.x, obj.x3d_dimensions.y, obj.x3d_dimensions.z,
       obj.x3d_location.x,   obj.x3d_location.y,
       obj.x3d_location.z - obj.x3d_dimensions.z/2,   % bottom of box
       obj.rotation];
```

---

## 7. Human Annotation of Symmetry Points

### 7.1 Annotation Tool

**For this dataset (HW7):** MATLAB GUI (`annotate_gui.m`)
**For DAIR-V2X:** CVAT open-source annotation platform

Both tools present the car image with its 2D bounding box overlaid. An annotator then clicks specific physical landmarks on the vehicle.

### 7.2 Annotation Preparation (`PREPARE data for annotators` notebook)

1. Loads tracking JSONs to get all car UUIDs in the dataset
2. For each car track, selects the best frame (least occlusion, good viewing angle)
3. Crops the car region and packages it for annotators
4. Output: per-car image crops + metadata with camera, frame, car index

### 7.3 Four Classes of Annotation Points

| Class | Description | Required? |
|---|---|---|
| **Tire-ground contact point** | Image projection of the point where the outer tyre edge contacts the road. Labelled: PF (passenger front), DF (driver front), DR (driver rear), PR (passenger rear) | At least ONE required |
| **Symmetry point pair** | A pair of left/right mirror-symmetric points on the vehicle body (e.g., corners of windshield, headlight centres, roof corners). Annotator draws a line from driver side → passenger side | Minimum 1 pair |
| **Extremal symmetry point pair** | A symmetry pair that aligns with the LATERAL outer edge of the tyres — used to locate the symmetry plane when only one-sided tyre contacts are available | Required if only one-sided tyres |
| **Side-mirror point pair** | Left and right corners of side-view mirrors — used to estimate overall width including mirrors | Optional but preferred |

### 7.4 Annotation GUI Workflow (`annotate_gui.m`)

```
Load image → Draw 2D bounding box in red
    ↓
For each of 14 click positions:
    roi = drawpoint('Color','r');                   % Matlab interactive point
    annotations(:, 14*(k-1)+j) = [x, y]';          % store pixel coordinates
    addlistener(roi, 'ROIMoved', @move_points_callback)  % allow dragging
    ↓
"Visual check" button → scatter all clicked points
    ↓
"Finished?" button → commit annotations
    ↓
Validity check: discard points outside 2D bounding box
    ↓
Save to annotations matrix
```

**Annotation time:** ~120 seconds per vehicle on average.

### 7.5 Output Format

A 2×14N matrix of pixel coordinates (2 rows = x,y; 14 columns per car × N cars). These pixel coordinates are the raw input to the 3D inference algorithm.

---

## 8. 3D Inference — Core Algorithm

This is the mathematical heart of the project. The goal is: given annotated 2D pixel coordinates of symmetric point pairs and tyre-ground contacts, recover the full 3D bounding box of the vehicle.

### 8.1 Coordinate Frame Convention

- **Right-handed world frame**, Y-axis aligned with gravity pointing DOWN
- Origin at the camera optical centre
- Projection equation:

```
λx = K·R·X
```

Where:
- `x` = homogeneous image point (3×1)
- `X` = 3D world point (3×1)
- `K` = intrinsic matrix (3×3)
- `R` = camera rotation matrix (3×3)
- `λ` = non-zero scale factor

### 8.2 Step 1 — Horizon Line Computation

The **horizon line** `l̃ₕ` is the projection of the infinite horizontal plane:

```
ṽₓ = K·R₁        (vanishing point of world X-axis)
ṽᵤ = K·R₃        (vanishing point of world Z-axis)
l̃ₕ = ṽₓ × ṽᵤ    (cross product in homogeneous coordinates)
```

**File:** Computed inside `hw7_exp.m` and used by `get_vpcar*.m`

### 8.3 Step 2 — Symmetry Plane Orientation (Vanishing Point Estimation)

Each annotated symmetry point pair `p'ᵢ = {p'ᵢₗ, p'ᵢᵣ}` defines a **symmetry line** in the image:

```
l̃'ᵢ = p̃'ᵢₗ × p̃'ᵢᵣ     (cross product of homogeneous point coords)
```

For a perfect symmetric vehicle with zero annotation error, ALL symmetry lines would meet at one **vanishing point `ṽₛ`** on the horizon. In practice, each line intersects the horizon at a noisy point:

```
ṽ'ᵢ = l̃'ᵢ × l̃ₕ
```

**Vanishing point estimation** uses the method of Pizlo et al. — find `ṽ*ₛ` on `l̃ₕ` that minimises the sum of squared distances to all symmetry lines:

```
ṽ*ₛ = argmin Σᵢ dist(l̃'ᵢ, ṽ*ₛ)²
```

This is implemented as a **least-squares line intersection** (`LSPointLines.m`).

The **unit normal to the symmetry plane** follows:

```
Nₛ = (K·R)⁻¹ · ṽ*ₛ
N̄ₛ = Nₛ / |Nₛ|           (normalised)
```

**Files:** `get_vpcar_ransac.m`, `get_vpcar_avg_intersects.m`, `get_vpcar_collins_weiss.m`, `LSPointLines.m`, `get_sym_normal.m`

### 8.4 Step 3 — Symmetry Plane Location

#### Scenario A: Tyre contacts on BOTH sides

1. For each annotated tyre contact `t'ᵢ` on the LEFT side, compute its 3D location using the terrain raster:

```
T'ᵢ = λₜ · (K·R)⁻¹ · t̄'ᵢ
```

where λₜ is solved by intersecting the ray through `t'ᵢ` with the pre-computed terrain height at that pixel.

2. Compute mean left tyre position `T'ₗ` and mean right tyre position `T'ᵣ`

3. Left extremal plane: `M̃ₗ = (aₛ, bₛ, cₛ, d̄ₗ)` where `(aₛ,bₛ,cₛ) = N̄ₛ` and `d̄ₗ` solved by `T̄'ₗ · M̃ₗ = 0`

4. Similarly for right plane `M̃ᵣ`

5. **Symmetry plane** midpoint:
```
d̄ₛ = (d̄ₗ + d̄ᵣ) / 2
```

6. **Vehicle width:**
```
w = |d̄ₗ - d̄ᵣ|
```

#### Scenario B: Tyre contacts on ONE side only

Uses **extremal symmetry pairs** to locate the hidden side. For each extremal pair `p*ᵢ = {p*ᵢₗ, p*ᵢᵣ}`:

Left 3D point from known extremal plane:
```
P*ᵢₗ(λ) = λ · (K·R)⁻¹ · [xᵖ*ᵢₗ, yᵖ*ᵢₗ, 1]ᵀ
λ solved by: P*ᵢₗ · M̃ₗ = 0
```

Right 3D point (on unknown plane at distance w):
```
P*ᵢᵣ = P*ᵢₗ + w · N̄ₛ
```

w is found by minimising distance from P*ᵢᵣ to the optical ray through the annotated right point. Closed-form solution using line-line distance:

```
N̂*ᵣ = D̂*'ᵢᵣ × (Nₛ × D̂*'ᵢᵣ)

w = - (P*ᵢₗᵀ · N̂*ᵣ) / (Nₛᵀ · N̂*ᵣ)
```

**Files:** `sym_points_3D.m`, `get_cMw.m`, `get_cMw_v2.m`, `center_points_3D.m`

### 8.5 Step 4 — 3D Locations of Non-Extremal Symmetry Point Pairs

For each non-extremal annotated pair `p'ᵢ = {p'ᵢₗ, p'ᵢᵣ}`:

1. **Rectify** the symmetry line to pass exactly through the estimated vanishing point `vₛ`:

```
p̄'ᵢ = centroid of p'ᵢₗ and p'ᵢᵣ

pᵢₗ = p̄'ᵢ + [(p'ᵢₗ - p̄'ᵢ)ᵀ(vₛ - p̄'ᵢ) / |vₛ - p̄'ᵢ|²] · (vₛ - p̄'ᵢ)
pᵢᵣ = same formula for right point
```

2. **Back-project** to 3D using two constraints:
   - The line `PᵢₗPᵢᵣ` must be normal to the symmetry plane
   - The midpoint of `Pᵢₗ` and `Pᵢᵣ` must lie ON the symmetry plane

```
Pᵢₗ = λₗ · (K·R)⁻¹ · pᵢₗ
Pᵢᵣ = λᵣ · (K·R)⁻¹ · pᵢᵣ
```

This yields a system solved for λₗ and λᵣ algebraically.

### 8.6 Step 5 — Levenberg-Marquardt Refinement

The algebraic estimate is refined by minimising the **total reprojection error** over all annotated points using the Levenberg-Marquardt non-linear least squares algorithm:

```
E = Σᵢ ||p'ᵢ - pᵢ||²
```

Where `p'ᵢ` is the annotated pixel and `pᵢ` is the re-projection of the estimated 3D point.

**Three free parameters adjusted:**
1. Orientation and position of the symmetry plane + extremal plane offsets
2. 2D position and length of each symmetry line
3. 1D position of each tyre-ground contact along the extremal plane/terrain intersection

**Four hard constraints:**
1. Tyre-ground contact points must remain ON the ground terrain
2. Left/right extremal planes remain parallel to and equally displaced from the symmetry plane
3. Tyre contacts and extremal symmetry points must remain on their respective extremal planes
4. Non-extremal symmetry pairs must remain on their symmetry lines, equally displaced from the symmetry plane

**Files:** `coop_fun_lsq.m`, `sym_3D_fun.m`, `sym_3D_con.m`, `lm_con.m`, `costFunction.m`, `do_opt_hw7.m`

### 8.7 Heading Angle Estimation

Multiple methods implemented and compared:

| Method | File | Description |
|---|---|---|
| Average intersects | `get_vpcar_avg_intersects.m` | Mean of per-pair horizon intersections |
| RANSAC | `get_vpcar_ransac.m` | Robust estimation, rejects outlier pairs |
| Collins-Weiss | `get_vpcar_collins_weiss.m` | Algebraic line-fitting on horizon |
| Pizlo | `get_vpcar_pizlo.m` | Based on Pizlo et al. 2012 formulation |
| Gaussian VP | `get_vpcar_gauss1.m` | Gaussian weighted intersection |
| Ground truth | `get_vpcar_gt.m` | Uses LiDAR GT heading as reference |

Heading angle is then converted:
```matlab
% From vanishing point to heading angle
yaw = heading_from_normal(N_sym_plane, extrinsics)
```

**File:** `heading_from_normal.m`, `get_angle_error.m`, `eval_angle.m`

---

## 9. Gaussian Vehicle Shape Prior

### 9.1 Purpose

When only a few symmetry points are visible (e.g., near-parallel view), the 3D point cloud may not constrain ALL vehicle dimensions. The **Canadian Vehicle Specifications (CVS) database** is used as a statistical prior to fill in missing dimensions.

### 9.2 The CVS Database

- **11,707 vehicles** sold in Canada, 2011–2023
- Covers: passenger cars, vans, SUVs, hatchbacks, station wagons, full-size pickups
- **Excludes:** buses, motorcycles, long-haul trucks
- Parameters captured (all in mm in raw CSV, converted to cm/m in code):

| Symbol | Meaning |
|---|---|
| OL | Overall Length |
| OW | Overall Width |
| OH | Overall Height |
| WB | Wheelbase |
| A | Front hood length (front end to windshield base) |
| C | Window height |
| D | Door-to-window height gap |
| E | Roof width |
| FH (F) | Front overhang |
| RH (G) | Rear overhang |
| TWF | Track width front |
| TWR | Track width rear |
| TW | Mean track width = (TWF + TWR) / 2 |

### 9.3 Pearson Correlations (why the prior works)

From `predict_a_from_b.py` and the paper (Table 1):

| | OH | OW | WB | RH | FH |
|---|---|---|---|---|---|
| OH | 1.00 | 0.59 | 0.76 | 0.61 | 0.43 |
| OW | 0.59 | 1.00 | 0.74 | 0.60 | 0.56 |
| WB | 0.76 | 0.74 | 1.00 | 0.70 | 0.49 |

Strong correlations mean: if you measure OW and WB from the image, you can predict OH, FH, RH with reasonable accuracy.

### 9.4 Multivariate Gaussian Model — Mathematics

All parameters modelled as **jointly Gaussian**. Partition into observed (`b`) and unobserved (`a`) parts:

```
[a]   ~  N( [μₐ],  [Σₐₐ  Σₐᵦ] )
[b]         [μᵦ]   [Σᵦₐ  Σᵦᵦ]
```

**Conditional distribution** of `a` given observed `b`:

```
a | b ~ N(μₐ|ᵦ, Σₐₐ|ᵦ)

μₐ|ᵦ = μₐ + Σₐᵦ · Σᵦᵦ⁻¹ · (b - μᵦ)

Σₐₐ|ᵦ = Σₐₐ - Σₐᵦ · Σᵦᵦ⁻¹ · Σᵦₐ
```

This is implemented as the MAP (maximum a posteriori) estimate = conditional mean `μₐ|ᵦ`.

### 9.5 Implementation

**Python side (`predict_a_from_b.py`):**
```python
def estimate_ab(a_part, b_part, df):
    mu_a = np.mean(sdf[a_part].values, 0)
    mu_b = np.mean(sdf[b_part].values, 0)
    cov_all = np.cov(sdf.values.T)
    cov_bb_inv = np.linalg.inv(cov_bb)
    cov_ab = cov_all[:len(a_part), len(a_part):]
    return mu_a, cov_ab, cov_bb_inv, mu_b

# Prediction:
a_predicted = mu_a + cov_ab @ cov_bb_inv @ (b_vector - mu_b)
```

**Matlab side (`gaussian_pred.m`):**
```matlab
b_pred = (mu_a' + cov_ab * cov_bb_inv * (b_vector' - mu_b')) / 100;
```

### 9.6 Conditioning Scenarios (4 variants trained)

| Observed (b) | Predicted (a) | Use case |
|---|---|---|
| `[OW, WB]` | `[RH, FH, OH]` | Width + wheelbase visible |
| `[OW]` | `[RH, FH, OH, WB]` | Only width visible |
| `[WB]` | `[OW]` | Only wheelbase visible |
| `[TW]` | `[OW]` | Only track width visible |

### 9.7 Serialised Model Files

- `gaussian_model_OW_WB.npz` — conditioned on OW + WB (2 observable dimensions)
- `gaussian_model_OW_WB_OH_OL.npz` — conditioned on OW + WB + OH + OL (4 observables, used in renderer)

Both store: `{mu_a, cov_ab, cov_bb_inv, mu_b, a_part, b_part}`

### 9.8 Dimension Bounding (Hard Constraints from CVS)

To prevent ill-conditioned estimates from producing physically impossible results:

| Dimension | Maximum (m) |
|---|---|
| Width including mirrors | 3.53 |
| Height | 3.05 |
| Front Overhang | 1.10 |
| Wheelbase | 4.47 |
| Rear Overhang | 2.02 |

**Files:** `get_bbox_from_gaussian_model.m`, `car_model_param_202405.mat`, `get_soft_constraints_gaussian.m`

### 9.9 Width Estimation Logic (hierarchy by available annotations)

| Mirrors | LR tyre contacts | FB tyre contacts | Method |
|---|---|---|---|
| ✗ | ✗ | ✗ | Extremal symmetry pairs |
| ✗ | ✗ | ✓ | Wheelbase → CVS model |
| ✗ | ✓ | any | Tyre-ground contact distance |
| ✓ | ✗ | ✗ | Mirrors: OW = w_mirrors - 0.25m |
| ✓ | ✗ | ✓ | Wheelbase → CVS model |
| ✓ | ✓ | any | Mirrors |

---

## 10. 3D Bounding Box Extraction & Output

### 10.1 3D Box Construction (`get_3d_bbox_hw7.m`)

Given predicted L, W, H, position (x,y,z), and heading angle `rz`:

**Rotation matrix** around vertical axis:
```matlab
R_bounding = [cos(rz)  -sin(rz)  0;
              sin(rz)   cos(rz)  0;
              0         0        1];
```

**8 corners** (relative to centre):
```matlab
corners = [
  -l/2,  l/2,  l/2, -l/2, -l/2,  l/2,  l/2, -l/2;   % X (length axis)
   w/2,  w/2, -w/2, -w/2,  w/2,  w/2, -w/2, -w/2;   % Y (width axis)
   0,    0,    0,    0,    h,    h,    h,    h         % Z (height)
];
corners_world = R_bounding * corners + [x; y; z];
```

### 10.2 3D Metrics Computed Per Vehicle

| Field | File | Description |
|---|---|---|
| `iou` | `get_iou_3D.m` | 3D IoU against LiDAR GT box |
| `iou_bev` | `get_iou_bev.m` | Bird's-eye-view IoU |
| `angle_difference` | `eval_angle.m` | Heading angle error (degrees) |
| `dist_base_bbox_diff` | `eval_dist_sym_plane.m` | Distance error at bounding box base |
| `dist_nearest_corner_diff` | | Nearest-corner location error |
| `reproj_error` | `get_reproj_error.m` | Mean reprojection error of symmetry points (pixels) |
| `pred_width_complete_box` | | Predicted OW |
| `pred_length_complete_box` | | Predicted OL |
| `pred_height_complete_box` | | Predicted OH |
| `wheelbase` | `save_3D_points_right_position.m` | Distance between front and rear tyre 3D positions |
| `pred_width_wo_mirrors` | | OW excluding mirror contribution (WWOM) |
| `dist_to_move` | | Mirror overhang correction constant |
| `gt_width/height/length` | from LiDAR labels | LiDAR-derived ground truth |

### 10.3 Output File Format (`write_results_hw7.m` → `sample.txt`)

Each row = one vehicle. 35 space-separated fields:

```
camera  target  car_id  num_sym_pairs  bbox_2D_height
reproj_error  gt_heading  pred_heading  angle_diff
dist_base_gt  dist_base_pred  dist_base_diff
dist_corner_gt  dist_corner_pred  dist_corner_diff
iou  iou_bev  mounting_height  ds
PRED_OL  PRED_OW  PRED_OH  PRED_WB
LD_OL  LD_OW  LD_OH
PRED_WWOM  tire_both_sides  has_mirrors  dist_to_move
LD_OW_NON  LD_OH_NON  LD_OL_NON
LENGTH_BY_GAUSSIAN  NUM_TIRES
```

Example row from `sample.txt`:
```
sc1 0035 16 7 147 2.02 -160.40 -160.24 0.15 35.36 35.06 -0.30 ... 4.76 2.30 1.62 2.69 ...
```

### 10.4 Main Experiment Orchestration (`hw7_exp.m` / `hw7_exp_official.m`)

```
Load frame image
    ↓
Load camera intrinsics (intrinsic_calibrations.json)
Load camera extrinsics (extrinsic_calibrations_23.json)
    ↓
Read car labels from JSON (read_cars_hw7.m)
    ↓
For each detected car:
    Load terrain height raster
    Get vanishing point / heading angle
    Compute symmetry plane orientation & location
    Back-project all symmetry + tyre points to 3D
    Run LM optimisation (sym_3D_fun.m, sym_3D_con.m)
    Apply Gaussian prior for missing dimensions
    Compute bounding box corners
    Compare against LiDAR GT
    Write result row (write_results_hw7.m)
```

---

## 11. Evaluation Against Ground Truth

### 11.1 Against LiDAR Ground Truth (HW7 EVAL notebooks)

**File:** `HW7 EVAL - Compare against groundtruth make and model.ipynb`

Pipeline:
1. Load `sample.txt` results → DataFrame (35 columns)
2. Look up each car's track UUID via `saved_tracks` JSON
3. Look up make/model dimensions from `saved_tracks_model_make` → CVS DB
4. Compute errors:
   ```python
   DIFF_OH = PRED_OH - GT_OH
   ABS_DIFF_OH = |DIFF_OH|
   ABS_DIFF_PERCENT_OH = ABS_DIFF_OH / GT_OH * 100
   ```
5. Output LaTeX tables for the paper

**Vehicle categories (4 types) used in analysis:**
- Type 1: Tyre contacts on both sides visible
- Type 2: Mirror points visible
- Type 3: Wheelbase visible (front+back tyres)
- Type 4: Only one tyre visible (least constrained)

### 11.2 Consistency Over Time

**File:** `HW7 EVAL - Consistency within a track.ipynb`

For each car track with ≥ 2 annotated frames separated by > 3m displacement:
- Compute standard deviation of predicted OL, OW, OH across frames
- Report Mean Standard Deviation (MSD) and Mean Percent Standard Deviation (MPSD)

**Results (Table 1 of supplementary):**
| Method | Height MPSD | Length MPSD | Width MPSD |
|---|---|---|---|
| LiDAR (single frame) | 39.4% | 41.5% | 54.1% |
| Symmetry (ours) | 5.7% | 7.0% | 4.5% |
| LiDAR (propagated) | 0.0% | 0.0% | 0.0% |

### 11.3 Key Performance Numbers (from Table 6 of main paper)

Dataset [1] (this codebase), N=512 vehicles:

| Method | Height MAE% | Length MAE% | Width MAE% |
|---|---|---|---|
| Single-frame LiDAR | 38.86% | 32.32% | 53.37% |
| Propagated LiDAR | 4.68% | 2.12% | 4.39% |
| **Our symmetry method** | **6.65%** | **5.54%** | **3.49%** |

**Key finding:** Symmetry method vastly outperforms single-frame LiDAR and is comparable to multi-frame propagated LiDAR.

---

## 12. CAD Parametric Car Model & OpenGL Rendering

### 12.1 Purpose

After the Matlab pipeline outputs predicted dimensions (OW, OH, OL, WB), the Python renderer:
1. Builds a **parametric CAD car model** from those dimensions
2. Renders it in **OpenGL** alongside the real LiDAR point cloud
3. Visually verifies that the predicted shape fits the observed points

### 12.2 The 18-Vertex Parametric Car Body

The car body is defined by 18 key vertices derived from 10 parameters:
`[A, C, D, E, F, G, OH, OL, OW, TW]`

| Parameter | Meaning |
|---|---|
| A | Front hood length |
| C | Window height |
| D | Door-to-window height gap |
| E | Roof width |
| F | Front overhang (tyre to front face) |
| G | Rear overhang (tyre to rear face) |
| OH | Overall height |
| OL | Overall length |
| OW | Overall width |
| TW | Mean track width |

**Vertex definitions** (in `render_car.py` and `optimize_render_car.py`):

```python
# Windshield base corners (front face, lower)
v1 = (anchor[0] + (OW-E)/2,     anchor[1] + (OH-C-D),      anchor[2])
v2 = (anchor[0] + (OW-E)/2,     anchor[1] + (OH-C-D*(1/5)),anchor[2])
v3 = (anchor[0] + OW-(OW-E)/2,  anchor[1] + (OH-C-D*(1/5)),anchor[2])
v4 = (anchor[0] + OW-(OW-E)/2,  anchor[1] + (OH-C-D),      anchor[2])

# A-pillar base (front face, upper)
v5 = (anchor[0],    anchor[1] + (OH-C-D),   anchor[2] - A*(1/5))
v6 = (anchor[0],    anchor[1] + (OH-C),     anchor[2] - A*(1/5))
v7 = (anchor[0]+OW, anchor[1] + (OH-C),     anchor[2] - A*(1/5))
v8 = (anchor[0]+OW, anchor[1] + (OH-C-D),   anchor[2] - A*(1/5))

# Windshield top (roof transition)
v9  = (anchor[0],    anchor[1] + (OH-C), anchor[2] - A)
v10 = (anchor[0]+OW, anchor[1] + (OH-C), anchor[2] - A)

# Roof corners (front + rear)
v11-v14 = roof panel at z = -A - (OL-A)*(2/10) to -(OL-(OL-A)*(1/10))

# Rear face
v15-v18 at z = -OL
```

**14 quad faces** rendered (windshield, hood, roof, trunk, sides, rear, etc.)
**4 cylinder tyres** rendered at positions derived from F (front), G (rear), TW (track width)

### 12.3 Tyre Cylinder Rendering

```python
def renderCylinder(self, color, center, radius, height):
    tire_radius = 43.18  # cm (standard ~17" tyre)
    # Two circular end-caps (GL_TRIANGLES) + side band (GL_QUAD_STRIP)
    for theta in range(0, 360, 15):
        arc_start = np.deg2rad(theta)
        arc_end   = np.deg2rad(theta + 15)
        # Fan triangles for circular face
        # Quad strip for cylindrical side
```

### 12.4 Conditional Parameter Prediction in Renderer

Both `render_car.py` and `optimize_render_car.py` call the Gaussian model to get ALL 10 parameters from the 4 observable ones:

```python
def get_conditional_params(self, pred_ow, pred_wb, pred_ol, pred_oh):
    mu_a, cov_ab, cov_bb_inv, mu_b, _, b_part = load_gaussian_model(
        "gaussian_model_OW_WB_OH_OL.npz")
    b_vector = [pred_ow, pred_wb, pred_oh, pred_ol]
    params = mu_a + cov_ab @ cov_bb_inv @ (b_vector - mu_b)
    # params → [A, C, D, E, FH, RH, TW]
    # Direct assignments:
    result[6] = pred_oh   # OH direct
    result[7] = pred_ol   # OL direct
    result[8] = pred_ow   # OW direct
```

### 12.5 OpenGL Scene Setup

```python
glClearColor(0, 0, 0, 1)
glEnable(GL_DEPTH_TEST)
glEnable(GL_LIGHTING)
glEnable(GL_LIGHT0)
glLightfv(GL_LIGHT0, GL_POSITION, [100.0, 1000.0, -1000.0, 0.5])
glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1.0, 1.0, 1.0, 1.0])
gluPerspective(45.0, aspect_ratio, 0.1, 7000.0)  # FOV, aspect, near, far
```

**Coordinate system for rendering (different from world frame):**
```python
# Original point cloud: x=length, y=height(down), z=width
# Rendering:           x=width,  y=height(up),   z=-length
transformed[:, 0] = -points[:, 2]   # x = -z_original (width)
transformed[:, 1] = -points[:, 1]   # y = -y_original (flip up)
transformed[:, 2] = -points[:, 0]   # z = -x_original (length away)
points *= 100                         # convert m → cm
```

**User interaction:**
- Mouse drag → rotate view (`rotX`, `rotY`)
- Arrow keys → pan (`x`, `y` translation)
- Scroll wheel → zoom (`z` translation)

---

## 13. PyTorch Optimization — Fitting CAD to Point Cloud

### 13.1 Purpose

`optimize_render_car.py` automatically aligns the CAD model to the observed 3D point cloud using gradient descent.

### 13.2 Distance Functions

**Point-to-quad distance** (`ud_quad`) — smooth differentiable distance from a 3D point to a quadrilateral face:

```python
def ud_quad(p, a, b, c, d, eps=1e-5):
    # ba, cb, dc, ad = quad edges
    nor = cross(ba, ad)         # face normal
    s = sign(dot(cross(ba,nor),pa)) + sign(dot(cross(cb,nor),pb)) + ...
    # If point projects inside quad → planar distance
    # Else → minimum edge distance
    plane_d2 = (dot(nor,pa))² / (|nor|² + eps)
    use_edges = s < 3.0
    return sqrt(min(edge_dists, plane_d2) + eps)
```

**Point-to-cylinder distance** (for tyres) — projects to ground contact point:

```python
def distance_to_cylinder(point, center, radius, height, axis):
    bottom_center = center.clone()
    bottom_center[1] = 0           # project to ground (y=0)
    bottom_center += (height/2) * axis
    radial_dist = torch.norm(point - bottom_center)
    return torch.abs(radial_dist)
```

### 13.3 Optimisation Loop (`optimize_fit` method)

**Objective:** Minimise mean squared distance from each observed 3D point to its nearest surface of the CAD model.

```python
translation = torch.tensor(dis, requires_grad=True, dtype=torch.float)
angle       = torch.tensor(heading, requires_grad=True, dtype=torch.float)

optimizer = torch.optim.RMSprop([translation, angle], lr=0.8, momentum=0.5)

for i in range(50):
    optimizer.zero_grad()
    
    # Build rotation matrix (Y-axis rotation by angle)
    c, s = cos(deg2rad(angle)), sin(deg2rad(angle))
    rot = [[c,0,s],[0,1,0],[-s,0,c]]
    
    for p in points:
        p_t = rot.T @ (p - translation)   # transform point to car frame
        
        if p[y] > 0.5:    # body point (above ground)
            md = min over all quad faces: ud_quad(p_t, *face_verts)
        else:             # tyre/ground point
            md = min over tyres: distance_to_cylinder(p_t, ...) * 5
        
    loss = mean(|min_dists|²)
    loss.backward()
    optimizer.step()

# Extract final pose
self.dist_to_move = tuple(translation.detach().numpy())
self.pred_heading_angle = angle.detach().item()
```

**Why RMSprop?** Adaptive learning rate handles the different scales of translation (cm) vs rotation (degrees). Momentum=0.5 prevents oscillation.

### 13.4 Two-Stage Initialisation

1. **Coarse initialisation:** `dis = mean(point_cloud) - center_car_model`
2. **Fine alignment:** 50 iterations of RMSprop

---

## 14. Full File Architecture Map

### `CAD_model_to_sent/` — Python Shape Model + Renderer

| File | Role | Key Input | Key Output |
|---|---|---|---|
| `predict_a_from_b.py` | Train Gaussian shape model + generate correlation heatmap | `cars_database/{year}_en.csv` | `gaussian_model_*.npz`, `correlation_heatmap.png` |
| `get_mean_car.py` | Compute mean car dimensions 2011–2023 | `cars_database/{year}_en.csv` | `mean_car_shape_2011_2023.csv` |
| `gaussian_model_OW_WB.npz` | Serialised Gaussian (2 conditionals) | — | Used by Matlab `gaussian_pred.m` |
| `gaussian_model_OW_WB_OH_OL.npz` | Serialised Gaussian (4 conditionals) | — | Used by Python renderers |
| `mean_car_shape_2011_2023.csv` | Single-row mean car shape | — | Used by `render_car.py` |
| `render_car.py` | Interactive OpenGL viewer (no optimisation) | `sample.txt`, `points.csv` | Visual display |
| `optimize_render_car.py` | Interactive OpenGL viewer + PyTorch fitting | `sample.txt`, `points.csv` | Fitted 3D car overlay |
| `visualize.py` | Simple Pygame point cloud viewer | `points.csv` | Rotating point cloud display |
| `gaussian.ipynb` | Exploration notebook for Gaussian model | car DB CSVs | plots/analysis |
| `points.csv` / `points1.csv` / `points2.csv` | LiDAR 3D points for test cars (metres) | — | Input to renderers |
| `sample.txt` | Pipeline output for 3 test cars | — | Input to renderers |
| `requirements.txt` | Python dependencies | — | — |
| `correlation_heatmap.png` | Pearson correlation matrix visualisation | — | Paper figure |
| `loss_vs_iteration.png` | Optimisation convergence plot | — | Debugging output |

---

### `3D_GroundTruths/Groundtruth Matlab - Share/` — Core Algorithm

#### Entry Points / Experiment Scripts
| File | Role |
|---|---|
| `hw7_exp.m` | Main experiment script for HW7 dataset — orchestrates full pipeline |
| `hw7_exp_official.m` | Official paper results version |
| `run_demo.m` / `run_demov2/v3/v4.m` | Progressive demo iterations |
| `do_opt_hw7.m` | Runs optimisation for HW7 dataset |
| `annotate_gui.m` | Interactive Matlab GUI for human annotation |

#### Calibration & Camera
| File | Role |
|---|---|
| `read_calib.m` | Load intrinsic + extrinsic JSON |
| `decompose_projection.m`, `Pdecomp.m`, `rq.m` | Decompose projection matrix → K, R, t |
| `check_k_matrix.m` | Validate K matrix properties |
| `check_ex_sym.m` | Check extrinsic symmetry consistency |
| `intrinsic_calibrations.json` | Per-camera K matrices |
| `Seg23/extrinsic_calibrations_23.json` | Per-camera T matrices |

#### Vanishing Point / Heading
| File | Role |
|---|---|
| `get_vpcar.m` | Main VP estimation dispatcher |
| `get_vpcar_ransac.m` | RANSAC-based VP from symmetry lines |
| `get_vpcar_avg_intersects.m` / `_v2.m` | Average intersections method |
| `get_vpcar_collins_weiss.m` | Collins-Weiss algebraic method |
| `get_vpcar_pizlo.m` | Pizlo 2012 method |
| `get_vpcar_gauss1.m` | Gaussian-weighted VP |
| `LSPointLines.m`, `LSPointLinesTest.m` | Least-squares point-on-lines solver |
| `heading_from_normal.m` | Convert symmetry plane normal → heading angle |
| `vp_car.m`, `vpcar.m` | Vanishing point car-specific utilities |
| `get_vppoints.m` | Get vanishing points from calibration |
| `findVP.m` | Find vanishing point from line set |

#### Symmetry & 3D Reconstruction
| File | Role |
|---|---|
| `sym_points_3D.m` | Main function: annotated 2D → 3D point cloud |
| `center_points_3D.m` | Centre 3D points on symmetry plane |
| `get_cMw.m` / `_v2.m` / `_v3.m` | Camera-to-world transform from symmetry |
| `get_cMw_tire.m` | Camera-to-world using tyre contacts |
| `get_sym_normal.m` | Symmetry plane normal from VP |
| `flip_sym_normal_vpcar.m` | Handle front/rear orientation ambiguity |
| `project_onto_sym.m` | Project point onto symmetry plane |
| `reconcile_using_mirror.m` | Use mirror points to refine width |
| `get_rotation_from_two_vecs.m` | Rotation matrix between two direction vectors |
| `get_world2car_rotation.m` | World → car coordinate rotation |

#### Optimisation
| File | Role |
|---|---|
| `coop_fun_lsq.m` | Main objective function for LM optimiser |
| `sym_3D_fun.m` | Symmetry constraint function |
| `sym_3D_con.m` | Non-linear constraints for optimiser |
| `costFunction.m` | Total cost function (reprojection + priors) |
| `lm_con.m` | Levenberg-Marquardt constraint wrapper |
| `lsqFun.m` | Least-squares function wrapper |
| `myfun.m` / `mycon.m` | fmincon function/constraint handles |
| `vpcar_mle_fun.m` / `_mle_con.m` | MLE vanishing point estimation |

#### Gaussian Prior (Matlab side)
| File | Role |
|---|---|
| `gaussian_pred.m` | Predict unobserved dims from observed dims |
| `car_model_param_202405.mat` | Serialised Gaussian model params for Matlab |
| `get_bbox_from_gaussian_model.m` | Apply Gaussian prior → complete bounding box |
| `get_soft_constraints_gaussian.m` | Soft penalisation of out-of-range dimensions |

#### Bounding Box & Geometry
| File | Role |
|---|---|
| `get_3d_bbox_hw7.m` | Construct 3D bbox from predicted params |
| `get_bbox_world_hw7.m` | Transform bbox to world coordinates |
| `get_3d_bbox.m` / `get_3d_bbox_a9.m` | Dataset-specific bbox variants |
| `get_3Dpoints_car_from_x.m` | Extract 3D corner points from parameter vector |
| `get_3d_after.m` | Post-refinement bbox extraction |
| `get_extremal_3D.m` / `get_non_extremal.m` | Extremal/non-extremal symmetry point handling |
| `get_refined_back.m` / `_tg.m` | Refine back face / tyre-ground positions |
| `get_obj_height.m` | Compute object height from 3D points |
| `get_base_bbox.m` | Get base (bottom) bbox from 3D box |
| `get_bbox_minimum.m` / `minBoundingBox.m` | Minimum 2D bounding box in BEV |
| `extend_a_point.m` | Extend point along direction |

#### Terrain & Height
| File | Role |
|---|---|
| `hw7_dtm.m` | Build Digital Terrain Model for HW7 |
| `get_height_raster.m` | Compute per-pixel height raster from LiDAR |
| `get_height_kernel_mean_hw7.m` | KNN-based height estimation |
| `get_height_grid.m` | Grid-based height interpolation |
| `compare_terrain.m` | Compare LiDAR terrain vs reference |
| `localize_planes.m` | Localise road planes |
| `computePlane.m` / `fitPlane.m` / `createPlane.m` | Plane fitting utilities |
| `hw7_proof.m` | Terrain model verification |
| `23_*_final_raster.mat` | Precomputed height rasters (6 cameras) |
| `23_*_roi_maskings.mat` | Region-of-interest masks (6 cameras) |

#### Evaluation
| File | Role |
|---|---|
| `eval_angle.m` / `eval_angle_coop.m` | Heading angle error |
| `eval_dist_sym_plane.m` | Symmetry plane location error |
| `eval_height.m` / `eval_length.m` / `eval_width.m` | Dimensional MAE |
| `eval_wheelbase.m` | Wheelbase error |
| `eval_iou.m` | 3D IoU evaluation |
| `eval_position.m` | 3D position error |
| `get_iou_3D.m` / `get_iou_bev.m` | IoU computation functions |
| `get_angle_error.m` / `_v2.m` | Angular error computation |
| `get_distance_error.m` / `_nearest.m` | Position error computation |
| `get_height_error.m` / `get_length_error.m` / `get_width_error.m` | Per-dimension error |
| `get_reproj_error.m` | Reprojection error in pixels |
| `write_results_hw7.m` | Format and write result rows to .txt |

#### Data I/O
| File | Role |
|---|---|
| `read_cars_hw7.m` | Load JSON ground-truth labels per frame |
| `read_annotations_rope.m` | Load symmetry annotations (rope dataset) |
| `save_3D_points_right_position.m` | Store 3D point results in annotations struct |
| `save_extracted.m` / `save_extracted_pre.m` | Save intermediate extraction results |
| `get_predicted_labels.m` | Retrieve label predictions |

#### Notebooks (Python, in `HW7 Dataset/` and `Groundtruth Matlab - Share/`)
| File | Role |
|---|---|
| `PREPARE data for annotators - Get cars from tracking HW7.ipynb` | Extract per-car crops for annotation |
| `HW7 EVAL - Compare against groundtruth make and model.ipynb` | Main evaluation: predicted vs CVS GT |
| `HW7 EVAL - Consistency within a track.ipynb` | Intra-track stability analysis |
| `EVAL - Car dimensions model Multivariate and get GT for car make model.ipynb` | Exploratory Gaussian model eval |

---

## End-to-End Pipeline Summary

```
[HARDWARE]
6×4K Cameras + 2×LiDAR (10m poles)
    │ 30fps video + 10fps point clouds
    │ NTP/PTP sync
    ▼

[FRAME EXTRACTION & ANONYMISATION]
19×10min recordings → 3,694 frames per sensor
Faces + plates blurred → to_zip/Seg23/{camera}/{frame}.png
    │
    ▼

[CALIBRATION DATA]
Intrinsics (K per camera) + Extrinsics (T camera→world)
→ intrinsic_calibrations.json
→ Seg23/extrinsic_calibrations_23.json
    │
    ▼

[VEHICLE DETECTION & TRACKING]
YOLOv8 → bounding boxes per frame
StrongSORT → persistent UUIDs across frames
→ saved_tracks/Seg23/{cam}/*.json
    │
    ▼

[TERRAIN MODEL]
LiDAR CSVs (lidar_local_gta/*.csv)
+ hw7_dtm.m + get_height_raster.m
→ 23_{cam}_final_raster.mat  (per-pixel world Z)
→ 23_{cam}_roi_maskings.mat
    │
    ▼

[INITIAL LABELS → JSON]
3D bounding boxes from multi-frame LiDAR propagation
→ label/Seg23/{camera}/{frame}.json
  {uuid, type, 3d_dimensions, 3d_location, rotation, 2d_box}
    │
    ▼

[HUMAN ANNOTATION of SYMMETRY POINTS]
PREPARE notebook → extracts car crops
annotate_gui.m → annotator clicks 14 points per car:
  - Symmetry pairs (windshield corners, headlights, roof, rear lights)
  - Extremal pairs (outer tyre width)
  - Tyre-ground contact points (PF, DF, DR, PR)
  - Mirror pairs
~120 sec/vehicle
→ annotations matrix (2 × 14N pixel coordinates)
    │
    ▼

[3D INFERENCE — MATLAB CORE]
hw7_exp.m / do_opt_hw7.m

  Step 1: Horizon line from camera extrinsics
  Step 2: Vanishing point estimation (RANSAC / avg-intersects)
          → Symmetry plane ORIENTATION (unit normal N̄ₛ)
  Step 3: Terrain raster lookup → tyre 3D positions
          → Symmetry plane LOCATION (d̄ₛ, width w)
  Step 4: Algebraic back-projection → 3D point cloud
  Step 5: Levenberg-Marquardt refinement (minimise reprojection error)
          Constraints: ground, parallel planes, extremal planes, symmetry lines

  + Gaussian prior (gaussian_pred.m + car_model_param_202405.mat):
    [OW, WB] → predict [OH, FH, RH]
    cov_ab @ cov_bb_inv @ (b - mu_b) + mu_a
    Hard bounds from CVS DB (max OH=3.05m, max WB=4.47m ...)

  get_3d_bbox_hw7.m → 8 corner coordinates
  Compute: IoU, heading error, position error, dimension MAE
  write_results_hw7.m → sample.txt (35 fields per vehicle)
    │
    ▼

[EVALUATION]
HW7 EVAL notebooks:
  sample.txt + saved_tracks_model_make → GT dims from CVS DB
  DIFF_OH = PRED_OH - GT_OH
  MAE, % error, LaTeX table output
  Track consistency: std dev across frames
    │
    ▼

[CAD RENDERING — PYTHON]
optimize_render_car.py

  Input: sample.txt (PRED_OW, PRED_OL, PRED_OH, PRED_WB)
         points.csv (LiDAR 3D point cloud, metres)

  1. Load Gaussian model → get all 10 params [A,C,D,E,F,G,OH,OL,OW,TW]
  2. Build parametric 18-vertex car body + 4 cylinder tyres
  3. Coarse initialisation: translate CAD centre → point cloud mean

  PyTorch optimisation (50 iterations, RMSprop lr=0.8):
    loss = mean(||min distance from each point to nearest CAD surface||²)
    ud_quad() for body faces (smooth differentiable distance)
    distance_to_cylinder() for tyre faces
    Gradients w.r.t. translation (3D) + heading angle (1D)

  4. Apply optimised pose → render aligned CAD + point cloud
  5. OpenGL interactive display:
     Blue metallic body quads (with Phong lighting)
     Black tyre cylinders
     Red point cloud (observed LiDAR)
     Checkerboard ground plane
```

---

*Document generated from: WACV 2025 papers #457 and #1454, all Matlab source files in `3D_GroundTruths/Groundtruth Matlab - Share/`, and all Python files in `CAD_model_to_sent/`.*
