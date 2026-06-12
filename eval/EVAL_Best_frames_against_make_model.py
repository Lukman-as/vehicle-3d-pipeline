import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import json

ROOT = Path(__file__).resolve().parents[1]
path = Path(__file__).resolve().parent / 'hw7_saved_make_model'
all_cams = ['sc1','sc2','sc3','sc4']

all_cams_make_model = dict()
for cam in all_cams:
    with open(path/f'seg23_{cam}_make_model_20240319.json', 'r') as f:
        one_cam = json.load(f)
        all_cams_make_model[cam] = one_cam[list(one_cam.keys())[0]]

# annotator
best_frames = False
if best_frames:
    anno = 'thuy'
else:
    anno = 'aime'

# suffix2 = '_enforce'
angle1 = 'avg'
angle2 = 'pizlo'

#bounds or not
bounds1 = 'bounds'
bounds2 = 'nobounds'

#fixtire or not
fixtire1 = '_fixtirefails'
fixtire2 = ''

fname_1 = f'hw7_results_{anno}_{angle1}_{bounds1}{fixtire2}.txt'

path = ROOT / 'data/gen/matlab_output'
df1 = pd.read_csv(path/fname_1, sep=r'\s+', header=None)

#Name the columns
col_names = ["camera","target", "annotated_car_id",
       "num_sym_pairs","bbox_2D_height",
             "reproj_error","gt_heading_angle",
             "pred_heading_angle","angle_difference",
             "dist_base_gt_bbox","dist_base_pred_bbox",
             "dist_base_bbox_diff","dist_nearest_corner_gt_bbox",
             "dist_nearest_corner_pred_bbox","dist_nearest_corner_diff",
             "iou","iou_bev","mounting_height","ds",
            "PRED_OL","PRED_OW","PRED_OH","PRED_WB","LD_OL","LD_OW","LD_OH","PRED_WWOM","tire_both_sides","has_mirrors","dist_to_move",
            "LD_OW_NON","LD_OH_NON","LD_OL_NON","LENGTH_BY_GAUSSIAN","NUM_TIRES"]
assert len(col_names) == df1.shape[-1]
col_names = ["camera", "target","annotated_car_id"] + [i for i in col_names if (i != "target" and i != "annotated_car_id") and i!= "camera"]
df1.columns = col_names

### Excluding cameras
df1 = df1[(df1.camera != 'lc1')& (df1.camera != 'lc2')]

# dims = ["OH","OL","OW"WWOM]
# def compare_dims(row):
#     for c in dims:
#         row[f"DIFF_{c}"] = row[f"PRED_{c}"]-row[f"LD_{c}"]
#     return row

# df1 = df1.apply(lambda row: compare_dims(row), axis = 1)
# df2 = df2.apply(lambda row: compare_dims(row), axis = 1)
# for c in dims:
#     df1[f"ABS_DIFF_{c}"] = df1[f"DIFF_{c}"].apply(abs)
# for c in dims:
#     df2[f"ABS_DIFF_{c}"] = df2[f"DIFF_{c}"].apply(abs)
    
df1['dist_base_bbox_diff_abs'] = df1['dist_base_bbox_diff'].apply(abs)
df1['dist_nearest_corner_diff_abs'] = df1['dist_nearest_corner_diff'].apply(abs)
df1 = df1[df1.dist_base_gt_bbox <=150 ]

### Refine dimension
refine_dim = False
if refine_dim:
    metric_to_check = 'PRED_OW'
    thres_to_check = 1.56*0.5 #From DB
    print(len(df1[(df1[metric_to_check] < thres_to_check)]), 'violate dimension')
    # print(df1[df1[metric_to_check] < thres_to_check][['target','annotated_car_id','dist_base_gt_bbox','dist_base_bbox_diff_abs',metric_to_check]].sample(n=10))
    df1 = df1[~(df1[metric_to_check] < thres_to_check)]
    metric_to_check = 'PRED_WWOM'
    thres_to_check = 1.56*0.5 #From DB
    print(len(df1[(df1[metric_to_check] < thres_to_check)]), 'violate dimension')
    # print(df1[df1[metric_to_check] < thres_to_check][['target','annotated_car_id','dist_base_gt_bbox','dist_base_bbox_diff_abs',metric_to_check]].sample(n=10))
    df1 = df1[~(df1[metric_to_check] < thres_to_check)]

sort_cols = ['camera','target', 'annotated_car_id']
df1 = df1.sort_values(by=sort_cols).reset_index()
df1.drop('index', axis=1, inplace=True)

# df = df1[df1.PRED_WWOM>df1.PRED_OW]
# print(df.info())
# sdf = df1[(df1.has_mirrors.astype(bool)) & (df1.tire_both_sides)]
# print(np.round(np.mean(sdf.PRED_OW - sdf.PRED_WWOM)/2,3)) #This is assuming the mirror points are theones widest

### PROCESSING TRACK
car_track_lookup = dict()
for cam in all_cams:
    with open(ROOT / f"data/gen/saved_tracks_json/Seg23/{cam}/Seg23_{cam}_20231011.json","r") as f:
        tracks = json.load(f)
    car_to_track = dict()
    for k,v in tracks.items():
        for i in v:
            car_to_track[i] = k
    car_track_lookup[cam] = car_to_track

def get_model(row,lookup_str):
    model = 'NA'
    dims = dict()
    if lookup_str in car_track_lookup[row.camera]:
        track_name = car_track_lookup[row.camera][lookup_str]
        if track_name in all_cams_make_model[row.camera]: #Gotta have gt dimension
            model_detect = all_cams_make_model[row.camera][track_name]
            model = list(model_detect.keys())[0]
    return model

vehicle_classification = {
    'Porsche Macan': 'Vans and SUVs',
    'Hyundai Elantra': 'Sedans',
    'Acura MDX': 'Vans and SUVs',
    'Toyota Camry': 'Sedans',
    'Lexus ES': 'Sedans',
    'Honda Pilot': 'Vans and SUVs',
    'Kia Sportage': 'Vans and SUVs',
    'Ford Edge': 'Vans and SUVs',
    'Mercedes-Benz GLC': 'Vans and SUVs',
    'Nissan Altima': 'Sedans',
    'Tesla Model 3': 'Sedans',
    'Honda CR-V': 'Vans and SUVs',
    'Toyota Matrix': 'Hatchbacks and Station Wagons',
    'Mercedes-Benz C300 4MATIC': 'Sedans',
    'BMW X5': 'Vans and SUVs',
    'Ford Escape': 'Vans and SUVs',
    'Toyota 4Runner': 'Vans and SUVs',
    'Toyota RAV4': 'Vans and SUVs',
    'BMW X3': 'Vans and SUVs',
    'Toyota RAV4 Hybrid': 'Vans and SUVs',
    'BMW X3 xDrive28i': 'Vans and SUVs',
    'Honda Civic': 'Sedans',
    'Mercedes-Benz C-Class': 'Sedans',
    'Toyota Corolla': 'Sedans',
    'Ford Fusion': 'Sedans'
}

def get_gt_dims(row):
    lookup_str = f"{str(int(row.target)).zfill(4)}_{int(row.annotated_car_id-1)}" #Difference between python index 0 and matlab index 1
    if lookup_str in car_track_lookup[row.camera]:
        track_name = car_track_lookup[row.camera][lookup_str]
        if track_name in all_cams_make_model[row.camera]: #Gotta have gt dimension
            model_detect = all_cams_make_model[row.camera][track_name]
            row["model"] = list(model_detect.keys())[0]
            row["class"] = vehicle_classification.get(list(model_detect.keys())[0],'')
            for k,v in model_detect[row.model].items():
                row[f"GT_{k}"] = v/100 #Convert to meter
        row['track_name'] = track_name
    return row

width_used = "WWOM"
use_make_model_width = 'GT_OW'
use_what_lidar = ''
# use_what_lidar = '_NON'
# critical_dims = ["OH","OL",width_used]
critical_dims =  ["OH","OL",width_used,"WB"]

def compare_dims(row, lidar):
    if lidar:
        critical_dims = ["OH","OL",width_used]
        for c in critical_dims:
            if c == "WWOM":
                row[f"DIFF_{c}"] = row[f"LD_OW{use_what_lidar}"]-row[use_make_model_width]
            else:
                row[f"DIFF_{c}"] = row[f"LD_{c}{use_what_lidar}"]-row[f"GT_{c}"]
    else:
        critical_dims = ["OH","OL",width_used,"WB"]
        for c in critical_dims:
            if c == "WWOM":
                # adjusting_for_tw = 0.225
                adjusting_for_tw = 0
                row[f"DIFF_{c}"] = row[f"PRED_{c}"]-adjusting_for_tw-row[use_make_model_width]
            else:
                row[f"DIFF_{c}"] = row[f"PRED_{c}"]-row[f"GT_{c}"]
                
    return row

def eval_diff_percent(row,lidar):
    # if lidar:
    #     critical_dims = ["OH","OL",width_used]
    # else:
    #     critical_dims = ["OH","OL",width_used,"WB"]
    for c in critical_dims:
        if c == "WWOM": #IF use special width 
            row[f"ABS_DIFF_PERCENT_{c}"] = round(row[f"ABS_DIFF_{c}"]/row[use_make_model_width]*100,3)
        else: #Fore everything else
            row[f"ABS_DIFF_PERCENT_{c}"] = round(row[f"ABS_DIFF_{c}"]/row[f"GT_{c}"]*100,3)
    return row

columns_labels_dict = {
    'DIFF_OH':'DIFF Height (m)', 
    'DIFF_OL':'DIFF Length (m)', 
    f'DIFF_{width_used}':'DIFF Width (m)',  
    'DIFF_WB':'DIFF Wheelbase (m)',
    'ABS_DIFF_OH':'MAE Height (m)', 
    'ABS_DIFF_OL':'MAE Length (m)', 
    f'ABS_DIFF_{width_used}':'MAE Width (m)',  
    'ABS_DIFF_WB':'MAE Wheelbase (m)', 
    'ABS_DIFF_PERCENT_OH':'Mean percentage of Absolute Error over groundtruth vehicle’s Height',  
    'ABS_DIFF_PERCENT_OL':'Mean percentage of Absolute Error over groundtruth vehicle’s Length', 
    f'ABS_DIFF_PERCENT_{width_used}':'Mean percentage of Absolute Error over groundtruth vehicle’s Width', 
    'ABS_DIFF_PERCENT_WB':'Mean percentage of Absolute Error over groundtruth vehicle’s Wheelbase', 
    'iou': r'$\mathbf{IoU}$', 
    'dist_base_bbox_diff_abs':'MAE location (m)', 
    'angle_difference':'Mean heading angle error (deg)', 
    'N':'Number of vehicles',
}

def generate_out_df(df):
    out_df = pd.DataFrame(df[metrics].mean()).T
    renamed_cols = [columns_labels_dict[i] for i in out_df.columns]
    out_df.columns = renamed_cols
    out_df['Number of vehicles'] = int(len(df))
    with pd.option_context("max_colwidth", 1000):
        re = out_df.T.to_latex(index=True,
                      formatters={"name": str.upper},
                      float_format="{:.5f}".format,
                      multirow=True,
                      multicolumn=True,
                      multicolumn_format='c',
                      position='h',
                     bold_rows=True
        )
    return re

def get_types(row):
    if row.tire_both_sides:
        type = 1
    elif row.has_mirrors:
        type = 2
    else:
        if row.PRED_WB > 0:
            type = 3
        else:
            type = 4
    row['type'] = type
    return row

metrics_width = [f'DIFF_{width_used}', f'ABS_DIFF_{width_used}', f'ABS_DIFF_PERCENT_{width_used}']
metrics_length = ['DIFF_OL', 'ABS_DIFF_OL', 'ABS_DIFF_PERCENT_OL']
metrics_wb = ['DIFF_WB', 'ABS_DIFF_WB', 'ABS_DIFF_PERCENT_WB']
metrics_height = [ 'DIFF_OH', 'ABS_DIFF_OH', 'ABS_DIFF_PERCENT_OH']

print(len(df1))
lidar= False
# target_df = df1[df1.tire_both_sides.astype(bool)]
target_df = df1
target_df = target_df.apply(get_gt_dims, axis = 1)
target_df = target_df.dropna()
print(len(target_df))
target_df = target_df.apply(get_types, axis = 1)

# type_selected = 1
# target_df = target_df[target_df['type'] == type_selected]
# print('Type selected', type_selected)
# target_df = target_df[target_df.PRED_WB != -1] #Uncomment back when evaluation

target_df = target_df.apply(lambda row: compare_dims(row, lidar=lidar), axis = 1)
for c in critical_dims:
    target_df[f"ABS_DIFF_{c}"] = target_df[f"DIFF_{c}"].apply(abs)
target_df = target_df.apply(lambda row: eval_diff_percent(row, lidar=lidar), axis = 1)
metrics =[f"DIFF_{c}" for c in critical_dims] + [f"ABS_DIFF_{c}" for c in critical_dims] + [f"ABS_DIFF_PERCENT_{c}" for c in critical_dims]
out_df1 = target_df[metrics]
# out_df1 = out_df1[out_df1[f'ABS_DIFF_{width_used}'] <=1]
print(f'LIDAR = {lidar} method\n', generate_out_df(out_df1))  
# out_df1[metrics_length].describe().round(2)[:2]