from pathlib import Path
import json
from tqdm import tqdm
import pandas as pd
import numpy as np
import shutil
import cv2
from collections import namedtuple

Rectangle = namedtuple('Rectangle', 'xmin ymin xmax ymax')

def overlap_area(a, b):  # returns None if rectangles don't intersect
    dx = min(a.xmax, b.xmax) - max(a.xmin, b.xmin)
    dy = min(a.ymax, b.ymax) - max(a.ymin, b.ymin)
    if (dx>=0) and (dy>=0):
        return dx*dy
    return 0

def union_area(a, b):  # returns None if rectangles don't intersect
    dx = max(a.xmax, b.xmax) - min(a.xmin, b.xmin)
    dy = max(a.ymax, b.ymax) - min(a.ymin, b.ymin)
    return dx*dy

ROOT = Path(__file__).resolve().parents[1]
path = ROOT / 'data/raw'
output_path = ROOT / 'data/gen'
segment = "23"
cameras = ["sc1", "sc2", "sc3", "sc4"]
output_img_path = Path(output_path/"saved_tracks_image")
output_img_path.mkdir(parents=True, exist_ok=True)

for camera in tqdm(cameras, desc="Camera"):
    label_files = sorted(
        (item for item in Path(path/f"label/Seg{segment}/{camera}").iterdir() if item.is_file()),
        key=lambda item: int(item.stem),
    )

    #Determine the unocculed cars based on looping through all bounding box
    unoccluded_car = set()
    for item in label_files:
        #Get the bounding box to filter out too small box first
        label = json.load(open(item,'r'))
        target = str(item).split("/")[-1].split(".")[0]
        for i,h in enumerate(label): #Looping through vehicles in label
            if h['type']  == 'Car' or h['type']  == 'Truck':
                ra = Rectangle(h['2d_box']['xmin'], h['2d_box']['ymin'], h['2d_box']['xmax'], h['2d_box']['ymax'])
                unoccluded = True
                for j,k in enumerate(label): #Looping through vehicles in label
                    if i!=j and k['type']  == 'Car' or k['type']  == 'Truck':
                        rb = Rectangle(k['2d_box']['xmin'], k['2d_box']['ymin'], k['2d_box']['xmax'], k['2d_box']['ymax'])
                        overlap = overlap_area(ra, rb)
                        if overlap > 0:
                            unoccluded = False
                            break
                if unoccluded:
                    unoccluded_car.add(f"{target}_{i}")

    print('Unoccluded cars:', len(unoccluded_car))

    img = cv2.imread(str(path/f"image/Seg{segment}/{camera}/0001.png"))
    y_size = img.shape[0]
    x_size = img.shape[1]
    # print('Image size:', x_size,y_size)

    ## Setup
    pad = 100 #Padding near the edge of image
    thres = 50 #Image has to be at least this size in x_size and y_size
    distance_thres = 3 #Cars have to be spaced from each other

    # cat_dict = {"E":0, "M":0,"H1":0,"H2":0}
    good_tracks_to_save = dict() 
    for item in tqdm(label_files):
        all_cars_in_track_ids = []
        #Get the bounding box to filter out too small box first
        label = json.load(open(item,'r'))
        target = str(item).split("/")[-1].split(".")[0]
        for obj_id, h in enumerate(label): #Looping through vehicles in label
            if (h['type']  == 'Car' or h['type']  == 'Truck') and f"{target}_{obj_id}" in unoccluded_car:
                if h['2d_box']['ymin'] >= pad and h['2d_box']['ymax'] <= y_size-pad and \
                h['2d_box']['xmin'] >= pad and h['2d_box']['xmax'] <=x_size-pad:
                    x_range = h['2d_box']['xmax']-h['2d_box']['xmin']
                    y_range = h['2d_box']['ymax']-h['2d_box']['ymin']
                    if(x_range < thres and y_range <thres): #Filter out too small cars but still within region of interest
                        continue
                    #Check location first
                    already_have_track = h['uuid'] in good_tracks_to_save
                    current_loc = np.array([h["3d_location"]["x"], h["3d_location"]["y"]])
                    if already_have_track:
                        prev_loc = good_tracks_to_save[h['uuid']][-1][-1] #Extract previous loclation
                        dist = np.linalg.norm(current_loc-prev_loc) #check dist with location
                        if dist < distance_thres: #Too close or too far
    #                             print(f"{target}_{obj_id} skipped for {h['uuid']}")
                            continue
                    #Select this car
                    selected_vehicle = f"{target}_{obj_id}"
                    all_cars_in_track_ids.append(selected_vehicle)
                    if already_have_track:
                        good_tracks_to_save[h['uuid']].append([selected_vehicle,current_loc])
                    else:
                        good_tracks_to_save[h['uuid']] = [[selected_vehicle,current_loc]]
                                        
            #Tally the cat #Need to adjust by doing translation to camera position first NO NEED - tally later
    #         for car in all_cars_in_track_ids:
    #             target = car.split("_")[0]
    #             obj_id = car.split("_")[1]
    #             label = json.load(open(path/f"label/Seg{segment}/{camera}/{target}.json",'r'))
    #             aloc = [float(label[int(obj_id)]['3d_location']['x']), float(label[int(obj_id)]['3d_location']['y'])]
    #             dist = np.linalg.norm(np.array(aloc))
    #             print(dist)
    #             if dist <= 50:
    #                 cat_dict["E"] += 1
    #             elif dist <= 50:
    #                 cat_dict["M"] += 1
    #             elif dist <= 100:
    #                 cat_dict["H1"] += 1
    #             else:
    #                 cat_dict["H2"] += 1 

    unique_cars = set()
    track_export = dict()
    for k,v in good_tracks_to_save.items():
        if len(v) >= 2: #Got to have at least 2 cars in a track
            # print(str(v[0][0]).split("\\")[-1])
            ordered_list = sorted([str(i[0]).split("\\")[-1] for i in v], key = lambda x:int(x.split("_")[0])) #Sort by frame number
            track_export[k] = ordered_list
            unique_cars.update(ordered_list)

    unique_cars = list(unique_cars)
    print('Unique cars in camera', camera, '=', len(unique_cars))

    buffer = 0.15
    for track_id, cars in track_export.items():
        for car in cars:
            target = car.split("_")[0]
            obj_id = car.split("_")[1]
            #Get the bounding box to filter out too small box first
            label = json.load(open(path/f"label/Seg{segment}/{camera}/{target}.json",'r'))
            h = label[int(obj_id)]
            x_range = h['2d_box']['xmax']-h['2d_box']['xmin']
            y_range = h['2d_box']['ymax']-h['2d_box']['ymin']    
            img = cv2.imread(str(path/f"image/Seg{segment}/{camera}/{target}.png"))
            x_max_buff = int(h['2d_box']['xmax']+buffer*x_range)
            x_min_buff = int(h['2d_box']['xmin']-buffer*x_range)
            y_max_buff = int(h['2d_box']['ymax']+buffer*y_range)
            y_min_buff = int(h['2d_box']['ymin']-buffer*y_range)
            a = max(0, y_min_buff)
            b = min(y_size,y_max_buff)
            c = max(0, x_min_buff)
            d = min(x_size,x_max_buff)
            cropped_image = img[a:b, c:d]
            text = '_'.join([str(w) for w in [a,b,c,d]])
            Path(output_img_path/f'Seg{segment}/{camera}/{track_id}').mkdir(parents=True, exist_ok=True)
            cv2.imwrite(f"{str(output_img_path)}/Seg{segment}/{camera}/{track_id}/{car}.jpg", cropped_image, [cv2.IMWRITE_JPEG_QUALITY, 100])

    #Save the tracking information
    output_json_path = Path(output_path/f"saved_tracks_json/Seg{segment}/{camera}")
    output_json_path.mkdir(parents=True, exist_ok=True)
    with open(f"{str(output_json_path)}/Seg{segment}_{camera}_202411.json","w") as f:
        json.dump(track_export,f)
