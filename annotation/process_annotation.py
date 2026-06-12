from pathlib import Path
import xmltodict
from tqdm import tqdm
import numpy as np
import json
np.set_printoptions(suppress=True)
import pickle
import cv2
import json
import csv


def dump_dict_to_csv(data, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for key, value in data.items():
            writer.writerow([key, value])

def get_corners(segment, camera, car):
    target = car.split("_")[0]
    obj_id = car.split("_")[1]
    #Get the bounding box to filter out too small box first
    label = json.load(open(path/f"label/Seg{segment}/{camera}/{target}.json",'r'))
    try:
        h = label[int(obj_id)]
    except:
        print(camera)
        print(target)
        print(obj_id)
        import sys
        sys.exit(1)
    x_range = h['2d_box']['xmax']-h['2d_box']['xmin']
    y_range = h['2d_box']['ymax']-h['2d_box']['ymin']    
    x_max_buff = int(h['2d_box']['xmax']+buffer*x_range)
    x_min_buff = int(h['2d_box']['xmin']-buffer*x_range)
    y_max_buff = int(h['2d_box']['ymax']+buffer*y_range)
    y_min_buff = int(h['2d_box']['ymin']-buffer*y_range)
    img = cv2.imread(str(path/f"image/Seg{segment}/{camera}/0001.png"))
    y_size = img.shape[0]
    x_size = img.shape[1]
    a = max(0, y_min_buff)
    b = min(y_size,y_max_buff)
    c = max(0, x_min_buff)
    d = min(x_size,x_max_buff)
    text = '_'.join([str(w) for w in [a,b,c,d]])
    return text

def get_anno_fullname(obj):
    camera = task_to_camera[obj['@task_id']]
    car = obj['@name'].split(".")[0]
    text = get_corners(segment, camera, car)
    fullname = f"Seg{segment}_{camera}-{car}-{text}.jpg"
    return fullname

def print_anno_result():
    print(count_all_cars)
    avg_sym = np.mean(num_sym_lines)
    print("Avg number of symmetry lines per valid car", avg_sym)
    print('Max',max(num_sym_lines))
    print('Min',min(num_sym_lines))
    # assert(avg_sym >= 4)
    print('Portion of valid cars', len(all_valid_cars_set)/count_all_cars)
    avg_center = count_center_points/len(all_valid_cars_set)
    print('Avg number of center points per valid car', avg_center) 
    # assert(avg_center >= 1)
    avg_tires = count_tires/len(all_valid_cars_set)
    print('Avg number of tires per valid car', avg_tires) 
    assert(avg_tires >= 1.5)

    for k,v in all_invalid_cars.items():
        print(k,v)

    print(len(all_invalid_cars))
    print(len(all_valid_cars_set))

### Get the annotation out
def process_one_object(obj, name):
    #Convert coordinate all points
    re = dict()
    re['obj_id'] = int(name.split("-")[1].split("_")[1])+1 #Account for matlab difference
    re['annotations'] = {
        'tire_points':{
            'DF': [-1,-1],
            'PF': [-1,-1],
            'DR': [-1,-1],
            'PR': [-1,-1],
        },
        "extremal_pairs" : [],
        "non_extremal_pairs" : [],
        "center_points" : [],
        "has_mirror": 0
    }
    corners = name.split("-")[-1].split("_")
    upper_leftx = int(corners[2]) 
    upper_lefty = int(corners[0])
#     print("POINTS",obj['points'])
    if not isinstance(obj['points'], dict): #For one pair only
        for p in obj['points']:
            try:
                x,y = p['@points'].split(",")
            except:
                print("Fail split")
                continue
            x = round(float(x) + upper_leftx,2)
            y =  round(float(y) + upper_lefty,2) #Shiffting annotation
            if p['@label'] == 'Tire contact':
                if p['attribute']['#text'] == "Passenger back":
                    re['annotations']['tire_points']['PR'] = [x,y];
                elif p['attribute']['#text'] == "Driver back":
                    re['annotations']['tire_points']['DR'] = [x,y];
                elif p['attribute']['#text'] == "Passenger front":
                    re['annotations']['tire_points']['PF'] = [x,y];
                elif p['attribute']['#text'] == "Driver front":
                    re['annotations']['tire_points']['DF'] = [x,y];
            elif p['@label'] == 'Center points':
                re['annotations']['center_points'].append([x,y])
    else:
        p = obj['points']
        x,y = p['@points'].split(",")
        x = round(float(x) + upper_leftx,2)
        y =  round(float(y) + upper_lefty,2) #Shiffting annotation
        if p['@label'] == 'Tire contact':
            if p['attribute']['#text'] == "Passenger back":
                re['annotations']['tire_points']['PR'] = [x,y];
            elif p['attribute']['#text'] == "Driver back":
                re['annotations']['tire_points']['DR'] = [x,y];
            elif p['attribute']['#text'] == "Passenger front":
                re['annotations']['tire_points']['PF'] = [x,y];
            elif p['attribute']['#text'] == "Driver front":
                re['annotations']['tire_points']['DF'] = [x,y];
        elif p['@label'] == 'Center points':
            re['annotations']['center_points'].append([x,y])
        
    if not isinstance(obj['polyline'], dict): #For one pair only
        for l in obj["polyline"]:
            p1,p2 = l['@points'].split(';')
            p1x, p1y = p1.split(',')
            p2x, p2y = p2.split(',')
            p1x = round(float(p1x) + upper_leftx,2)
            p1y = round(float(p1y) + upper_lefty,2)
            p2x = round(float(p2x) + upper_leftx,2)
            p2y = round(float(p2y) + upper_lefty,2)
            if l['attribute']['#text'] == "Non-extremal":
                re['annotations']['non_extremal_pairs'].append([p1x,p1y,p2x,p2y])
            elif l['attribute']['#text'] == "Extremal" and len(re['annotations']['extremal_pairs']) == 0:
                re['annotations']['extremal_pairs'].append([p1x,p1y,p2x,p2y])
            elif l['attribute']['#text'] == "Mirror":
                re['annotations']['has_mirror'] = 1
                mirror_pairs = [p1x,p1y,p2x,p2y]
    else:
        l = obj['polyline']
        p1,p2 = l['@points'].split(';')
        p1x, p1y = p1.split(',')
        p2x, p2y = p2.split(',')
        p1x = round(float(p1x) + upper_leftx,2)
        p1y = round(float(p1y) + upper_lefty,2)
        p2x = round(float(p2x) + upper_leftx,2)
        p2y = round(float(p2y) + upper_lefty,2)
        if l['attribute']['#text'] == "Non-extremal":
            re['annotations']['non_extremal_pairs'].append([p1x,p1y,p2x,p2y])
        elif l['attribute']['#text'] == "Extremal" and len(re['annotations']['extremal_pairs']) == 0:
            re['annotations']['extremal_pairs'].append([p1x,p1y,p2x,p2y])
        elif l['attribute']['#text'] == "Mirror":
            re['annotations']['has_mirror'] = 1
            mirror_pairs = [p1x,p1y,p2x,p2y]
    if re['annotations']['has_mirror']:
        re['annotations']['non_extremal_pairs'].append(mirror_pairs) #Ensure always append to the end
    return re


ROOT = Path(__file__).resolve().parents[1]
path = ROOT / "data/raw"
output_path = ROOT / "data/gen"
annotation_path = path / "anno"
annotation_folder = "23_short_cams_thuy_20250329"
with open(annotation_path/f'annotations.xml') as fd:
    anno = xmltodict.parse(fd.read())

segment = "23"
buffer = 0.15
official_task_ids = ["224", "221", "222", "223"]
task_to_camera = {
        "224": "sc1",
        "221": "sc2",
        "222": "sc3",
        "223": "sc4",
    }

num_sym_lines = []
count_extremal = 0
count_mirror = 0
all_valid_cars_set = set()
all_invalid_cars = dict()
count_all_cars = 0
count_center_points = 0
count_tires = 0
for i in tqdm(anno['annotations']['image']):
    if i['@task_id'] not in official_task_ids:
        continue
    name = get_anno_fullname(i)
    target = name.split("-")[0]
    corners = name.split("-")[-1].split("_")
    upper_leftx = int(corners[2]) 
    upper_lefty = int(corners[0])
    if True: #No more intersection
        all_sym_pairs = []
        count_all_cars += 1
        extremal_exist = False
        nonextremal_exist = False
        problematic_sym_pair = []
        problematic_tire = []
        problematic_center = []
        all_tires_annotated = []
        num_tires = 0
        num_centers = 0
        if 'polyline' in i and len(i['polyline']) > 0: #Need this to be valid
            num_mirror = 0
            if not isinstance(i['polyline'], dict): #For one pair only
                for k in i['polyline']:
                    pstring = k['@points']
                    if len(pstring.split(";"))!=2:
                        problematic_sym_pair.append(k)
                        continue
                    pl_x = float(pstring.split(";")[0].split(',')[0])
                    pl_y = float(pstring.split(";")[0].split(',')[1])
                    pr_x = float(pstring.split(";")[1].split(',')[0])
                    pr_y = float(pstring.split(";")[1].split(',')[1])
                    pl_x = round(pl_x + upper_leftx,2)
                    pl_y = round(pl_y + upper_lefty,2)
                    pr_x = round(pr_x + upper_leftx,2)
                    pr_y = round(pr_y + upper_lefty,2)
                    all_sym_pairs.append([pl_x,pl_y,pr_x,pr_y])
                    if isinstance(k,dict) and k['attribute']['#text'] == 'Non-extremal':
                        nonextremal_exist = True
                    if isinstance(k,dict) and k['attribute']['#text'] == 'Extremal':
                        extremal_exist = True
                    if isinstance(k,dict) and k['attribute']['#text'] == 'Mirror':
                        num_mirror+=1
            else:
                    k = i['polyline']
                    pstring = k['@points']
                    if len(pstring.split(";"))!=2:
                        problematic_sym_pair.append(k)
                    else:
                        pl_x = float(pstring.split(";")[0].split(',')[0])
                        pl_y = float(pstring.split(";")[0].split(',')[1])
                        pr_x = float(pstring.split(";")[1].split(',')[0])
                        pr_y = float(pstring.split(";")[1].split(',')[1])
                        pl_x = round(pl_x + upper_leftx,2)
                        pl_y = round(pl_y + upper_lefty,2)
                        pr_x = round(pr_x + upper_leftx,2)
                        pr_y = round(pr_y + upper_lefty,2)
                        all_sym_pairs.append([pl_x,pl_y,pr_x,pr_y])
                        if isinstance(k,dict) and k['attribute']['#text'] == 'Non-extremal':
                            nonextremal_exist = True
                        if isinstance(k,dict) and k['attribute']['#text'] == 'Extremal':
                            extremal_exist = True
                        if isinstance(k,dict) and k['attribute']['#text'] == 'Mirror':
                            num_mirror+=1
        if 'points' in i and len(i['points']) > 0:
            if not isinstance(i['points'], dict): #For one pair only
                for p in i['points']:
                    if p['@label'] == 'Center points':
                        if len(p['@points'].split(",")) != 2:
                            problematic_center.append(p)
                            continue
                        num_centers += 1

                    elif p['@label'] == 'Tire contact':
                        if len(p['@points'].split(",")) != 2  or p['attribute']['#text'] in all_tires_annotated:
                            problematic_tire.append(p)
                        else:
                            all_tires_annotated.append(p['attribute']['#text'])
                            num_tires += 1
            else:
                p = i['points']
                if p['@label'] == 'Center points':
                    if len(p['@points'].split(",")) != 2:
                        problematic_center.append(p)
                    else:
                        num_centers += 1

                elif p['@label'] == 'Tire contact':
                    if len(p['@points'].split(",")) != 2:
                        problematic_tire.append(p)
                    else:
                        all_tires_annotated.append(p['attribute']['#text'])
                        num_tires += 1
        #Check sym_order
        if len(all_sym_pairs) > 0:
            if len(problematic_sym_pair) == 0 and len(problematic_tire) == 0 and num_tires > 0:
                sym_order_good = True; #Pending sym order for now
                if sym_order_good and extremal_exist and (num_mirror <= 1) and nonextremal_exist:       
                    all_valid_cars_set.add(name)
                    count_mirror+=num_mirror #Add mirror when valid only
                    count_center_points += num_centers
                    count_tires += num_tires
                    num_sym_lines.append(len(i['polyline'])) #Add sym lines when valid only
                if not sym_order_good:
                    if name in all_invalid_cars:
                        all_invalid_cars[f"task_{i['@task_id']}_{name}"].append('Order of sym pairs not from Driver to Passenger')
                    else:
                        all_invalid_cars[f"task_{i['@task_id']}_{name}"] = ['Order of sym pairs not from Driver to Passenger']

#             if num_centers == 0:
#                 str_to_put = f"No center points annotated"
#                 if name in all_invalid_cars:
#                     all_invalid_cars[f"task_{i['@task_id']}_{name}"].append(str_to_put)
#                 else:
#                     all_invalid_cars[f"task_{i['@task_id']}_{name}"] = [str_to_put]
                    
            if num_tires == 0:
                str_to_put = f"No tire points annotated"
                if name in all_invalid_cars:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"].append(str_to_put)
                else:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"] = [str_to_put]
            
            if len(problematic_sym_pair) > 0:
                str_to_put = f"More than 2 points for this pair {json.dumps(problematic_sym_pair)}"
                if name in all_invalid_cars:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"].append(str_to_put)
                else:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"] = [str_to_put]
                    
            if len(problematic_tire) > 0:
                str_to_put = f"More than 2 points for this tire {json.dumps(problematic_tire)}"
                if name in all_invalid_cars:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"].append(str_to_put)
                else:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"] = [str_to_put] 
                    
#             if len(problematic_center) > 0:
#                 str_to_put = f"More than 2 points for this center point {json.dumps(problematic_center)}"
#                 if name in all_invalid_cars:
#                     all_invalid_cars[f"task_{i['@task_id']}_{name}"].append(str_to_put)
#                 else:
#                     all_invalid_cars[f"task_{i['@task_id']}_{name}"] = [str_to_put] 
                    
            if not extremal_exist:
                if name in all_invalid_cars:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"].append('No extremal')
                else:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"] = ['No extremal']
            if not nonextremal_exist:
                if name in all_invalid_cars:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"].append('No nonextremal')
                else:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"] = ['No nonextremal']
        
            if num_mirror > 1:
                if name in all_invalid_cars:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"].append('More than 1 mirror')
                else:
                    all_invalid_cars[f"task_{i['@task_id']}_{name}"] = ['More than 1 mirror']
            #Check if inference list or not
#             if name.split("-")[0] in infer_failed:
#                 infer_failed.remove(name.split("-")[0])
#                 text = "Please check carefully for the order of each sym pair to make sure it goes from Driver to Passenger and also whether the tire points are labeled correct. Remove all sym pairs without distinguishing features or markings."
#                 if name in all_invalid_cars:
#                     all_invalid_cars[f"task_{i['@task_id']}_{name}"].append(text)
#                 else:
#                     all_invalid_cars[f"task_{i['@task_id']}_{name}"] = [text]
        else:
            if name in all_invalid_cars:
                all_invalid_cars[f"task_{i['@task_id']}_{name}"].append('Not done')
            else:
                all_invalid_cars[f"task_{i['@task_id']}_{name}"] = ['Not done']

# print_anno_result()


### Exporting all valid cars set for combination based on logic
name_out = annotation_folder
with open(output_path/f'{name_out}.pkl', 'wb') as file:
    pickle.dump(all_valid_cars_set, file)

#Create batch dict
batch_dict = dict()
count_objs = 0
for i in tqdm(anno['annotations']['image']):
    if i['@task_id'] not in official_task_ids:
        continue
    name = get_anno_fullname(i)
    if name in all_valid_cars_set:
        camera = name.split("-")[0].split("_")[-1]
        text = name.split("-")[1]
        frame = text.split("_")[0]
        obj_id =  text.split("_")[1]
        if camera not in batch_dict:
            batch_dict[camera] = dict()
        if frame in batch_dict[camera]:
            batch_dict[camera][frame].append(process_one_object(i, name))
        else:
            batch_dict[camera][frame] = [process_one_object(i, name)]
        count_objs += 1

for l,m in batch_dict.items():
    saved_path = output_path / "formatted_anno" / name_out / l / "annotation"
    saved_path.mkdir(parents=True, exist_ok=True)
    for k,v in m.items():
        with open(saved_path/f'{k}.json', 'w') as f:
            f.write(json.dumps(v))
