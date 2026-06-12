import json
from pathlib import Path
import cv2
from tqdm import tqdm


def get_biggest_cars(path, segment, camera, time, output_path):
    print(camera)
    img = cv2.imread(str(path/f"image/Seg{segment}/{camera}/0001.png"))
    y_size = img.shape[0]
    x_size = img.shape[1]
    json_path = Path(output_path/f"saved_tracks_json/Seg{segment}/{camera}")
    with open(f"{str(json_path)}/Seg{segment}_{camera}_{time}.json","r") as f:
        good_tracks_to_save = json.load(f)
        buffer = 0.15
        for k,v in sorted(good_tracks_to_save.items()):
            for car in v:
                target, obj_id = car.split("_")
                tracking_output_path = Path(output_path/f"saved_tracks_model_make/Seg{segment}/{camera}/{k}")
                tracking_output_path.mkdir(parents=True, exist_ok=True)
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
                cv2.imwrite(str(tracking_output_path/f"Seg{segment}_{camera}-{car}-{text}.jpg"), cropped_image, [cv2.IMWRITE_JPEG_QUALITY, 100])

def main():
    ROOT = Path(__file__).resolve().parents[1]
    path = ROOT / 'data/raw'
    output_path = ROOT / 'data/gen'
    segment = "23"
    cameras = ["sc1", "sc2", "sc3", "sc4"]
    time = "202411"
    for camera in cameras:
        get_biggest_cars(path, segment, camera, time, output_path)

if __name__ == "__main__":
    main()
