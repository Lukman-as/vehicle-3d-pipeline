%% Filter out all the coop files
data_path = '/mnt/c/Projects_raw_data/COOP';
file = sprintf("%s/data1/cooperative-vehicle-infrastructure/" + ...
        "cooperative/data_info.json",data_path);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
data_info = jsondecode(str); 
coop_data = struct;

for i = 1:size(data_info,1)
    arr = split(data_info(i).infrastructure_image_path,"/");
    arr = split(arr(end),".");
    fieldname = sprintf("obj_%s",string(arr(1,1)));
    coop_data.(fieldname) = data_info(i);
    coop_data.(fieldname).("file_id") = string(arr(1,1));
end

%% Filter out the infra data
file = sprintf("%s/data1/cooperative-vehicle-infrastructure/" + ...
        "infrastructure-side/data_info.json",data_path);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
data_info = jsondecode(str);
all_scene_ids = [];
for i = 1:size(data_info,1)
    arr = split(data_info{i}.image_path,"/");
    arr = split(arr(end),".");
    fieldname = sprintf("obj_%s",string(arr(1,1)));
    if isfield(coop_data,fieldname)
        fn = fieldnames(data_info{i});
        coop_data.(fieldname).("infra_info") = struct;
        for k = 1:numel(fn)
            coop_data.(fieldname).("infra_info").(fn{k}) = data_info{i}.(fn{k}); 
        end
        all_scene_ids = [all_scene_ids string(arr(1,1))];
    end
end

%% Main loop to eval
fileID = fopen('results_batch_2.txt','w');
fileID_opt = fopen('results_batch_2_opt.txt','w');
batch = "scene_15596_ceci_0731"
folder = sprintf("%s/data1/cooperative-vehicle-infrastructure/infrastructure-side/%s/annotation",data_path,batch)
filePattern = fullfile(folder, '*.json');
all_files = dir(filePattern);
using_rectified = true;

%% Get one file one
car_model_param = load('car_model_param.mat');
bias_check = [];
invalid_cars = [];

format long;
for k = 1 : length(all_files)
    baseFileName = all_files(k).name;
    parts = split(baseFileName,".");
    target = string(parts(1))
    debug = false;
    if debug
        target = "015896";
    end

    obj_field = sprintf("obj_%s",target);
    lidar_time = str2num(coop_data.(obj_field).("infra_info").pointcloud_timestamp);
    camera_time = str2num(coop_data.(obj_field).("infra_info").image_timestamp);
    time_diff = camera_time - lidar_time;

    %% Read the raw_image file
    data_path = '/mnt/c/Projects_raw_data/COOP';
    file = sprintf("%s/data1/cooperative-vehicle-infrastructure/infrastructure-side/image/%s.jpg",...
        data_path,target);

    fid = fopen(file);
    raw_image = imread(file);
    if false
        figure;
        imshow(raw_image)
    end
    image = raw_image;
    
    % ======= GET THE EXTRINSICS, INTRINSICS ==========
    file = sprintf("%s/data1/cooperative-vehicle-infrastructure/infrastructure-side/calib/camera_intrinsic/%s.json",data_path,target);
    fid = fopen(file);
    raw = fread(fid,inf);
    str = char(raw'); 
    fclose(fid); 
    intrinsics_data = jsondecode(str);
    k_matrix = reshape(intrinsics_data.cam_K,[3,3])'
    if debug
        break
    end
end
