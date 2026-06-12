%% READ THE raw_image File
drive_name = '000113'
file_name = '1616536889200';
camera_name = 'cam03'
data_path = '/mnt/c/Projects_raw_data/ONCE/data';
raw_image = imread(sprintf("%s/%s/%s/%s.jpg",data_path,drive_name,camera_name,file_name));
vpcar_point = 'in';
near_side = 'passenger';

if true
    figure
    imshow(raw_image)
end

%% ======= GET THE VELO TO CAM ROTATION MATRIX ==========
json_file = sprintf("%s/%s/%s.json",data_path,drive_name,drive_name)
fid = fopen(json_file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
raw_labels = jsondecode(str); 

%% ======= UNDISTORT RAW IMAGE ==========
k_matrix = raw_labels.calib.(camera_name).cam_intrinsic
distortionCoefficients = raw_labels.calib.(camera_name).distortion
imageSize = [size(raw_image,1) size(raw_image,2)]
intrinsics = cameraIntrinsicsFromOpenCV(k_matrix, ...
                                       distortionCoefficients,imageSize);
image = undistortImage(raw_image,intrinsics,'OutputView','same');

%% ======= VISUALIZING DISTORT AND UNDISSTORT ==========
imshowpair(raw_image, image)
tiledlayout(2,1);
nexttile
imshow(raw_image)
nexttile
imshow(image)

%% ======= SET UP COORDINATE SYSTEM ==========
M_cam_velo = inv(raw_labels.calib.(camera_name).cam_to_velo)

% Decompose into
R_cam_velo = M_cam_velo(1:3,1:3)
t_cam_velo = M_cam_velo(1:3,4)

%Set up world Pending continue later
R_velo_world = [[0 -1 0]' [0 0 -1]' [1 0 0]']
R_cam_world = R_cam_velo * R_velo_world
R_world_cam = inv(R_cam_world)
t_world_velo = R_world_cam*t_cam_velo

M_world_velo = [inv(R_velo_world) t_world_velo; 0 0 0 1]
R = R_cam_world
