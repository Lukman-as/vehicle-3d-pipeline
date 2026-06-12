



%% Begin of the loop
target = "000000"
ipu_id = "1"
cam_id = "1"

%% READ THE raw_image File
data_path = '/mnt/c/Projects_raw_data/IPS';
file = sprintf("%s/data/IPU%s/IPU%s_CAM%s/%s.png",data_path, ipu_id, ipu_id, cam_id,target);
fid = fopen(file);

raw_image = imread(file);

if true
    figure;
    imshow(raw_image)
end
image = raw_image;

%% ======= GET THE EXTRINSICS, INTRINSICS ==========
file = sprintf("%s/calib_json/%s.json",data_path,target);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
exin_data = jsondecode(str)
k_matrix = reshape(exin_data.x_IPU1_cam1_IntrinsicMatrix,[3,3])
proj_matrix = reshape(exin_data.x_IPU1_Lidar___IPU1_cam1,[4,4])';

format long
rad_distortion = exin_data.x_IPU1_cam1_Intrinsics_RadialDistortion
tan_distortion = exin_data.x_IPU1_cam1_Intrinsics_TangentialDistortion


cameraParams = cameraParameters("K",k_matrix,"RadialDistortion",rad_distortion,"TangentialDistortion",tan_distortion)

image_undistorted = undistortImage(image,cameraParams);

montage({image,image_undistorted})
title("Original Image (left) vs. Corrected Image (right)")

imshow(image_undistorted)



%% ==== Get Rotation and Translation out =====
rot = proj_matrix(1:3,1:3);
assert(det(rot)==1)
t = proj_matrix(1:3,4);
M_cam_velo = [rot,t;0 0 0 1];


file = sprintf("%s/data/PC_COM_ROI/%s.pcd",data_path, target)
ptCloud = pcread(file)
ptCloud_world = [ptCloud.Location';ones(1,size(ptCloud.Location,1))];
% ptCloud_world = M_cam_velo*ptCloud_world;
% ptCloud_world = ptCloud_world(1:3,:);


cam_cloud = proj_matrix*ptCloud_world

pcshow(cam_cloud(1:3,:)')


% [k_matrix,[0,0,0]';0 0 0 1]*
ptCloud_2D = proj_matrix*ptCloud_world;
ptCloud_2D = ptCloud_2D(1:3,:);
ptCloud_2D = k_matrix*ptCloud_2D

% ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
ptCloud_2D = ptCloud_2D./3


if true
    figure;
    hold on
    imshow(image)
    hold on
    scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),2,"green",'filled')
end



