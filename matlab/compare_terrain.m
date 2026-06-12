%% READ THE raw_image File
target = "015596";
file_name = target;
scene = struct;
scene.file_id = target;

data_path = '/mnt/c/Projects_raw_data/COOP';
file = sprintf("%s/data1/cooperative-vehicle-infrastructure/infrastructure-side/image/%s.jpg",...
    data_path,target);
raw_image = imread(file);

if false
    figure;
    imshow(raw_image)
end
image = raw_image;

%% Load the image
file = sprintf("%s/data1/cooperative-vehicle-infrastructure/infrastructure-side/calib/camera_intrinsic/%s.json",data_path,file_name);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
intrinsics_data = jsondecode(str);
k_matrix = reshape(intrinsics_data.cam_K,[3,3])';
load uframe_15596_rec.mat uniform_frame;
M_cam_world_main = uniform_frame.M_cam_world_main;
M_world_velo = uniform_frame.M_world_velo_main;
R = M_cam_world_main(1:3,1:3); %There is only rotation
k_matrix_main = k_matrix;
R_main = R;

%% Load the terrain 
load lowess_15596_rob_kur_clo
f1 = f;
load lowess_15596_rec
f2 = f;

tic
f1([0 40])
toc

tic
f2([0 40])
toc


%% Prediction in the region
x_val = -40:0.5:40;
z_val = 0:0.5:100;
[xn, zn] = ndgrid(x_val,z_val); 
M2 = [xn(:), zn(:)];

tic
y_pred1 = f1(M2);
% z_plot1 = reshape(y_pred1, size(x_val,2), size(z_val,2));
% points_model1 = [M2(:,1) y_pred1 M2(:,2)]; %This is the prediction
toc


tic
y_pred2 = f2(M2);
toc

%Subtract from each other
y_pred_diff = y_pred1-y_pred2;

points_model = [M2(:,1) y_pred2 M2(:,2)];

if true
    figure
    hold on
    diff_model = [M2(:,1) y_pred_diff M2(:,2)]; %This is the prediction
    pcshow(diff_model)
    pcshow([0,0,0], 'magenta','MarkerSize',100);
end


%Visualize ontop of image 
points_model_2D = k_matrix*R*points_model';
points_model_2D = points_model_2D./points_model_2D(3,:);

load 15596_roi_mask_rec;

%Within polygons instead
%Filter out outliers height for visualization
within_indice = y_pred_diff <= 2 & ...
        y_pred_diff >= -2 & ...
        inpolygon(points_model_2D(1,:),points_model_2D(2,:),mask(:,1),mask(:,2))';
within_indice = logical(within_indice);
points_model_2D = points_model_2D(:,within_indice');


colors = y_pred_diff(within_indice);

mean(colors)
median(colors)

figure;
hold on
imshow(image)
hold on
scatter(points_model_2D(1,:),points_model_2D(2,:),2,colors,'filled');
% drawpolygon('Position',mask,'Color','r','FaceAlpha',0,'MarkerSize',1);
colormap(gca,"parula")
colorbar

