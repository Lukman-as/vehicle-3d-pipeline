%% Read the raw_image file
debug = true;
if debug
    target = "0001";
end
sequence = "17";
camera = "sc1";
data_path = '/mnt/c/Projects_raw_data/HW7';
file = sprintf("%s/image/Seq%s/%s/%s.png",...
    data_path,sequence, camera, target);
fid = fopen(file);
raw_image = imread(file);
if false
    figure;
    imshow(raw_image)
end
image = raw_image;


%% ======= GET THE EXTRINSICS, INTRINSICS ==========
file = sprintf("%s/calib/intrinsic_calibrations.json",data_path);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
intrinsics_data = jsondecode(str);
k_matrix = intrinsics_data.(sprintf("%s",camera)).intrinsic_matrix(1:3,1:3);
assert(k_matrix(1,2)==0);

file = sprintf("%s/calib/extrinsic_calibrations.json",data_path);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid);
extrinsic_allcams = jsondecode(str);
extrinsics_data = extrinsic_allcams.(sprintf("T_%s_localgta",camera));
T_velo_utm = extrinsic_allcams.("T_localgta_gta");

%% ======= SET UP WORLD AT CAM ==========
rot = extrinsics_data(1:3,1:3);
t = extrinsics_data(1:3,4);
assert(abs(det(rot)-1)<1e-5); %Check valid rotation
M_cam_velo = [rot,t;0 0 0 1];
R_velo_world = [[0 -1 0]' [0 0 -1]' [1 0 0]']; %Set up rotation of World with y-axis point down 
assert(abs(det(R_velo_world)-1)<1e-5)
R_cam_velo = M_cam_velo(1:3,1:3);
M_cam_world = [R_cam_velo*R_velo_world,[0 0 0]'; 0 0 0 1]; %Transformation from world to cam
M_world_velo = inv(M_cam_world)*M_cam_velo; %Transformation from velo to world
assert(norm(M_cam_world*M_world_velo-M_cam_velo)<1e-6);
R = M_cam_world(1:3,1:3);
assert(abs(det(R)-1)<1e-5); %Change from 1e-5

%% =======  Get horizon line ======= 
vp_points = get_vppoints(k_matrix,R)
vp_points_2D = vp_points./vp_points(3,:);
horizon_line = cross(vp_points(:,1),vp_points(:,3));

if false
    figure;
    hold on
    imshow(image)
    hold on
    scatter(vp_points_2D(1,1),vp_points_2D(2,1),100,'red','filled');
    scatter(vp_points_2D(1,2),vp_points_2D(2,2),100,'green','filled');
    scatter(vp_points_2D(1,3),vp_points_2D(2,3),50,'blue','filled');
    line([vp_points_2D(1,1), vp_points_2D(1,3)],[vp_points_2D(2,1),...
        vp_points_2D(2,3)],'Color','red','LineWidth',2)
    hold on
    axis equal
    axis on
end


%%  =======  TEST THE TRANSFORMATION IN THE TERRAIN MODEL =======
% Get raw utm
file = sprintf("%s/terrain/dsm_hw7.xyz",data_path);
fid = fopen(file);
terrain = fscanf(fid, '%f');
terrain = reshape(terrain, 3, []);
terrain = double(terrain);

% Transform terrain to world coordinate system 
terrain = M_world_velo*T_velo_utm*[terrain; ones(1,size(terrain,2))];
terrain = terrain(1:3,:);

%Only infront of camera
terrain_cam = R*terrain(1:3,:);
terrain = terrain(:, terrain_cam(3,:) > 0);


%% Visualize on image
if true
    if false
        figure
        hold on
        pcshow(terrain',MarkerSize=3);
        pcshow([0,0,0], 'magenta','MarkerSize',100);
    end
    
    % Visualization on image
    if true
        figure;
        hold on
        imshow(image)
        hold on
        terrain_2D = k_matrix*R*terrain(1:3,:);
        terrain_2D = terrain_2D./terrain_2D(3,:);
        within_indice = terrain_2D(1,:)>=0 & terrain_2D(1,:)<=size(image,2) & ...
            terrain_2D(2,:)>=0 & terrain_2D(2,:)<=size(image,1);
        terrain_2D = terrain_2D(:,within_indice);
        colors = terrain(2,within_indice);
        scatter(terrain_2D(1,:),terrain_2D(2,:),1,colors,'filled');
    end
end