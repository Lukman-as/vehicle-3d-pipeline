%% SETUP
data_path = '/mnt/c/Projects_raw_data/HW7';
segment = "26";
camera = "lc2";

%% ======= GET THE EXTRINSICS, INTRINSICS ==========
file = sprintf("%s/calib/intrinsic_calibrations.json",data_path);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
intrinsics_data = jsondecode(str);
k_matrix = intrinsics_data.(sprintf("%s",camera)).intrinsic_matrix(1:3,1:3);
assert(k_matrix(1,2)==0);

file = sprintf("%s/image/Seg%s/extrinsic_calibrations_%s.json",data_path,segment,segment);
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


%% Load the terrain
file = sprintf("%s/terrain/dtm_points_aoi.xyz",data_path);
fid = fopen(file);
terrain = fscanf(fid, '%f');
terrain = reshape(terrain, 3, []);
terrain = double(terrain);

% Transform terrain to world coordinate system 
terrain = M_world_velo*T_velo_utm*[terrain; ones(1,size(terrain,2))];
terrain = terrain(1:3,:);

%Only infront of camera, this is
% terrain_cam = R*terrain(1:3,:);
% terrain = terrain(:, terrain_cam(3,:) > 0); %Crucial to remove behind cam



%% Read LIDAR  TODO: read LIDAR and visualize on image    
target = "0050";
file = sprintf("%s/image/Seg%s/lidar_local_gta/%s.csv",data_path,segment, num2str(str2num(target)-1)); %Difference due to indexing
ptCloud = csvread(file);

%% Transformation to World Coordinate system
ptCloud_world = M_world_velo*[ptCloud';ones(1,size(ptCloud,1))];
ptCloud_world = ptCloud_world(1:3,:);
%Only infront of camera, this is
% ptCloud_world_cam = R*ptCloud_world;
% ptCloud_world = ptCloud_world(:, ptCloud_world_cam(3,:) > 0); %Crucial to remove behind cam

%% Not in camera but in 3D
if false
    figure
    hold on
    pcshow(terrain','blue','MarkerSize',5);
    pcshow(ptCloud_world','green','MarkerSize',5);
    pcshow([0,0,0], 'magenta','MarkerSize',100);
end

if false
    figure;
    hold on
    colors = terrain(2,:);
    scatter(terrain(1,:),terrain(3,:),1,'blue','filled');
    colors = ptCloud_world(2,:);
    scatter(ptCloud_world(1,:),ptCloud_world(3,:),3,colors,'filled');
%     scatter(ptCloud_world(1,:),ptCloud_world(3,:),2,'green','filled');
    roi = drawpolygon('Color','r');
    axis equal
    axis on
end

% save(sprintf("hw7_proof_maskings_%s_%s_long_lidar.mat",segment, target), 'v1');

%% Selecting inside ROI
upper_bound = mean(ptCloud_world(2,:)) + 3*std(ptCloud_world(2,:));
lower_bound = mean(ptCloud_world(2,:)) - 3*std(ptCloud_world(2,:));
ptCloud_world = ptCloud_world(:,ptCloud_world(2,:) <= upper_bound & ptCloud_world(2,:) >= lower_bound);

load(sprintf("hw7_proof_maskings_%s_%s_long_lidar.mat",segment, target))
[in1,on] = inpolygon(ptCloud_world(1,:),ptCloud_world(3,:),v1(:,1)',v1(:,2)');
ptCloud_inlier = ptCloud_world(:,in1);
% [in1,on] = inpolygon(ptCloud_world(1,:),ptCloud_world(3,:),v2(:,1)',v2(:,2)');
% ptCloud_inlier = ptCloud_world(:,in1);

% ptCloud_inlier = [ptCloud_inlier_1, ptCloud_inlier_2];

numCols = size(ptCloud_inlier, 2);
randomIndices = randperm(numCols);
% Select a specific number of random rows (e.g., 2 random rows)
numRandom = 500;
sel_col= randomIndices(1:numRandom);
ptCloud_compare = ptCloud_inlier(:,sel_col);

% histogram(ptCloud_compare(2,:))



%% Read the raw_image file
file = sprintf("%s/image/Seg%s/%s/%s.png",data_path,segment, camera, target);
fid = fopen(file);
raw_image = imread(file);
if false
    figure;
    imshow(raw_image)
end
image = raw_image;

%% Visualize on image and 3D
if false
    if false
        figure
        hold on
        pcshow(terrain',MarkerSize=3);
        pcshow([0,0,0], 'magenta','MarkerSize',100);
    end
    
    % Visualization on image
    if false
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
%         scatter(terrain_2D(1,:),terrain_2D(2,:),1,colors,'filled','pentagram');
%         scatter(terrain_2D(1,:),terrain_2D(2,:),1,"blue",'filled','pentagram');


        %Selected points only within image only
        ptCloud_2D = k_matrix*R*ptCloud_world;
        ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
        within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
        ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
        ptCloud_2D = ptCloud_2D(:,within_indice);
        colors = round(ptCloud_world(2,within_indice),2);
        scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),3,colors,'filled');
%         scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,"yellow",'filled');
%         colorbar
    end
end

% 
% %% Proof 1: Randomly sample in the pointCloud world and then compare to the nearest in the terrain
% %Filter out within image first
% ptCloud_2D = k_matrix*R*ptCloud_world;
% ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
% within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
% ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
% ptCloud_world = ptCloud_world(1:3,within_indice); %Select point within image first
% ptCloud_2D = k_matrix*R*ptCloud_world;
% ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
% 
% %Select point within the polygon from both
% %% Draw polygon on image
% if false
%     figure;
%     hold on
%     imshow(image)
%     hold on
%     colors = round(ptCloud_2D(2,:),2);
%     scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,colors,'filled');
%     roi = drawpolygon('Color','r');
%     hold on
% end
% 
% % save('Seg23_sc4_roi.mat', 'v1');
% load('Seg23_sc4_roi.mat');
% 
% if false
%     figure;
%     hold on
%     imshow(image)
%     hold on
%     colorbar
%     colors = round(ptCloud_world(2,:),2);
%     scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,colors,'filled');
%     drawpolygon('Position',v1,'Color','r');
% end
% 
% %% Select inpolygon only
% [in1,on] = inpolygon(ptCloud_2D(1,:),ptCloud_2D(2,:),v1(:,1)',v1(:,2)');
% ptCloud_inlier = ptCloud_world(:,in1);
% % histogram(ptCloud_inlier(2,:),100)
% 
% 
% % %Select form inliers point first
% % maxDistance = 0.2;  %RANSAC distance is okay to make more granular ransac heights first
% % referenceVector = [0,1,0];
% % maxAngularDistance = 0.05;
% % 
% % ptCloud = pointCloud(ptCloud_world(1:3,within_indice)');
% % [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
% %         maxDistance,referenceVector,maxAngularDistance);
% % remainPtCloud = select(ptCloud,outlierIndices);
% % plane1 = select(ptCloud,inlierIndices);
% % ransac_height = double(-model1.Parameters(4));
% % ptCloud_inlier = plane1.Location;
% % ptCloud_inlier = ptCloud_inlier';
% 
% 
% if false
%     figure;
%     hold on
%     imshow(image)
%     hold on
%     %Selected points only within image only
%     ptCloud_2D = k_matrix*R*ptCloud_world;
%     ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
%     within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
%     ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
%     ptCloud_2D = ptCloud_2D(:,within_indice);
%     colors = round(ptCloud_world(2,within_indice),2);
%     scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,"yellow",'filled');
% 
%     ptCloud_2D = k_matrix*R*ptCloud_inlier;
%     ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
%     within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
%     ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
%     ptCloud_2D = ptCloud_2D(:,within_indice);
%     colors = round(ptCloud_world(2,within_indice),2);
%     scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,"green",'filled');
% end
% 
% %Random columns
% numCols = size(ptCloud_inlier, 2);
% randomIndices = randperm(numCols);
% % Select a specific number of random rows (e.g., 2 random rows)
% numRandom = 500;
% sel_col= randomIndices(1:numRandom);
% ptCloud_compare = ptCloud_inlier(:,sel_col);
% 
% %Visualize the compare
% if false
%     figure;
%     hold on
%     imshow(image)
%     hold on
%     %Selected points only within image only
%     ptCloud_2D = k_matrix*R*ptCloud_world;
%     ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
%     within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
%     ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
%     ptCloud_2D = ptCloud_2D(:,within_indice);
%     colors = round(ptCloud_world(2,within_indice),2);
%     scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,"yellow",'filled');
% 
%      terrain_2D = k_matrix*R*terrain(1:3,:);
%     terrain_2D = terrain_2D./terrain_2D(3,:);
%     within_indice = terrain_2D(1,:)>=0 & terrain_2D(1,:)<=size(image,2) & ...
%         terrain_2D(2,:)>=0 & terrain_2D(2,:)<=size(image,1);
%     terrain_2D = terrain_2D(:,within_indice);
%     colors = terrain(2,within_indice);
% %         scatter(terrain_2D(1,:),terrain_2D(2,:),1,colors,'filled','pentagram');
%     scatter(terrain_2D(1,:),terrain_2D(2,:),1,"blue",'filled','pentagram');
% 
%     ptCloud_2D = k_matrix*R*ptCloud_compare;
%     ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
%     scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),5,"red",'filled');
% end


%% Find the nearest point in 3D
[distances,indices] = pdist2([terrain(1,:);terrain(3,:)]',...
    [ptCloud_compare(1,:);ptCloud_compare(3,:)]',...
    'euclidean','Smallest',1);

nearest_in_terrain = terrain(:,indices);
height_diff = ptCloud_compare(2,:) - nearest_in_terrain(2,:);

target
bias = mean(height_diff)
mae = mean(abs(height_diff))


% histogram(height_diff,100)
% 
% nearest_in_terrain(:,1:10)'
% ptCloud_compare(:,1:10)'

%Project on top of image
if false
    figure;
    hold on
    imshow(image)
    hold on
    terrain_2D = k_matrix*R*nearest_in_terrain(1:3,:);
    terrain_2D = terrain_2D./terrain_2D(3,:);
    within_indice = terrain_2D(1,:)>=0 & terrain_2D(1,:)<=size(image,2) & ...
        terrain_2D(2,:)>=0 & terrain_2D(2,:)<=size(image,1);
    terrain_2D = terrain_2D(:,within_indice);
    colors = nearest_in_terrain(2,within_indice);
%     scatter(terrain_2D(1,:),terrain_2D(2,:),5,colors,'filled','pentagram');
    scatter(terrain_2D(1,:),terrain_2D(2,:),5,"blue",'filled');

    


    %Selected points only within image only
    ptCloud_2D = k_matrix*R*ptCloud_compare;
    ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
    within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
    ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
    ptCloud_2D = ptCloud_2D(:,within_indice);
    colors = round(ptCloud_compare(2,within_indice),2);
%     scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),5,colors,'filled');
    scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),5,"red",'filled');
    sel =  randsample(size(terrain_2D,2),200)';
    for k = 1:size(sel,2)
        i = sel(k)
        text(terrain_2D(1,i)+1,terrain_2D(2,i)-1,sprintf('%.3f',height_diff(i)), 'Color', 'yellow','FontSize',6);
    end
end

