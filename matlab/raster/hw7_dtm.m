%% SETUP
data_path = '/mnt/c/Projects_raw_data/HW7';
segment = "23";
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

% %Only infront of camera, this is
terrain_cam = R*terrain(1:3,:);
terrain = terrain(:, terrain_cam(3,:) > 0); %Crucial to remove behind cam



%% Read LIDAR  TODO: read LIDAR and visualize on image    
target = "0001";
file = sprintf("%s/image/Seg%s/lidar_local_gta/%s.csv",data_path,segment, num2str(str2num(target)-1)); %Difference due to indexing
ptCloud = csvread(file);

%% Transformation to World Coordinate system
ptCloud_world = M_world_velo*[ptCloud';ones(1,size(ptCloud,1))];
ptCloud_world = ptCloud_world(1:3,:);
%Only infront of camera, this is
ptCloud_world_cam = R*ptCloud_world;
ptCloud_world = ptCloud_world(:, ptCloud_world_cam(3,:) > 0); %Crucial to remove behind cam





%% Read the raw_image file
target = "0001";
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
        scatter(terrain_2D(1,:),terrain_2D(2,:),1,colors,'filled','pentagram');

        %Selected points only within image only
%         ptCloud_2D = k_matrix*R*ptCloud_world;
%         ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
%         within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
%         ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
%         ptCloud_2D = ptCloud_2D(:,within_indice);
%         colors = round(ptCloud_world(2,within_indice),2);
%         scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,colors,'filled');
%         scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),1,"yellow",'filled');
        colorbar
    end
end





%% Draw polygon on image
if false
    figure;
    hold on
    imshow(image)
    hold on
    scatter(terrain_2D(1,:),terrain_2D(2,:),1,colors,'filled');
    roi = drawpolygon('Color','r');
    hold on
end

% save('23_lc2_roi_maskings.mat', 'v1');
load '23_lc2_roi_maskings.mat';

if false
    figure;
    hold on
    imshow(image)
    drawpolygon('Position',v1,'Color','r');
    hold on
end

%Filter out the things that are in only
terrain_2D = k_matrix*R*terrain(1:3,:);
terrain_2D = terrain_2D./terrain_2D(3,:);
[in1,on] = inpolygon(terrain_2D(1,:),terrain_2D(2,:),v1(:,1)',v1(:,2)');
terrain_in = terrain(:,in1);
terrain_notin = terrain(:,~in1);

%Revisualize
if false
    figure;
    hold on
    imshow(image)
    hold on
    terrain_2D = k_matrix*R*terrain_in(1:3,:);
    terrain_2D = terrain_2D./terrain_2D(3,:);
    colors = terrain_in(2,:);
    scatter(terrain_2D(1,:),terrain_2D(2,:),1,colors,'filled','pentagram');
    drawpolygon('Position',v1,'Color','r');
    hold on
    colorbar
end
terrain = terrain_in;




%% Remove outliers just to clear some because RANSAC does not work here not really flat
upper_bound = mean(terrain(2,:)) + 3*std(terrain(2,:));
lower_bound = mean(terrain(2,:)) - 3*std(terrain(2,:));
terrain = terrain(:,terrain(2,:) <= upper_bound & terrain(2,:) >= lower_bound);


%% K Nearest neighbor approach - Find the ideal K
X = [terrain(1,:)' terrain(3,:)'];
y = terrain(2,:)'; % Response variable

mean(y)
std(y)
histogram(y)

if false
    format long
    number_of_exps = 100000;
    tic
    for k_val = 1:1:10
        tic
        knn = k_val;
        y_pred = zeros(number_of_exps,1);
        y_true = zeros(number_of_exps,1);
        for i = 1:number_of_exps
            % Get the number of rows in the matrix
            numRows = size(X, 1);
            % Generate a random permutation of row indices
            randomRowIndices = randperm(numRows);
            
            % Select a specific number of random rows (e.g., 2 random rows)
            numRandomRows = 1;
            sel_row = randomRowIndices(1:numRandomRows);
            X_sel = X(sel_row,:);
        
            %Calculate all the distances
            X_leave_one_out = X([1:sel_row-1, sel_row+1:end],:);
            y_leave_one_out = y([1:sel_row-1, sel_row+1:end],:);
            distances = pdist2(X_sel, X_leave_one_out);
            [minValue, linearIndex] = mink(distances,knn);
        
            %Equal weighing
            y_nearby = y(linearIndex);
            y_pred(i) = mean(y_nearby);
            y_true(i) = y(randomRowIndices(1:numRandomRows), :);
        end
        k_val
        rmse = sqrt(sum(power(y_pred-y_true,2))/size(y_true,1))
        toc
    end
    toc
end



%% KNN marching in 2D to get the final RASTER
knn_selected = 4;
points_model = terrain';
if true
    x_vals = 1:1:size(image,2); %Gotta start from 1, otherwise index error
    y_vals = 1:1:size(image,1);
    [xn, zn] = ndgrid(x_vals,y_vals); 
    all_points = [xn(:), zn(:)];
    within_indice = inpolygon(all_points(:,1),all_points(:,2),v1(:,1),v1(:,2))';
    within_indice = logical(within_indice);
    all_points = all_points(within_indice,:);
    size(all_points)
    % Fill in all for the raster
    tic
    origin = [0,0,0];
    bs = 5000;
    buffer = 10; %Buffer around the impact points to limit the region to calculate the distance, speed up
    mean_height = mean(points_model(:,2));

    final_raster = zeros(size(image,2),size(image,1));    
    for i=1:size(all_points)
        reproj_ray = inv(k_matrix*R)*[all_points(i,:),1]';
        impact_point = mean_height/reproj_ray(2)*reproj_ray;
        x_min = impact_point(1)-buffer;
        x_max = impact_point(1)+buffer;
        z_min = impact_point(3)-buffer;
        z_max = impact_point(3)+buffer;
        local_points_model = points_model(points_model(:,1) > x_min & points_model(:,1) < x_max & ... 
            points_model(:,3) > z_min & points_model(:,3) < z_max,:);
        distance_3D = point_to_line_distance(local_points_model, origin, reproj_ray);
        [minVal, linearIndex] = mink(distance_3D,knn_selected); %Use the same k as above
        final_raster(all_points(i,1), all_points(i,2)) = mean(local_points_model(linearIndex,2));
        if mod(i, bs) == 0
            i
            every_batch = toc
            sprintf("time left: %.2f minutes",every_batch/bs*(size(all_points,1)-i)/60)
            tic
        end
    end
end


%% KNN Visualize raster and save
if false
    % Find the row and column indices of non-zero elements
    [rowIndices, colIndices, values] = find(final_raster);
    
    % Create a matrix of indices with corresponding values
    indicesWithValues = [rowIndices, colIndices, values];

    points_model_2D = indicesWithValues(:,1:2)';
    colors = indicesWithValues(:,3)';

    figure;
    hold on
    imshow(image)
    hold on
    scatter(points_model_2D(1,:),points_model_2D(2,:),2,colors,'filled');
    drawpolygon('Position',mask,'Color','r','FaceAlpha',0,'MarkerSize',1);
    colormap(gca,"parula")
    c = colorbar;
    c.Label.String = 'Vertical depth (m)';
    c.FontSize = 23;
    c.Label.FontSize = 30;
%     c.FontSize = 15;
%     c.Label.FontSize = 15;
    multiple = 0.05;
    c.Ticks = round(min(colors)/multiple)*multiple:multiple:round(max(colors)/multiple)*multiple;
%     c.Limits = [6,8];
    c.Position = [0.89 0.093 0.025 0.87];
%     c.Position = [0.9 0.093 0.025 0.87];
    axis equal;
end

% save 23_lc2_final_raster final_raster