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

%% Selected pics
% pics = [700,780,2998,3144;
% 697,790,2865,3030;
% 695,777,2748,2905;
% 781,890,1696,1877;
% 675,750,3158,3306;];
% 
% size(raw_image)
% 
% for i=1:size(pics,1)
%     pos = pics(i,:);
% %     figure;
% %     imshow(raw_image(pos(1):pos(2),pos(3):pos(4),:));
%     filename = sprintf("%s/%s_%s-%s_%s_%s_%s.jpg",data_path,target,num2str(i),num2str(pos(1)),num2str(pos(2)),num2str(pos(3)),num2str(pos(4)));
%     imwrite(raw_image(pos(1):pos(2),pos(3):pos(4),:), filename);
% end
% 

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


%%  =======  TEST THE TRANSFORMATION USING ONE POINT IN THE TERRAI MODEL =======
if false
    lat = 43.84446529774904;
    lon = -79.38203124552143;
    [x,y,zone] = ll2utm(lat,lon);
    latlon_utm = [x; y];
    % Get raw utm
%     file = sprintf("%s/terrain/dsm_hw7.xyz",data_path);
    file = sprintf("%s/terrain/dtm_points_aoi.xyz",data_path);
    fid = fopen(file);
    terrain = fscanf(fid, '%f');
    terrain = reshape(terrain, 3, []);
    terrain = double(terrain);
    [minValue, minIndex] = min(vecnorm(latlon_utm - terrain(1:2,:)));
    target_point = terrain(:,minIndex);
    
    % Transform terrain to world coordinate system 
    terrain = M_world_velo*T_velo_utm*[terrain; ones(1,size(terrain,2))];
    terrain = terrain(1:3,:);
    
    %Only infront of camera, this is
    terrain_cam = R*terrain(1:3,:);
    terrain = terrain(:, terrain_cam(3,:) > 0);
    % Chop off none intersection
%     terrain = terrain(:, terrain(3,:) > -68);
end


%% Visualize on image
if false
%     terrain = terrain(:, terrain(2,:) > 8 & terrain(2,:) < 20); %Only near the ground
    target_point = M_world_velo*T_velo_utm*[target_point; 1];
    target_point = target_point(1:3,:);
    
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
        scatter(terrain_2D(1,:),terrain_2D(2,:),1,colors,'filled');
%         colorbar
        target_point_2D = k_matrix*R*target_point(1:3,:);
        target_point_2D = target_point_2D./target_point_2D(3,:);
        scatter(target_point_2D(1,:),target_point_2D(2,:),7,'r','filled');
    end
end

%% Adopting cloth
if false
    ptCloud_down_las = terrain;
    temp = ptCloud_down_las(2,:)*(-1);
    ptCloud_down_las(2,:) = ptCloud_down_las(3,:);
    ptCloud_down_las(3,:) = temp;
    pcdLocation= table(ptCloud_down_las'); %creates table with columns 1:3 = xyz-coord. 
    pcdLocation= table2array(pcdLocation); %converts to array
    pcdLocation= double(pcdLocation); % this is where we convert from single type to double type ptCloud2 = pointCloud(pcdLocation);
    ptCloud_clean = pointCloud(pcdLocation);
    
    % 1 for tilted terrain, 
    % 2 for terrain with gentle slop, 
    % 3 for city areas with flat terrain
    terrain_type = 3; %FIXED due to urban area, realatively flat
    slope_post_processing = false; %No slope no need
    clothResolution = 0.5;  %Resolution of cloth, smaller is more fine grain 
    class_threshold = 0.2; %Instead of 0.5 because you want to remove anything that is 10 cm above the ground
    max_iteration = 500; %Longer is better allow for time step optimization
    time_step = 0.65; %Use default for now, may reduce latter
    
    [groundIndex,nonGroundIndex] = csf_filtering(ptCloud_clean.Location,...
        terrain_type,slope_post_processing,clothResolution,class_threshold,max_iteration, time_step);
    %extract gound points and non-ground points
    groundPoints = pointCloud(ptCloud_clean.Location(groundIndex,:));
    nonGroundPoints = pointCloud(ptCloud_clean.Location(nonGroundIndex,:));
    assert(size(nonGroundIndex,1) > 0)
    size(nonGroundIndex,1)
    %show results
    figure
    pcshow(groundPoints,MarkerSize=3)
    title('ground points')
    figure;
    pcshow(nonGroundPoints)
    title('non-ground points, such as trees and houses')
    figure
    pcshowpair(groundPoints,nonGroundPoints)
    title('ground points and non-ground points, such as trees and houses')
    
    %% Save to ptcloud
    terrain_only = ptCloud_clean.Location(groundIndex,:)';
    temp = terrain_only(3,:)*(-1);
    terrain_only(3,:) = terrain_only(2,:);
    terrain_only(2,:) = temp;
    
    above_ground = ptCloud_clean.Location(nonGroundIndex,:)';
    temp = above_ground(3,:)*(-1);
    above_ground(3,:) = above_ground(2,:);
    above_ground(2,:) = temp;
end 


%% Visualization on image
if false
    figure;
    hold on
    imshow(image)
    hold on
    terrain_2D = k_matrix*R*above_ground(1:3,:);
    terrain_2D = terrain_2D./terrain_2D(3,:);
    within_indice = terrain_2D(1,:)>=0 & terrain_2D(1,:)<=size(image,2) & ...
        terrain_2D(2,:)>=0 & terrain_2D(2,:)<=size(image,1);
    terrain_2D = terrain_2D(:,within_indice);
    colors = above_ground(2,within_indice);
    scatter(terrain_2D(1,:),terrain_2D(2,:),2,colors,'filled');
%     colorbar
    target_point_2D = k_matrix*R*target_point(1:3,:);
    target_point_2D = target_point_2D./target_point_2D(3,:);
    scatter(target_point_2D(1,:),target_point_2D(2,:),7,'r','filled');
end

%% Downsampling
% save hw7_terrain terrain_only
if false
    size(terrain_only)
    ptCloud = pointCloud(terrain_only');
    terrain_only_down = pcdownsample(ptCloud,'gridAverage',0.7) 
    terrain_only_down =  terrain_only_down.Location';
    size(terrain_only_down,2)
    size(terrain_only_down,2)/size(terrain_only,2)

    figure
    hold on
    pcshow(terrain_only_down',MarkerSize=3);
    pcshow([0,0,0], 'magenta','MarkerSize',100);
end

%% Local curvefit
if false
    tic
    bw = 50;
    [f,gof,output] = fit([terrain_only(1,:)' terrain_only(3,:)'],...
        terrain_only(2,:)','lowess',"Span",bw/size(terrain_only,2), "Robust","Off")
    % plot(f,[ptCloud_world_accum(1,:)' ptCloud_world_accum(3,:)'],ptCloud_world_accum(2,:)')
    toc
    tic
    f([0 40])
    toc
end

%% Load terrain
load fitted_model_hw7_sc2.mat;
load hw7_ptCloud_above;
ptCloud_above_2D = k_matrix*R*ptCloud_above;
ptCloud_above_2D = ptCloud_above_2D./ptCloud_above_2D(3,:);
fobj = f;
car_model_param = load('car_model_param.mat');
fileID = fopen('results_batch_2.txt','w');
fileID_opt = fopen('results_batch_2_opt.txt','w');

if true
    % Get raw utm
    file = sprintf("%s/terrain/dsm_hw7.xyz",data_path);
    fid = fopen(file);
    terrain = fscanf(fid, '%f');
    terrain = reshape(terrain, 3, []);
    terrain = double(terrain);
    % Transform terrain to world coordinate system 
    terrain = M_world_velo*T_velo_utm*[terrain; ones(1,size(terrain,2))];
    terrain = terrain(1:3,:);
    
    %Only infront of camera, this is
    terrain_cam = R*terrain(1:3,:);
    terrain = terrain(:, terrain_cam(3,:) > 0);
    % Chop off none intersection
    ptCloud_world = terrain(:, terrain(3,:) > -68);
    size(ptCloud_world)

    ptCloud = pointCloud(ptCloud_world');
    ptCloud_world_down = pcdownsample(ptCloud,'gridAverage',0.8) 
    ptCloud_world_down =  ptCloud_world_down.Location';
    size(ptCloud_world_down,2)
    size(ptCloud_world_down,2)/size(ptCloud_world,2)
    ptCloud_world = ptCloud_world_down;
end


%% Filter out points to be within the mask and above the terrain model
if false
    tic
    y_pred = f([ptCloud_world(1,:)',ptCloud_world(3,:)']); %Use the terrain model to get the height
    toc
    above = ptCloud_world(2,:)' < (y_pred - 0.05); %A little of cusshion
    ptCloud_above = ptCloud_world(:,above');
    ptCloud_above_2D = k_matrix*R*ptCloud_above;
    ptCloud_above_2D = ptCloud_above_2D./ptCloud_above_2D(3,:);
%     save hw7_ptCloud_above ptCloud_above
end


%% Get the annotation
batch = "0001_07_17_thao";
file = sprintf("%s/anno/batch_%s/annotation/%s.json",data_path,batch,target);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
anno = jsondecode(str);

%Load annotations in but skip those without a tire annoted, we need at
%least a tire
annotations = struct;
all_cars_ids = [];


for i = 1:size(anno)
    if iscell(anno)
        if isfield(anno{i},'skip_reason') || ~isfield(anno{i},'annotations')
            continue
        end
        fn = fieldnames(anno{i}.annotations.tire_points);
        %Check min one tire point
        skipped = true;
        for k = 1:numel(fn)
            if sum(anno{i}.annotations.tire_points.(fn{k})) > 0
                skipped = false;
                break
            end
        end
        if ~skipped
            fieldname = sprintf("obj_%s",int2str(i));
            annotations.(fieldname) = anno{i}.annotations;
            all_cars_ids = [all_cars_ids i];
        end
    elseif isstruct(anno)
        if isfield(anno(i),'skip_reason') || ~isfield(anno(i),'annotations')
            continue
        end   
        fieldname = sprintf("obj_%s",int2str(anno(i).obj_id)); %Difference old farmat and new format
        annotations.(fieldname) = anno(i).annotations;
        %Skip the car that
        if anno(i).obj_id == 4
            continue
        end

        all_cars_ids = [all_cars_ids anno(i).obj_id];
%             break %If wanna test one car only DEBUG
    end
end

%% Start processing points
all_annos = struct;
% Process all the car tires
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = annotations.(fieldname);
    tire_fields = ["DR","PR","DF","PF"];
    tires = struct;
    for i=1:size(tire_fields,2)
        a_tire = one_anno.tire_points.(tire_fields(i));
%             a_tire = round(a_tire); #TODO do you need to round a tire
        if a_tire(1) > 0 || a_tire(2) > 0
            tires.(tire_fields(i)) = [a_tire; 1];
        end
    end
    all_annos.(fieldname).('tires') = tires;
end

for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = annotations.(fieldname);
    'fixing reshape'
    one_anno.extremal_pairs
    [2,size(one_anno.extremal_pairs,2)/2]
    ex_pairs = reshape(one_anno.extremal_pairs,[2,size(one_anno.extremal_pairs,2)/2])
%         ex_pairs = round(ex_pairs);
    ex_pairs = [ex_pairs; ones(1,size(ex_pairs,2))];
    all_annos.(fieldname).('ex') = ex_pairs;
end

%Process of non extremal points
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = annotations.(fieldname);
    all_pairs = zeros(3,size(one_anno.non_extremal_pairs,1)*2);
    for k = 1:size(one_anno.non_extremal_pairs,1)
        a_pair = reshape(one_anno.non_extremal_pairs(k,:),2,[]);
%             a_pair = round(a_pair);
        a_pair = [a_pair; ones(1,size(a_pair,2))];
        all_pairs(:,2*k-1:2*k) =  a_pair;
    end
    all_annos.(fieldname).('nonex') = all_pairs;
end

%% Visualize all annotations THESIS VIZ
if false
    figure;
    hold on
    imshow(image)
    hold on
        for i = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(i);
            fieldname = sprintf("obj_%s",int2str(annotated_car_id))
            tires = all_annos.(fieldname).('tires');
            fields = fieldnames(tires)
            for k = 1:size(fields,1)
                that_tire = string(fields(k));
                scatter(tires.(that_tire)(1),tires.(that_tire)(2),20,'green','filled');
                text(tires.(that_tire)(1)+5,tires.(that_tire)(2)-5,fields(k), 'Color', 'yellow','FontSize',10)
            end
            text(tires.(that_tire)(1)-15,tires.(that_tire)(2)+15,int2str(all_cars_ids(i)), ...
                'Color', 'black','FontSize',15)
        end
    
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i);
        fieldname = sprintf("obj_%s",int2str(annotated_car_id))
        points = all_annos.(fieldname).('ex');
        scatter(points(1,:),points(2,:),30,'red','filled');
        points = all_annos.(fieldname).('nonex');
        scatter(points(1,:),points(2,:),10,"magenta",'filled');
        if isfield(all_annos.(fieldname),'center')
            points = all_annos.(fieldname).('center');
            scatter(points(1,:),points(2,:),10,"cyan",'filled');
        end
    end

%         for k=1:size(all_cars_ids,2)
%             obj_index = all_cars_ids(k)
%             [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, obj_index);
%     %         text(bbox_2D(1)+15,bbox_2D(2)-15,int2str(obj_index), 'Color', 'red','FontSize',10);
%         end

    axis on
    axis equal
end

%% Dictionary for tire avail
id_to_tire =["DF","PF","PR","DR"];

%% Print out algorithmic scenarior first and go from there
%Configuration for each cars
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = all_annos.(fieldname);
%         if isfield(one_anno,'center')
%             all_annos.(fieldname).('has_center') = 1;
%         else
%             all_annos.(fieldname).('has_center') = 0;
%         end
    tires = all_annos.(fieldname).('tires');
    fields = fieldnames(tires);
    string_fields = string(fields);
    all_annos.(fieldname).tire_ids = [];
    for k = 1:size(string_fields,1)
        all_annos.(fieldname).tire_ids = ...
            [all_annos.(fieldname).tire_ids find(id_to_tire == string_fields(k))];
    end
    all_annos.(fieldname).tire_ids = sort(all_annos.(fieldname).tire_ids);
    flag = char(fields(1));
    flag = flag(1);
    all_annos.(fieldname).('tire_both_sides') = 0;
    for k = 1:size(fields,1)
        that_tire = char(fields(k));
        if that_tire(1) ~= flag
            all_annos.(fieldname).('tire_both_sides') = 1; %Check tire using the word
            break
        end
    end
    all_annos.(fieldname).('num_sym_pairs') = size(one_anno.('nonex'),2)/2 ...
        + size(one_anno.('ex'),2)/2;
%         all_annos.(fieldname).('tires_avail') = join(string_fields,'-');
end


%% Find visible
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    all_annos = find_visible(annotated_car_id, all_annos);
end

%% Determine the height of tires
cars_in_roi = [];
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   fieldname = sprintf("obj_%s",int2str(annotated_car_id));
   origin_2D = all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).car_origin));
%        mounting_height = get_height_kernel(fobj, k_matrix, R,origin_2D); %For one tires only
   [in_roi, mounting_height] = get_height_kernel_mean_hw7(fobj, k_matrix, R,id_to_tire, annotated_car_id, all_annos); 
   if in_roi
        cars_in_roi = [cars_in_roi annotated_car_id];
        assert(mounting_height>0);
        all_annos.(fieldname).mounting_height = mounting_height;
   end
end
all_cars_ids = cars_in_roi; %Only evaluate cars in ROI

%% ======= VISUALIZING VP CAR==========
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = all_annos.(fieldname);
    anno_sympoints_homo = [one_anno.ex one_anno.nonex];
    vp_car = get_vpcar_pizlo(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image);
    assert(abs(dot(vp_car,horizon_line)) < 1e-6, 'VP car gotta be on horizon line')
    all_annos.(fieldname).vp_car = vp_car;
end

%% Visualization of found vp_car for one car THESISVIS
if false
    figure;
    hold on
    imshow(image)
    hold on
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i);
%             if annotated_car_id ~= 1
%                 continue
%             end
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        one_anno = all_annos.(fieldname);
%             if isfield(all_annos.(fieldname),'tire_pairs')
%                 anno_sympoints_homo = [one_anno.ex one_anno.nonex one_anno.tire_pairs];
%             else
%                 anno_sympoints_homo = [one_anno.ex one_anno.nonex];
%             end
        vp_car = all_annos.(fieldname).vp_car;

        %Plotting non-extremal
        anno_sympoints_homo = [one_anno.nonex]
        for i = 1:size(anno_sympoints_homo,2)/2
             scatter([anno_sympoints_homo(1,2*i)], [anno_sympoints_homo(2,2*i)],10,'magenta','filled');
             scatter([anno_sympoints_homo(1,2*i-1)], [anno_sympoints_homo(2,2*i-1)],10,'magenta','filled');
        end
        for i = 1:size(anno_sympoints_homo,2)/2
                line([anno_sympoints_homo(1,2*i), anno_sympoints_homo(1,2*i-1)], [anno_sympoints_homo(2,2*i), anno_sympoints_homo(2,2*i-1)],'Color','green','LineWidth',2,'LineStyle','--'); 
%                     if norm(vp_car-one_anno.ex(:,1)) < norm(vp_car-one_anno.ex(:,2))
%                         line([anno_sympoints_homo(1,2*i), vp_car(1,:)], [anno_sympoints_homo(2,2*i), vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
%                     else
%                         line([anno_sympoints_homo(1,2*i-1), vp_car(1,:)], [anno_sympoints_homo(2,2*i-1), vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
%                     end
        end

        %Plotting extremal
        anno_sympoints_homo = [one_anno.ex]
%             scatter(anno_sympoints_homo(1,:),anno_sympoints_homo(2,:),20,'r','filled');
        for i = 1:size(anno_sympoints_homo,2)/2
                scatter([anno_sympoints_homo(1,2*i)], [anno_sympoints_homo(2,2*i)],10,'magenta','filled');
                scatter([anno_sympoints_homo(1,2*i-1)], [anno_sympoints_homo(2,2*i-1)],10,'magenta','filled');
        end
        for i = 1:size(anno_sympoints_homo,2)/2
                line([anno_sympoints_homo(1,2*i), anno_sympoints_homo(1,2*i-1)], [anno_sympoints_homo(2,2*i), anno_sympoints_homo(2,2*i-1)],'Color','green','LineWidth',2,'LineStyle','--'); 
        end
%             line([anno_sympoints_homo(1,2), vp_car(1,1)], [anno_sympoints_homo(2,2), vp_car(2,1)],'Color','yellow','LineWidth',2,'LineStyle','--');
    end

    %             scatter(vp_car(1,:),vp_car(2,:),180,'yellow','filled');
    %Vanishing line
%         scatter(vp_points_2D(1,1),vp_points_2D(2,1),100,'red','filled');
%     scatter(vp_points_2D(1,2),vp_points_2D(2,2),100,'green','filled');
%         scatter(vp_points_2D(1,3),vp_points_2D(2,3),50,'blue','filled');
%         line([vp_points_2D(1,1), vp_points_2D(1,3)],[vp_points_2D(2,1),...
%             vp_points_2D(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
%     line([vp_points_2D(1,2), vp_points_2D(1,3)],[vp_points_2D(2,2),...
%         vp_points_2D(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
    axis equal
%         axis on
%         xlabel('X')
%         ylabel('Y')
end

%% ======= GETTING THE SYMMETRY NORMAL ==========
% Enforcing it going into the car
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = get_sym_normal(annotated_car_id, all_annos,k_matrix,R);
end

%% Localizing planes
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = localize_planes(k_matrix,R,annotated_car_id,all_annos,id_to_tire,image);
end

%%  Locating all symmetry points

for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = sym_points_3D_pizlo(k_matrix,R,annotated_car_id,all_annos,image);
end

%% ALl unit testing
threshold = 1e-6;
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_unit_tests(annotated_car_id,all_annos,id_to_tire,threshold);
end

%% Visualize and calculate reprojection error
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = get_reproj_error(k_matrix,R,annotated_car_id,all_annos,id_to_tire);
end

%% Get wheelbase
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = eval_wheelbase(annotated_car_id,all_annos,id_to_tire);
end

%% IOU Eval
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   use_gaussian = false;
   all_annos = get_pred_bbox(k_matrix,R,annotated_car_id,all_annos,id_to_tire,car_model_param,image);
end

% for i = 1:size(all_cars_ids,2)
%    annotated_car_id = all_cars_ids(i);
%    all_annos = eval_iou(annotated_car_id,all_annos);
% end
% 
% for i = 1:size(all_cars_ids,2)
%    annotated_car_id = all_cars_ids(i);
%    all_annos = eval_width(annotated_car_id,all_annos);
% end

%% Print results
% for i = 1:size(all_cars_ids,2)
%    annotated_car_id = all_cars_ids(i);
%    write_results(fileID,all_annos,annotated_car_id,target,id_to_tire)
% end

%% Do the LM non linear optimization
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
%    if annotated_car_id ~=2
%        continue
%    end
   all_annos = do_opt_hw7(annotated_car_id,all_annos,id_to_tire,horizon_line,k_matrix,R,image,ptCloud_world,car_model_param);
end

%% ALl unit testing run again
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_unit_tests(annotated_car_id,all_annos,id_to_tire,threshold);
end

%% Rerun evaluation agains
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = get_reproj_error_with_3D_points(k_matrix,R,annotated_car_id,all_annos);
end

for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    all_annos = eval_width(annotated_car_id,all_annos);
end

for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = eval_length(annotated_car_id,all_annos);
end

for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = eval_height(annotated_car_id,all_annos);
end

%% Visualize after on 2D images all visualizaiton on 2D images happen here THESIS VIS
edges = [
     1 2;
    2 3;
    3 4;
     4 1;
    5 6;
    6 7;
    7 8;
    8 5;
    1 5;
    2 6;
    3 7;
    4 8;
];
edges_2D = [
         1 3;
         1 2;
         2 4;
         3 4;
    ];


if false
    figure;
    imshow(image)
    hold on
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i);
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),...
            all_annos.(fieldname).all_annotated_points(2,:),15,"red",'filled')
        scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),...
            all_annos.(fieldname).all_3D_points_to_2D(2,:),15,"green",'filled')
        if false
            bbox = all_annos.(fieldname).corners_bbox_world;
            sym_plane_vis = [mean([bbox(:,1), bbox(:,2)],2),...
                mean([bbox(:,5), bbox(:,6)],2),...
                mean([bbox(:,3), bbox(:,4)],2),...
                mean([bbox(:,7), bbox(:,8)],2)];
            sym_plane_vis_2D = k_matrix*R*sym_plane_vis;
            sym_plane_vis_2D = sym_plane_vis_2D./sym_plane_vis_2D(3,:);
            for k = 1:size(edges_2D,1)
                x_coor = [sym_plane_vis_2D(1, edges_2D(k,1)) sym_plane_vis_2D(1, edges_2D(k,2))];
                y_coor = [sym_plane_vis_2D(2, edges_2D(k,1)) sym_plane_vis_2D(2, edges_2D(k,2))];
                line(x_coor, y_coor,'Color','red','LineWidth',1);
            end
        end

        %If outputing sym plan visualization
        if false
            bbox = all_annos.(fieldname).pred_bbox;
            sym_plane_vis = [mean([bbox(:,1), bbox(:,2)],2),...
                mean([bbox(:,5), bbox(:,6)],2),...
                mean([bbox(:,3), bbox(:,4)],2),...
                mean([bbox(:,7), bbox(:,8)],2)];
            sym_plane_vis_2D = k_matrix*R*sym_plane_vis;
            sym_plane_vis_2D = sym_plane_vis_2D./sym_plane_vis_2D(3,:);

            for k = 1:size(edges_2D,1)
                x_coor = [sym_plane_vis_2D(1, edges_2D(k,1)) sym_plane_vis_2D(1, edges_2D(k,2))];
                y_coor = [sym_plane_vis_2D(2, edges_2D(k,1)) sym_plane_vis_2D(2, edges_2D(k,2))];
                line(x_coor, y_coor,'Color','green','LineWidth',1);
            end
        end
        %If output bounding box
        if true
            pred_bbox_2D = k_matrix*R*all_annos.(fieldname).pred_bbox_points_only;
            pred_bbox_2D = pred_bbox_2D./pred_bbox_2D(3,:);
%             scatter(pred_bbox_2D(1,1),pred_bbox_2D(2,1),20,'red','filled');
%             scatter(pred_bbox_2D(1,2),pred_bbox_2D(2,2),20,'green','filled');
%             scatter(pred_bbox_2D(1,3),pred_bbox_2D(2,3),20,'blue','filled');
            for k = 1:size(edges,1)
                x_coor = [pred_bbox_2D(1, edges(k,1)) pred_bbox_2D(1, edges(k,2))];
                y_coor = [pred_bbox_2D(2, edges(k,1)) pred_bbox_2D(2, edges(k,2))];
                line(x_coor, y_coor,'Color','green','LineWidth',1)
            end
        end

        text(all_annos.(fieldname).all_3D_points_to_2D(1,1)-15,all_annos.(fieldname).all_3D_points_to_2D(2,2)+25,int2str(all_cars_ids(i)), ...
        'Color', 'black','FontSize',15)
    end
    if false %If add Lidar points
        within_indice = ptCloud_above_2D(1,:)>=0 & ptCloud_above_2D(1,:)<=size(image,2) & ...
        ptCloud_above_2D(2,:)>=0 & ptCloud_above_2D(2,:)<=size(image,1);
        colors = ptCloud_above(2,within_indice);
        scatter(ptCloud_above_2D(1,within_indice),ptCloud_above_2D(2,within_indice),3,colors,'filled')
        colormap(gca,"parula")
        colorbar
    end

    if false %If add Lidar points
        ptCloud_world_2D = k_matrix*R*ptCloud_world;
        ptCloud_world_2D = ptCloud_world_2D./ptCloud_world_2D(3,:);
        within_indice = ptCloud_world_2D(1,:)>=0 & ptCloud_world_2D(1,:)<=size(image,2) & ...
        ptCloud_world_2D(2,:)>=0 & ptCloud_world_2D(2,:)<=size(image,1);
        colors = ptCloud_world(2,within_indice);
        scatter(ptCloud_world_2D(1,within_indice),ptCloud_world_2D(2,within_indice),3,colors,'filled')
        colormap(gca,"parula")
        colorbar
    end

end

%% Visualize box in 3D 
if false
    figure;
    hold on
    for i = 1:size(all_cars_ids,2)
       annotated_car_id = all_cars_ids(i);
       fieldname = sprintf("obj_%s",int2str(annotated_car_id));
       pcshow([all_annos.(fieldname).pred_bbox(1,:)',all_annos.(fieldname).pred_bbox(2,:)',all_annos.(fieldname).pred_bbox(3,:)'],'red','MarkerSize',300);
%        pcshow([all_annos.(fieldname).corners_bbox_world(1,:)',all_annos.(fieldname).corners_bbox_world(2,:)',all_annos.(fieldname).corners_bbox_world(3,:)'],'red','MarkerSize',500);
       pcshow([all_annos.(fieldname).all_3D_points(1,:)',all_annos.(fieldname).all_3D_points(2,:)',all_annos.(fieldname).all_3D_points(3,:)'],'green','MarkerSize',200);
    end
    pcshow(ptCloud_above',ptCloud_above(2,:)','MarkerSize',10);
    pcshow([0,0,0], 'magenta','MarkerSize',500);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end