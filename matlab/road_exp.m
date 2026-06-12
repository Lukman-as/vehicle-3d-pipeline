%% READ THE raw_image File
drive_name = 'single-infrastructure-side-image'
file_name = '001566';
data_path = '/mnt/c/Projects_raw_data/ROAD';
raw_image = imread(sprintf("/%s/%s/%s.jpg",data_path,drive_name,file_name));

if false
    figure
    imshow(raw_image)
end

image = raw_image;

%% ======= GET THE EXTRINSICS, INTRINSICS ==========
file = sprintf("%s/single-infrastructure-side/calib/camera_intrinsic/%s.json",data_path,file_name);
fid = fopen(file);
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
intrinsics_data = jsondecode(str)
k_matrix = reshape(intrinsics_data.cam_K,[3,3])';


file = sprintf("%s/single-infrastructure-side/calib/virtuallidar_to_camera/%s.json",data_path,file_name)
fid = fopen(file)
raw = fread(fid,inf);
str = char(raw'); 
fclose(fid); 
extrinsics_data = jsondecode(str); 

rot = extrinsics_data.rotation;
t = extrinsics_data.translation;
M_cam_velo = [rot,t;0 0 0 1];


rot(:,1) = rot(:,1)/norm(rot(:,1));
rot(:,2) = rot(:,2)/norm(rot(:,2));
rot(:,3) = rot(:,3)/norm(rot(:,3));


a = 1/8*rot*(3*rot'*rot*rot'*rot-10*rot'*rot+15*diag(ones(1,3)));
assert(abs(det(a)-1)<1e-5);
assert(norm(a-rot,'fro')<5e-2);
    
M_nvelo_velo = [inv(a)*extrinsics_data.rotation, [0 0 0]';0 0 0 1];
M_cam_nvelo = [a, t; 0 0 0 1];

assert(norm(M_cam_nvelo*M_nvelo_velo-M_cam_velo)<1e-6);
R_nvelo_world = [[0 -1 0]' [0 0 -1]' [1 0 0]']; %TO make y axis vertical
assert(det(R_nvelo_world)==1);

R_cam_nvelo = M_cam_nvelo(1:3,1:3);
M_cam_world = [R_cam_nvelo*R_nvelo_world,[0 0 0]'; 0 0 0 1];
M_world_nvelo = M_cam_world\M_cam_nvelo;

assert(norm(M_cam_world*M_world_nvelo-M_cam_nvelo,"fro")<1e-6);

a = cross(k_matrix*M_cam_world(1:3,1),k_matrix*M_cam_world(1:3,3));
a = a./a(3);
b = cross(k_matrix*M_cam_nvelo(1:3,1),k_matrix*M_cam_nvelo(1:3,2));%XY axis
b = b./b(3);
assert(norm(a-b,'fro') < 1e-5, 'Minimal difference in the horizon line');

vec = -M_world_nvelo(1:3,3);
vec/norm(vec);
rad2deg(acos(dot(vec/norm(vec),[0,1,0]')));
assert(rad2deg(acos(dot(vec/norm(vec),[0,1,0]')))<0.1,'Y_axis of velo alignment with world x z plane');

M_world_velo = M_world_nvelo * M_nvelo_velo; %It is okay for this to not have det 1
det(M_world_velo);
assert(norm(M_cam_world*M_world_velo-M_cam_velo,'fro')<1e-6);


 %% ======= SETTING UP YOUR OWN WORLD ======== %%
R_cam_world = M_cam_world(1:3,1:3);
R = R_cam_world;
assert(abs(det(R)-1)<1e-5);
assert(k_matrix(1,2)==0);
    

%% ======= GET LABELS IN ==========
file = sprintf("%s/single-infrastructure-side/label/camera/%s.json",data_path,file_name)
raw_labels = read_cars_road(file)

%% ======= GET THE 3D BOUNDING BOXES OF ONE CAR ==========
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


if true
    figure
    hold on
    imshow(image)
    %Yaxis vanishing
    hold on
    for k=1:size(raw_labels.valid_ids,2)
        obj_index = raw_labels.valid_ids(k)
        [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, obj_index)
        
        corners_bbox_world = M_world_velo*corners_velo_aug
        corners_bbox_2D = k_matrix*R*corners_bbox_world(1:3,:);
        corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:);

        scatter(corners_bbox_2D(1,1),corners_bbox_2D(2,1),50,'red','filled');
        scatter(corners_bbox_2D(1,2),corners_bbox_2D(2,2),50,'green','filled');
        scatter(corners_bbox_2D(1,3),corners_bbox_2D(2,3),50,'blue','filled');
        for i = 1:size(edges,1)
            x_coor = [corners_bbox_2D(1, edges(i,1)) corners_bbox_2D(1, edges(i,2))];
            y_coor = [corners_bbox_2D(2, edges(i,1)) corners_bbox_2D(2, edges(i,2))];
            line(x_coor, y_coor,'Color','red','LineWidth',1)
        end
        for i = 1:size(edges_2D,1)
                x_coor = [bbox_2D(1, edges_2D(i,1)) bbox_2D(1, edges_2D(i,2))];
                y_coor = [bbox_2D(2, edges_2D(i,1)) bbox_2D(2, edges_2D(i,2))];
                line(x_coor, y_coor,'Color','white','LineWidth',1);
        end
        text(bbox_2D(1)+15,bbox_2D(2)-15,int2str(obj_index), 'Color', 'yellow','FontSize',8);
    end
end

%% Read LIDAR
file = sprintf("%s/single-infrastructure-side-velodyne/%s.pcd",data_path,file_name)
ptCloud = pcread(file)
ptCloud_world = [ptCloud.Location';ones(1,size(ptCloud.Location,1))];
% ptCloud_world = M_world_velo*[ptCloud.Location';ones(1,size(ptCloud.Location,1))];
% ptCloud_world = M_cam_velo*[ptCloud.Location';ones(1,size(ptCloud.Location,1))];
ptCloud_world = ptCloud_world(1:3,:)'
maxDistance = 0.05;
referenceVector = [0,0,1];
maxAngularDistance = 2;
ptCloud = pointCloud(ptCloud_world);
[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
        maxDistance,referenceVector,maxAngularDistance);
remainPtCloud = select(ptCloud,outlierIndices);
plane1 = select(ptCloud,inlierIndices);
ptCloud_above = remainPtCloud.Location;
ptCloud_2D_above = M_cam_velo*[ptCloud_above';ones(1,size(ptCloud_above,1))]
ptCloud_ground = plane1.Location;
ptCloud_2D_ground = M_cam_velo*[ptCloud_ground';ones(1,size(ptCloud_ground,1))]



if false
%     Checking transform to cam 
    figure
    hold on
%     pcshow(ptCloud_world, 'blue','MarkerSize',20);
    pcshow(ptCloud_2D_ground(1:3,:)', 'blue','MarkerSize',20);
    pcshow(ptCloud_2D_above(1:3,:)', 'green','MarkerSize',20);
%     for k=1:size(all_cars_ids,2)
%         obj_index = all_cars_ids(k)
%         [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, obj_index);
%         corners_cam_aug = M_cam_velo*corners_velo_aug;
%         pcshow(corners_cam_aug(1:3,:)', 'red','MarkerSize',50);
%     end
    for k=1:size(raw_labels.valid_ids,2)
        obj_index = raw_labels.valid_ids(k)
        [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, obj_index);
        corners_cam_aug = M_cam_velo*corners_velo_aug;
        pcshow(corners_cam_aug(1:3,:)', 'red','MarkerSize',50);
    end
end

if false
    img_resized = imresize(image, [1100 size(image,2)]);
    imshowpair(image, img_resized);
    tiledlayout(2,1);
    nexttile
    imshow(image);
    nexttile
    imshow(img_resized);
    hold off
    image = img_resized;
end

if true
    k_matrix_fixed = k_matrix;
    ptCloud_2D_above = ptCloud_2D_above(1:3,:)
    ptCloud_2D_above = k_matrix_fixed*ptCloud_2D_above;
    ptCloud_2D_above = ptCloud_2D_above./ptCloud_2D_above(3,:);
%     ptCloud_2D_above(1,:) = ptCloud_2D_above(1,:)-k_matrix(1,3)+960
%     ptCloud_2D_above(2,:) = ptCloud_2D_above(2,:)-k_matrix(2,3)+540

    ptCloud_2D_ground = ptCloud_2D_ground(1:3,:);
    ptCloud_2D_ground = k_matrix_fixed*ptCloud_2D_ground;
    ptCloud_2D_ground = ptCloud_2D_ground./ptCloud_2D_ground(3,:);
    figure;
    hold on
    imshow(image)
    hold on
    scatter(ptCloud_2D_above(1,:),ptCloud_2D_above(2,:),2,"green",'filled')
    scatter(k_matrix_fixed(1,3),k_matrix_fixed(2,3),30,"red",'filled')
    scatter(size(image,2)/2,size(image,1)/2,30,"yellow",'filled')
%     for i = 1:size(all_cars_ids,2)
%        annotated_car_id = all_cars_ids(i);
%        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
%         corners_bbox_world = all_annos.(fieldname).corners_bbox_world(1:3,:);
%         corners_bbox_2D = k_matrix*R*corners_bbox_world;
%         corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:);
%         for i = 1:size(edges,1)
%             x_coor = [corners_bbox_2D(1, edges(i,1)) corners_bbox_2D(1, edges(i,2))];
%             y_coor = [corners_bbox_2D(2, edges(i,1)) corners_bbox_2D(2, edges(i,2))];
%             line(x_coor, y_coor,'Color','red','LineWidth',1)
%         end
%     end

    for k=1:size(raw_labels.valid_ids,2)
        obj_index = raw_labels.valid_ids(k)
        [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, obj_index);

        corners_bbox_world = M_world_velo*corners_velo_aug;
        corners_bbox_world = R*corners_bbox_world(1:3,:);
        corners_bbox_2D = k_matrix*corners_bbox_world;
        corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:)
        for i = 1:size(edges,1)
            x_coor = [corners_bbox_2D(1, edges(i,1)) corners_bbox_2D(1, edges(i,2))];
            y_coor = [corners_bbox_2D(2, edges(i,1)) corners_bbox_2D(2, edges(i,2))];
            line(x_coor, y_coor,'Color','red','LineWidth',1)
        end
    end
    scatter(ptCloud_2D_ground(1,:),ptCloud_2D_ground(2,:),2,"blue",'filled')
    axis on
    axis equal
end








%% =======  Get horizon line ======= 
vp_points = get_vppoints(k_matrix,R)
vp_points_2D = vp_points./vp_points(3,:)

% ======= VISUALIZING VANISHING POINT ==========
if true
    figure;
    hold on
    imshow(image)
    hold on
    scatter(vp_points_2D(1,1),vp_points_2D(2,1),100,'red','filled');
    scatter(vp_points_2D(1,2),vp_points_2D(2,2),100,'green','filled');
    scatter(vp_points_2D(1,3),vp_points_2D(2,3),50,'blue','filled');
    line([vp_points_2D(1,1), vp_points_2D(1,3)],[vp_points_2D(2,1),...
        vp_points_2D(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
    line([vp_points_2D(1,2), vp_points_2D(1,3)],[vp_points_2D(2,2),...
        vp_points_2D(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
    hold on
    %%Plot the intersection of all vanishing lines
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end

%% Get horizon line
horizon_line = cross(vp_points(:,1),vp_points(:,3))
horizon_line./horizon_line(3)

a = cross(vp_points(:,3),vp_points(:,2))
a./a(3)

%% ======= GET ANNOTATION POINT IN AND ASSERT IN THE 2D BOX ==========
file = sprintf("%s/single-infrastructure-side/annotation/%s.txt",data_path,file_name)
[num_cars_anno, annotations]= read_annotations_rope(file)

all_cars_ids = 1:num_cars_anno
% all_cars_ids = all_cars_ids(all_cars_ids~=2)
% all_cars_ids = all_cars_ids(all_cars_ids~=8)
all_cars_annotations = struct

% Process all the car tires
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id))
    one_anno = annotations.(fieldname)
    tire_fields = ["DR","PR","DF","PF"]
    tires = struct;
    for i=1:size(tire_fields,2)
        a_tire = one_anno.tire_points.(tire_fields(i));
        a_tire = round(a_tire)
        if a_tire(1) > 0 || a_tire(2) > 0
            tires.(tire_fields(i)) = [a_tire; 1];
        end
    end
    all_annos.(fieldname).('tires') = tires
end

for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id))
    one_anno = annotations.(fieldname)
    ex_pairs = reshape(one_anno.extremal_pairs,[2,size(one_anno.extremal_pairs,2)/2]);
    ex_pairs = round(ex_pairs)
    ex_pairs = [ex_pairs; ones(1,size(ex_pairs,2))];
    all_annos.(fieldname).('ex') = ex_pairs
end

%Process of non extremal points
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i);
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = annotations.(fieldname);
    all_pairs = zeros(3,size(one_anno.non_extremal_pairs,1)*2);
    for k = 1:size(one_anno.non_extremal_pairs,1)
        a_pair = reshape(one_anno.non_extremal_pairs(k,:),2,[]);
        a_pair = round(a_pair);
        a_pair = [a_pair; ones(1,size(a_pair,2))];
        all_pairs(:,2*k-1:2*k) =  a_pair;
    end
    all_annos.(fieldname).('nonex') = all_pairs;
end

% Process center points
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id))
    one_anno = annotations.(fieldname);
    if size(one_anno.center_points, 1) > 0
        pairs = [one_anno.center_points ones(size(one_anno.center_points,1),1)]';
        pairs = round(pairs);
        all_annos.(fieldname).('center') = pairs
    end
end

%% Visualize all annotations
if true
    figure
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
            text(tires.(that_tire)(1)+3,tires.(that_tire)(2)-3,fields(k), 'Color', 'yellow','FontSize',6)
        end
        text(tires.(that_tire)(1)-25,tires.(that_tire)(2)+15,int2str(annotated_car_id), ...
            'Color', 'yellow','FontSize',10)
    end

    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i);
        fieldname = sprintf("obj_%s",int2str(annotated_car_id))
        points = all_annos.(fieldname).('ex');
        scatter(points(1,:),points(2,:),15,'red','filled');
        points = all_annos.(fieldname).('nonex');
        scatter(points(1,:),points(2,:),10,"magenta",'filled');
        if isfield(all_annos.(fieldname),'center')
            points = all_annos.(fieldname).('center');
            scatter(points(1,:),points(2,:),10,"cyan",'filled');
        end
    end
    for k=1:size(raw_labels.valid_ids,2)
        obj_index = raw_labels.valid_ids(k);
        [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, obj_index);
        text(bbox_2D(1)+15,bbox_2D(2)-15,int2str(obj_index), 'Color', 'red','FontSize',10);
    end
end

%% Manual mapping fornow
preid_to_gtid = [26,16,22,52,35,3,41,18,10]
% preid_to_gtid = [17,9,5,13,24,27]

for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = get_bbox_world_road(all_annos, annotated_car_id,preid_to_gtid,raw_labels,M_world_velo);
end

%% Dictionary for tire avail
id_to_tire =["DF","PF","PR","DR"]


%% Print out algorithmic scenarior first and go from there
%Configuration for each cars
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id))
    one_anno = all_annos.(fieldname)
    if isfield(one_anno,'center')
        all_annos.(fieldname).('has_center') = 1;
    else
        all_annos.(fieldname).('has_center') = 0;
    end
    tires = all_annos.(fieldname).('tires');
    fields = fieldnames(tires)
    string_fields = string(fields)
    all_annos.(fieldname).tire_ids = [];
    for k = 1:size(string_fields,1)
        all_annos.(fieldname).tire_ids = ...
            [all_annos.(fieldname).tire_ids find(id_to_tire == string_fields(k))]
    end
    all_annos.(fieldname).tire_ids = sort(all_annos.(fieldname).tire_ids)
    flag = char(fields(1));
    flag = flag(1);
    all_annos.(fieldname).('tire_both_sides') = 0;
    for k = 1:size(fields,1)
        that_tire = char(fields(k));
        if that_tire(1) ~= flag
            all_annos.(fieldname).('tire_both_sides') = 1;
            break
        end
    end
    all_annos.(fieldname).('num_sym_pairs') = size(one_anno.('nonex'),2)/2 ...
        + size(one_anno.('ex'),2)/2;
    all_annos.(fieldname).('tires_avail') = join(string_fields,'-');
end

%% Show the configuration
params = ["has_center","tire_both_sides","num_sym_pairs","tires_avail"]
size(params)
for i = 1:size(all_cars_ids,2)
    annotated_car_id = all_cars_ids(i)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = all_annos.(fieldname);
    for k = 1:size(params,2)
        params(k)
        all_annos.(fieldname).(params(k))
    end
end

%% Add tires to vp_car
mounting_height = 7;
% for i = 1:size(all_cars_ids,2)
%     annotated_car_id = all_cars_ids(i)
%     all_annos = add_tire_pairs(annotated_car_id, all_annos,id_to_tire,k_matrix,R,mounting_height)
% end

% for i = 1:size(all_cars_ids,2)
%     annotated_car_id = all_cars_ids(i)
%     all_annos = add_tire_pairs_horizon(annotated_car_id, all_annos,id_to_tire);
% end

if true
    figure
    hold on
    imshow(image)
    hold on
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i)
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        if isfield(all_annos.(fieldname),'tire_pairs')
            line([all_annos.(fieldname).tire_pairs(1,1), all_annos.(fieldname).tire_pairs(1,2)],[all_annos.(fieldname).tire_pairs(2,1),...
                all_annos.(fieldname).tire_pairs(2,2)],'Color','g','LineWidth',2,'LineStyle','--')
        end
    end
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end

%% ======= VISUALIZING VP CAR==========
if true
    figure
    hold on
    imshow(image)
    hold on
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i)
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        one_anno = all_annos.(fieldname);
        if isfield(all_annos.(fieldname),'tire_pairs')
            anno_sympoints_homo = [one_anno.ex one_anno.nonex one_anno.tire_pairs];
        else
            anno_sympoints_homo = [one_anno.ex one_anno.nonex];
        end
%         if isfield(all_annos.(fieldname),'tire_pairs_horizon')
%            anno_sympoints_homo = [anno_sympoints_homo one_anno.tire_pairs_horizon];
%         end
        vp_car = get_vpcar(anno_sympoints_homo, horizon_line)
%         vpcar_collins = inv(k_matrix*R)*get_vpcar_collins(anno_sympoints_homo, image);
%         vpcar_collins = k_matrix*R*[vpcar_collins(1) 0 vpcar_collins(3)]';
%         vpcar_collins = vpcar_collins./vpcar_collins(3)
        vp_car = get_vpcar_collins(anno_sympoints_homo, image,vp_points);
        all_annos.(fieldname).vp_car = vp_car;

        scatter(anno_sympoints_homo(1,:),anno_sympoints_homo(2,:),10,'r','filled');
        scatter(vp_car(1,:),vp_car(2,:),180,'yellow','filled');
        for i = 1:size(anno_sympoints_homo,2)/2
            if norm(vp_car-one_anno.ex(:,1)) < norm(vp_car-one_anno.ex(:,2))
                line([anno_sympoints_homo(1,2*i), vp_car(1,:)], [anno_sympoints_homo(2,2*i), vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
            else
                line([anno_sympoints_homo(1,2*i-1), vp_car(1,:)], [anno_sympoints_homo(2,2*i-1), vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
            end
        end
    end
    %Vanishing line
    scatter(vp_points_2D(1,1),vp_points_2D(2,1),100,'red','filled');
    scatter(vp_points_2D(1,2),vp_points_2D(2,2),100,'green','filled');
    scatter(vp_points_2D(1,3),vp_points_2D(2,3),50,'blue','filled');
    line([vp_points_2D(1,1), vp_points_2D(1,3)],[vp_points_2D(2,1),...
        vp_points_2D(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
    line([vp_points_2D(1,2), vp_points_2D(1,3)],[vp_points_2D(2,2),...
        vp_points_2D(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end

%% ======= GETTING THE SYMMETRY NORMAL ==========
% Enforcing it going into the car
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i)
   all_annos = flip_sym_normal(annotated_car_id, all_annos,id_to_tire,k_matrix,R);
end

%% Visualize to check Find tire ground and check sym normal
if true
    figure
    hold on
    imshow(image)
    hold on
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i)
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        near_tground = all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).car_origin));
        temp =  inv(k_matrix*R)*near_tground;
        lambda =  all_annos.(fieldname).mounting_height/temp(2);
        near_tground_3D = [temp(1)*lambda all_annos.(fieldname).mounting_height temp(3) * lambda]';
        near_tground_3D_image = k_matrix*R*near_tground_3D;;
        near_tground_3D_image = near_tground_3D_image./near_tground_3D_image(3);
        
        sym_normal_unit = all_annos.(fieldname).sym_normal_unit
        sym_normal_shifted = near_tground_3D+sym_normal_unit*4;
        sym_normal_shifted_image = k_matrix*R*sym_normal_shifted;
        sym_normal_shifted_image = sym_normal_shifted_image./sym_normal_shifted_image(3);
        scatter(near_tground_3D_image(1,:),near_tground_3D_image(2,:),40,'green','filled'); %Plot the sample 2D 
        line([near_tground_3D_image(1), sym_normal_shifted_image(1)], ...
            [near_tground_3D_image(2), sym_normal_shifted_image(2)],'Color','green','LineWidth',1.5,'LineStyle','--');
    end
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end

%% Localizing planes
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i)
   all_annos = localize_planes(k_matrix,R,annotated_car_id,all_annos,id_to_tire,mounting_height,image);
end

%% Locating all symmetry points
% annotated_car_id = 8
% fieldname = sprintf("obj_%s",int2str(annotated_car_id));
% all_annos.(fieldname)
% all_annos = sym_points_3D(k_matrix,R,annotated_car_id,all_annos,image,mounting_height);


for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i)
   all_annos = sym_points_3D(k_matrix,R,annotated_car_id,all_annos,image,mounting_height);
end

%% Locating center points
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = center_points_3D(k_matrix,R,annotated_car_id,all_annos)
end

%% ALl unit testing
threshold = 1e-2;
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_unit_tests(k_matrix,R,annotated_car_id,all_annos,id_to_tire,mounting_height,threshold);
end

%% Visualize and calculate reprojection error
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = get_reproj_error(k_matrix,R,annotated_car_id,all_annos,id_to_tire);
end

if true
    figure
    hold on
    imshow(image)
    hold on
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i);
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),...
            all_annos.(fieldname).all_annotated_points(2,:),10,"red",'filled')
        scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),...
            all_annos.(fieldname).all_3D_points_to_2D(2,:),10,"green",'filled')
    end
end

%% Get wheelbase
for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = eval_wheelbase(annotated_car_id,all_annos,id_to_tire);
end

%% IOU Eval
% annotated_car_id = 7;
% fieldname = sprintf("obj_%s",int2str(annotated_car_id));
% get_pred_corners(annotated_car_id,all_annos,id_to_tire);
% 

for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i)
   all_annos = get_pred_corners(annotated_car_id,all_annos,id_to_tire);
end



for i = 1:size(all_cars_ids,2)
   annotated_car_id = all_cars_ids(i);
   all_annos = eval_iou(annotated_car_id,all_annos);
end

%% Visualize on 2D ground plane
edges_2D = [
         1 4;
         1 2;
         2 3;
         3 4;
    ];
if true
    figure;
    hold on
    for i = 1:size(all_cars_ids,2)
       annotated_car_id = all_cars_ids(i);
       fieldname = sprintf("obj_%s",int2str(annotated_car_id));
       scatter(all_annos.(fieldname).truth_ground_corners(1,:),...
            all_annos.(fieldname).truth_ground_corners(2,:),10,"red",'filled')
        scatter(all_annos.(fieldname).predicted_ground_corners(1,:),...
            all_annos.(fieldname).predicted_ground_corners(2,:),10,"green",'filled')
        for i = 1:size(edges_2D,1)
            x_coor = [all_annos.(fieldname).truth_ground_corners(1, edges_2D(i,1)) all_annos.(fieldname).truth_ground_corners(1, edges_2D(i,2))];
            y_coor = [all_annos.(fieldname).truth_ground_corners(2, edges_2D(i,1)) all_annos.(fieldname).truth_ground_corners(2, edges_2D(i,2))];
            line(x_coor, y_coor,'Color','red','LineWidth',1);
        end
        for i = 1:size(edges_2D,1)
            x_coor = [all_annos.(fieldname).predicted_ground_corners(1, edges_2D(i,1)) all_annos.(fieldname).predicted_ground_corners(1, edges_2D(i,2))];
            y_coor = [all_annos.(fieldname).predicted_ground_corners(2, edges_2D(i,1)) all_annos.(fieldname).predicted_ground_corners(2, edges_2D(i,2))];
            line(x_coor, y_coor,'Color','green','LineWidth',1);
        end
        text(all_annos.(fieldname).predicted_ground_corners(1,1)+2,...
            all_annos.(fieldname).predicted_ground_corners(2,1)-2,int2str(annotated_car_id), 'Color', 'black','FontSize',8);
    end
    axis equal
    axis on
    xlabel('X')
    ylabel('Z')

end



%% Visualize box in 3D
% annotated_car_id = 8;
% fieldname = sprintf("obj_%s",int2str(annotated_car_id));
if true
    figure;
    hold on
    for i = 1:size(all_cars_ids,2)
       annotated_car_id = all_cars_ids(i);
       fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        pcshow([all_annos.(fieldname).predicted_corners(1,:)',all_annos.(fieldname).predicted_corners(2,:)',all_annos.(fieldname).predicted_corners(3,:)'],'yellow','MarkerSize',500);
        pcshow([all_annos.(fieldname).corners_bbox_world(1,:)',all_annos.(fieldname).corners_bbox_world(2,:)',all_annos.(fieldname).corners_bbox_world(3,:)'],'red','MarkerSize',500);
        pcshow([all_annos.(fieldname).all_3D_points(1,:)',all_annos.(fieldname).all_3D_points(2,:)',all_annos.(fieldname).all_3D_points(3,:)'],'green','MarkerSize',300);

    end
    pcshow(ptCloud_world,'blue','MarkerSize',10);
%     view(180,135)
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    hold off
end

%% Visualize your box
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

if true
    figure
    imshow(image)
    hold on
    for i = 1:size(all_cars_ids,2)
       annotated_car_id = all_cars_ids(i);
       fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        corners_bbox_world = all_annos.(fieldname).corners_bbox_world(1:3,:);
        corners_bbox_2D = k_matrix*R*corners_bbox_world;
        corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:);
        predicted_corners = all_annos.(fieldname).predicted_corners(1:3,:);
        predicted_corners_bbox_2D = k_matrix*R*predicted_corners;
        predicted_corners_bbox_2D = predicted_corners_bbox_2D./predicted_corners_bbox_2D(3,:);
        scatter(all_annos.(fieldname).all_annotated_points(1,:),all_annos.(fieldname).all_annotated_points(2,:),20,'red','filled');
        scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),all_annos.(fieldname).all_3D_points_to_2D(2,:),10,'green','filled');

%         scatter(corners_bbox_2D(1,1),corners_bbox_2D(2,1),20,'red','filled');
%         scatter(corners_bbox_2D(1,2),corners_bbox_2D(2,2),20,'green','filled');
%         scatter(corners_bbox_2D(1,3),corners_bbox_2D(2,3),20,'blue','filled');
%         scatter(predicted_corners_bbox_2D(1,1),predicted_corners_bbox_2D(2,1),20,'red','filled');
%         scatter(predicted_corners_bbox_2D(1,2),predicted_corners_bbox_2D(2,2),20,'green','filled');
%         scatter(predicted_corners_bbox_2D(1,3),predicted_corners_bbox_2D(2,3),20,'blue','filled');
        for i = 1:size(edges,1)
            x_coor = [corners_bbox_2D(1, edges(i,1)) corners_bbox_2D(1, edges(i,2))];
            y_coor = [corners_bbox_2D(2, edges(i,1)) corners_bbox_2D(2, edges(i,2))];
            line(x_coor, y_coor,'Color','red','LineWidth',1)
        end
        for i = 1:size(edges,1)
            x_coor = [predicted_corners_bbox_2D(1, edges(i,1)) predicted_corners_bbox_2D(1, edges(i,2))];
            y_coor = [predicted_corners_bbox_2D(2, edges(i,1)) predicted_corners_bbox_2D(2, edges(i,2))];
            line(x_coor, y_coor,'Color','green','LineWidth',1)
        end
    end
    for i = 1:size(all_cars_ids,2)
        annotated_car_id = all_cars_ids(i)
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
        near_tground_3D = all_annos.(fieldname).all_3D_points(:,1);
        near_tground_3D_image = k_matrix*R*near_tground_3D;
        near_tground_3D_image = near_tground_3D_image./near_tground_3D_image(3);
        
        sym_normal_unit = all_annos.(fieldname).sym_normal_unit
        sym_normal_shifted = near_tground_3D+sym_normal_unit*4;
        sym_normal_shifted_image = k_matrix*R*sym_normal_shifted;
        sym_normal_shifted_image = sym_normal_shifted_image./sym_normal_shifted_image(3);
%         scatter(near_tground_3D_image(1,:),near_tground_3D_image(2,:),40,'green','filled'); %Plot the sample 2D 
%         line([near_tground_3D_image(1), sym_normal_shifted_image(1)], ...
%             [near_tground_3D_image(2), sym_normal_shifted_image(2)],'Color','yellow','LineWidth',1.5,'LineStyle','--');
        text(all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).tire_ids(1)))(1)-25,...
            all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).tire_ids(1)))(2)+35,int2str(annotated_car_id), ...
                'Color', 'yellow','FontSize',10)
    end
%     axis equal
%     axis on
%     xlabel('X')
%     ylabel('Y')
end