%% READ THE CALIBERATION MATRIX
file_name = '000010';
car_index = 1;
near_side = 'driver';
vpcar_point = 'in';
data_path = 'C:\Projects_raw_data\KITTI Dataset\training';
image = imread(sprintf("%s\\image_2\\%s.png",data_path,file_name));

if true
    figure
    imshow(image)
end

%% ======= GET THE VELO TO CAM ROTATION MATRIX ==========
[P2_reshaped, R0_rect_aug, Tr_velo_to_cam_aug] = read_calib(data_path,file_name);

%% ======= READ ALL THE CARS IN ==========
all_cars = read_cars(data_path,file_name);

%% ======= GET ALL THE INTRINSIC AND EXTRINSICS MATRIX ==========
%Generate the translation matrix from cam 0(reference cam to cam2)
k_matrix = P2_reshaped(1:3,1:3);
R = diag(ones(3,1)) %R_cam2_world as you usually know, perfectly aligned

% [U,S,V] = svd(P2_reshaped)
% C = V(1:3,end) / V(end, end) %Normalized
% [K,R] = rq(P2_reshaped(1:3,1:3))
% D = diag(sign(diag(K)));
% K = K*D;
% R = D*R
% K = K/K(end,end)
% t = -R*C

t_cam2_rect = inv(k_matrix)*(P2_reshaped*[0 0 0 1]')
% t_rect_cam2 = -t_cam2_rect
% cam2_to_rect = [diag(ones(3,1)) t_rect_cam2; 0 0 0 1]
% rect_to_cam2 = inv(cam2_to_rect)
R_world_cam2 = inv(R) %%R_cam2_world 
t_world_rect = R_world_cam2*t_cam2_rect
rect_to_world = [diag(ones(3,1)) t_world_rect; 0 0 0 1] 
velo_to_rect = R0_rect_aug*Tr_velo_to_cam_aug
velo_to_world = rect_to_world*velo_to_rect

%% ======= ANNOTATE GUI =======
if false
    edges_2D = [
         1 3;
         1 2;
         2 4;
         3 4;
    ];
    % all_cars = all_cars(1:1,:)
    global annotations 
    annotations = zeros(2, 14*size(all_cars,1))
    f = figure
    imshow(image)
    for k = 1:size(all_cars,1)
        acar = all_cars(k,:);
        bbox_2D = [acar(1) acar(1) acar(3) acar(3); 
            acar(2) acar(4) acar(2) acar(4);
                    ones(1,4)];
        hold on
        scatter(bbox_2D(1,:),bbox_2D(2,:),30,'green','filled'); %Plot the sample 2D
        for i = 1:size(edges_2D,1)
            x_coor = [bbox_2D(1, edges_2D(i,1)) bbox_2D(1, edges_2D(i,2))];
            y_coor = [bbox_2D(2, edges_2D(i,1)) bbox_2D(2, edges_2D(i,2))];
            line(x_coor, y_coor,'Color','red','LineWidth',1);
        end
        global still_moving
        still_moving = true
    
        for j = 1:14
            roi = drawpoint('Color','r');
            annotations(:,14*(k-1)+j) = [roi.Position(1),roi.Position(2)]';
            roi.Label = sprintf('%.0f',j)
            addlistener(roi,'ROIMoved',@(src,eventdata)move_points_callback(src,eventdata,j,k,f))
        end
    
        while still_moving
            uicontrol('Position',[20 75 60 20],'String','Visual check','Callback','uiresume(f)');
            uiwait(f)
            delete(findobj(gca, 'type', 'scatter'));
            scatter(annotations(1,14*(k-1)+1:14*(k-1)+14),annotations(2,14*(k-1)+1:14*(k-1)+14),10,'green','filled');
            c = uicontrol('String','Finished?','Callback', @(src,eventdata)finish_moving_callback(src,eventdata,f));
        end
    
        %Clean up the unknown points
        for h = 14*(k-1)+1:14*(k-1)+14
            tp_x = annotations(1,h);
            tp_y = annotations(2,h);
            if tp_x < min(bbox_2D(1,:)) || tp_x > max(bbox_2D(1,:)) || ...
                 tp_y < min(bbox_2D(2,:)) || tp_y > max(bbox_2D(2,:))
                annotations(:,h) = 0;
            end
        end
        disp('Continue to annotate the next car');
    end
    disp('Finished annotations for all cars');
    
    save(sprintf("data\\annotations\\%s.mat",file_name), 'annotations');
    figure
    imshow(image)
    hold on
    scatter(annotations(1,:),annotations(2,:),20,'green','filled');
end
%% ======= GET THE 3D BOUNDING BOXES OF ONE CAR ==========
% car_index = 1
corners_cam_aug = get_3d_bbox(all_cars, car_index)

corners_bbox_world = rect_to_world*corners_cam_aug
corners_bbox_world = corners_bbox_world(1:3,:)
corners_bbox_2D = k_matrix*R*corners_bbox_world
corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:)

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
    figure;
    hold on
    imshow(image)
    %Yaxis vanishing
    hold on
    % scatter(corners_bbox_2D(1,1),corners_bbox_2D(2,1),50,'blue','filled');
    % scatter(corners_bbox_2D(1,2),corners_bbox_2D(2,2),50,'blue','filled');
    for i = 1:size(edges,1)
        x_coor = [corners_bbox_2D(1, edges(i,1)) corners_bbox_2D(1, edges(i,2))];
        y_coor = [corners_bbox_2D(2, edges(i,1)) corners_bbox_2D(2, edges(i,2))];
        line(x_coor, y_coor,'Color','red','LineWidth',1)
    end
end

%% ======= READ ALL THE VELODYNE FILE IN ==========
velo = get_velo(data_path,file_name);
corners_velo = inv(velo_to_rect)*corners_cam_aug

if false
    figure;
    pcshow([velo(:,1),velo(:,2),velo(:,3)]);
    hold on
    pcshow([corners_velo(1,:)',corners_velo(2,:)',corners_velo(3,:)'],'red','MarkerSize',500);
    title('Sphere with Default Color Map');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end

%Gotta find all velodyne points within this box of corners and then
min_x = min(corners_velo(1,:));
max_x = max(corners_velo(1,:));
min_y = min(corners_velo(2,:));
max_y = max(corners_velo(2,:));
min_z = min(corners_velo(3,:));
max_z = max(corners_velo(3,:));

good_rows = find(velo(:,1) >= min_x & velo(:,1)  <= max_x &...
    velo(:,2) >= min_y & velo(:,2)  <= max_y &...
    velo(:,3) >= min_z & velo(:,3)  <= max_z);

all_car_points = velo(good_rows,1:3)';

%Visualize only the car parts
if true
    figure;
    pcshow([velo(good_rows,1),velo(good_rows,2),velo(good_rows,3)],'MarkerSize',100);
    hold on
    pcshow([corners_velo(1,:)',corners_velo(2,:)',corners_velo(3,:)'],'red','MarkerSize',500);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end

%% ======= GET 2D VANISHING POINTS ==========
vp_points = get_vppoints(k_matrix,R)

% % %======= VISUALIZING VANISHING POINT ==========
% if true
%     figure;
%     hold on
%     imshow(image)
%     hold on
%     scatter(vp_points(1,1),vp_points(2,1),100,'red','filled');
%     scatter(vp_points(1,2),vp_points(2,2),100,'green','filled');
%     scatter(vp_points(1,3),vp_points(2,3),50,'blue','filled');
%     line([vp_points(1,1), vp_points(1,3)],[vp_points(2,1),...
%         vp_points(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
%     line([vp_points(1,2), vp_points(1,3)],[vp_points(2,2),...
%         vp_points(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
%     hold on
%     %%Plot the intersection of all vanishing lines
%     axis equal
%     axis on
%     xlabel('X')
%     ylabel('Y')
% end

%% Get horizon line
horizon_line = cross(vp_points(:,1),vp_points(:,3))


%% ======= 2.1 SETTING UP TIRE GROUND POINT ==========
load(sprintf("data\\annotations\\%s.mat",file_name));
anno = annotations(:,14*(car_index-1)+1:14*car_index)';
near_tground_index = 11;
near_tground_back_index = 14;
near_tground = [anno(near_tground_index,:) 1]';
near_tground_back = [anno(near_tground_back_index,:) 1]';
anno_sympoints = anno(1:size(anno,1)-4,:);

%% ======= GET THE INTERSECTION WITH HORIZON AND TAKE THE MEAN ==========
anno_sympoints_homo = [anno_sympoints ones(size(anno_sympoints,1),1)];
anno_sympoints_homo = anno_sympoints_homo'

vp_car = get_vpcar(anno_sympoints_homo, horizon_line)

%======= VISUALIZING VP CAR==========
if true
    figure
    hold on
    imshow(image)
    hold on
    scatter(anno_sympoints_homo(1,:),anno_sympoints_homo(2,:),40,'r','filled'); %Plot the sample 2D 
    scatter(vp_car(1,:),vp_car(2,:),180,'yellow','filled'); %Plot the sample 2D 
    scatter(near_tground(1,:),near_tground(2,:),40,'green','filled'); %Plot the sample 2D 
    % scatter(near_tground_back(1,:),near_tground_back(2,:),40,'green','filled'); %Plot the sample 2D 
    for i = 1:size(anno_sympoints_homo,2)/2
        line([anno_sympoints_homo(1,2*i), vp_car(1,:)], [anno_sympoints_homo(2,2*i), vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
    end
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end
%% ======= GETTING THE SYMMETRY NORMAL ==========
% If vp car points into the car
% if vp car points out of the car
if  strcmp(vpcar_point,'out')
    sym_normal = -inv(k_matrix*R)*vp_car;
else
    sym_normal = inv(k_matrix*R)*vp_car
end
sym_normal_unit = sym_normal/norm(sym_normal);

%% ========== BEGIN 2.2 ==============
poleheight = 1.74;

%% Looping for noise - Uncomment when mass testing
% height_debug = true;
% annotation_debug = false;
% if annotation_debug || height_debug
%     fileID = fopen('iterations.txt','w');
% end
% aa = 0
% % poleheight_list = [1.37-0.15:0.03:1.37+0.15]
% % poleheight_list = [1.74]
% for i = 1:size(poleheight_list,2)
%     poleheight = poleheight_list(i)
% % for i = 1:1
%     noise_x = 0;
%     noise_y = 0;
%     noise_x_back = 0;
%     noise_y_back  = 0;
% 
%     if annotation_debug
%         rng('shuffle')
%         noise_x = 0;
%         noise_y = 0;
%         noise_x_back = normrnd(0,0.2);
%         noise_y_back = normrnd(0,0.2);
%     end
%     near_tground(1) = near_tground(1)+noise_x;
%     near_tground(2) = near_tground(2)+noise_y;
%     near_tground_back(1) = near_tground_back(1)+noise_x_back;
%     near_tground_back(2) = near_tground_back(2)+noise_y_back;

%% Testing one car only
temp =  inv(k_matrix*R)*near_tground;
lambda = poleheight/temp(2);
near_tground_3D = [temp(1)*lambda poleheight temp(3) * lambda]';

% Test the velo of tires
near_tground_3D_aug = [near_tground_3D; ones(1,size(near_tground_3D,2))];
near_tground_3D_velo = inv(velo_to_world)*near_tground_3D_aug;

% Getting the near plane equation
near_d = -dot(near_tground_3D,sym_normal) %Based on the constrainted that T.Ms = 0
near_plane = [sym_normal; near_d]
near_plane_normalized = near_plane/norm(near_plane(1:3))

% If there is back point, have to do correction here
backpoint = false;
if near_tground_back(1) > 0 & near_tground_back(2) > 0
    backpoint = true;
end

if backpoint
    [near_tground_3D, near_tground_back_3D, near_plane_normalized] = ...
    near_plane_correction(k_matrix,R,sym_normal_unit, near_tground,near_tground_3D,...
    near_tground_back,poleheight);
end


% Test the velo of tires
near_tground_3D_aug = [near_tground_3D; ones(1,size(near_tground_3D,2))]
near_tground_3D_velo = inv(velo_to_world)*near_tground_3D_aug;
if backpoint
    near_tground_back_3D_aug = [near_tground_back_3D; ones(1,size(near_tground_back_3D,2))]
    near_tground_back_3D_velo = inv(velo_to_world)*near_tground_back_3D_aug;
end

if true
    %Visualize only the car parts
    figure;
    pcshow([velo(good_rows,1),velo(good_rows,2),velo(good_rows,3)],'MarkerSize',100);
    hold on
    pcshow([corners_velo(1,:)',corners_velo(2,:)',corners_velo(3,:)'],'red','MarkerSize',500);
    pcshow([near_tground_3D_velo(1,:)',near_tground_3D_velo(2,:)',...
        near_tground_3D_velo(3,:)'],'green','MarkerSize',500);
    % pcshow([mirror_end_velo(1,:)',mirror_end_velo(2,:)',...
    %     mirror_end_velo(3,:)'],'yellow','MarkerSize',500);
    if backpoint
        pcshow([near_tground_back_3D_velo(1,:)',near_tground_back_3D_velo(2,:)',...
            near_tground_back_3D_velo(3,:)'],'green','MarkerSize',500);
    end
    hold on
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end

% ======= USE THE LEFT PLANE TO FIND EXTREMAL POINT ==========
%Get the pair of extremal points
if  strcmp(near_side,'driver')
    near_extremal_index = 2;
    far_extremal_index = 1;
%     else
%         near_extremal_index = 1;
%         far_extremal_index = 2;
end

near_extremal = anno_sympoints_homo(:,near_extremal_index);
far_extremal = anno_sympoints_homo(:,far_extremal_index);
[w_unit, near_extremal_3D, far_extremal_3D] = get_extremal_3D(k_matrix, R, near_extremal, far_extremal, near_plane_normalized);
sym_plane_2ndway_normalized = [near_plane_normalized(1:3); near_plane_normalized(4)-w_unit/2];


% Test the velo of extremal
near_extremal_3D_aug = [near_extremal_3D; ones(1,size(near_extremal_3D,2))]
near_extremal_3D_velo = inv(velo_to_world)*near_extremal_3D_aug;
far_extremal_3D_aug = [far_extremal_3D; ones(1,size(far_extremal_3D,2))]
far_extremal_3D_velo = inv(velo_to_world)*far_extremal_3D_aug;

%% Visualize only the tire and first extremal
if true
    figure;
    pcshow([velo(good_rows,1),velo(good_rows,2),velo(good_rows,3)],'MarkerSize',100);
    hold on
    pcshow([corners_velo(1,:)',corners_velo(2,:)',corners_velo(3,:)'],'red','MarkerSize',500);
    pcshow([near_tground_3D_velo(1,:)',near_tground_3D_velo(2,:)',...
        near_tground_3D_velo(3,:)'],'green','MarkerSize',500);
    pcshow([near_tground_back_3D_velo(1,:)',near_tground_back_3D_velo(2,:)',...
        near_tground_back_3D_velo(3,:)'],'green','MarkerSize',500);
    pcshow([near_extremal_3D_velo(1,:)',near_extremal_3D_velo(2,:)',...
        near_extremal_3D_velo(3,:)'],'green','MarkerSize',800);
    pcshow([far_extremal_3D_velo(1,:)',far_extremal_3D_velo(2,:)',...
        far_extremal_3D_velo(3,:)'],'green','MarkerSize',500);
    hold on
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end

% Project on to sym 
ds = near_plane_normalized(4)-w_unit/2;
if backpoint
    tire_extremal = [near_extremal_3D far_extremal_3D near_tground_3D near_tground_back_3D];
else
    tire_extremal = [near_extremal_3D far_extremal_3D near_tground_3D];
end

tire_extremal_aug = [tire_extremal; ones(1,size(tire_extremal, 2))];
cMw = get_cMw(sym_normal_unit,tire_extremal);
tire_extremal_sym_coor = cMw*tire_extremal_aug;

tire_extremal_on_sym = project_onto_sym(tire_extremal_sym_coor(1:3,:),ds, [0 0 1]');

all_car_points_aug = [all_car_points; ones(1,size(all_car_points, 2))];
lidar_points_world = velo_to_world*all_car_points_aug;
lidar_points_sym_coor = cMw*lidar_points_world;
lidar_points_on_sym = project_onto_sym(lidar_points_sym_coor(1:3,:),ds, [0 0 1]');

corners_bbox_world = rect_to_world*corners_cam_aug;
corners_bbox_sym_coor = cMw*corners_bbox_world;
corners_bbox_world = corners_bbox_world(1:3,:);
corners_bbox_on_sym = project_onto_sym(corners_bbox_sym_coor(1:3,:),ds, [0 0 1]');


if true
    figure;
    hold on
    pcshow([tire_extremal_sym_coor(1,:)',tire_extremal_sym_coor(2,:)',tire_extremal_sym_coor(3,:)'],'green','MarkerSize',500);
    pcshow([lidar_points_sym_coor(1,:)',lidar_points_sym_coor(2,:)',...
        lidar_points_sym_coor(3,:)'],'blue','MarkerSize',300);
    pcshow([corners_bbox_sym_coor(1,:)',corners_bbox_sym_coor(2,:)',corners_bbox_sym_coor(3,:)'],'red','MarkerSize',500);

    hold on
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end


if true
    figure;
    txt = ['\leftarrow y = ' num2str(-poleheight) 'm'];
    t = text(tire_extremal_on_sym(1,3),tire_extremal_on_sym(2,3)-0.02,txt);
    t.FontSize = 14;
    hold on
    set(gca, 'YDir','reverse')
%     set(gca, 'XDir','reverse')
    scatter(lidar_points_on_sym(1,:),lidar_points_on_sym(2,:),5,'blue','filled');
    scatter(corners_bbox_on_sym(1,:),corners_bbox_on_sym(2,:),15,'red','filled');
    scatter(tire_extremal_on_sym(1,:),tire_extremal_on_sym(2,:),30,'green','filled');
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end
%% Figure out the plane equation for the far plane
far_plane_normalized = [near_plane_normalized(1:3); near_plane_normalized(4)-w_unit];

%% ========== VERIFYING 2.2 SYM NORMAL SOLUTION ==========
near_extremal_3D_to_2D = k_matrix*R*near_extremal_3D;
near_extremal_3D_to_2D = near_extremal_3D_to_2D/near_extremal_3D_to_2D(3);
far_extremal_3D_to_2D = k_matrix*R*far_extremal_3D;
far_extremal_3D_to_2D = far_extremal_3D_to_2D/far_extremal_3D_to_2D(3);

% shifted_far_sym_normal = far_extremal_normal
% shifted_far_sym_normal_2D = k_matrix*R*(shifted_far_sym_normal)
% shifted_far_sym_normal_2D = shifted_far_sym_normal_2D/shifted_far_sym_normal_2D(3)

%Doing the symmetry normal for the visualization
shifted_sym_normal = sym_normal_unit;
shifted_sym_normal_2D = k_matrix*R*(shifted_sym_normal);
shifted_sym_normal_2D = shifted_sym_normal_2D/shifted_sym_normal_2D(3);

%Connect a point to a pair of extremal

if true
    figure;
    %hold on
    imshow(image)
    hold on
    scatter(vp_car(1,1),vp_car(2,1),400,'yellow','filled');
    % scatter(anno_sympoints_homo(1,7:8),anno_sympoints_homo(2,7:8),80,'red','filled'); %Plot the sample 2D 
    % scatter(near_tground(1,:),near_tground(2,:),40,'blue','filled'); %Plot the sample 2D 
    scatter(near_extremal_3D_to_2D(1,:),near_extremal_3D_to_2D(2,:),40,'blue','filled');
    scatter(far_extremal_3D_to_2D(1,:),far_extremal_3D_to_2D(2,:),40,'blue','filled'); 
    % line([anno_sympoints_homo(1,7), shifted_far_sym_normal_2D(1)],...
    %     [anno_sympoints_homo(2,7),shifted_far_sym_normal_2D(2)],...
    %     'Color','green','LineWidth',1,'LineStyle','--')
    line([near_extremal_3D_to_2D(1), shifted_sym_normal_2D(1)],[near_extremal_3D_to_2D(2)...
        ,shifted_sym_normal_2D(2)],'Color','red','LineWidth',1,'LineStyle','--')
    %%Plot the intersection of all vanishing lines
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end

%% ========== PART 3 - FOR ALL THE POINTS ==========
%DIVERGE IF ELSE
if  strcmp(near_side,'driver')
    non_extremal_indice = [4,3;
                           6,5;
                           8,7;
                           10,9];
%     else
%         non_extremal_indice = [3,4;
%                                5,6;
%                                7,8;
%                                9,10];
end


all_3d_points = zeros(size(anno_sympoints_homo,1),size(anno_sympoints_homo,2));
all_3d_points(:,near_extremal_index) = near_extremal_3D;
all_3d_points(:,far_extremal_index) = far_extremal_3D;

all_non_extremal_points = [];
non_extremal_separations = zeros(1, size(non_extremal_indice,1));
%PROCESSING FOR NONEXTREMAL INDICES

vpa_cannot = false;
for k = 1: size(non_extremal_indice,1)
%     disp('Processing')
%     non_extremal_indice(k,1)
%     non_extremal_indice(k,2)
    non_ex1_l = anno_sympoints(non_extremal_indice(k,1),:)';
    non_ex1_r = anno_sympoints(non_extremal_indice(k,2),:)';
    %Get the centroid of it
    non_ex1_s = mean([non_ex1_l , non_ex1_r], 2);
    %Find rectified symmetry points
    non_ex1_vline = vp_car(1:2) - non_ex1_s; %Symmetry line
    non_ex1_near_proj = non_ex1_l - non_ex1_s;
    non_ex1_far_proj = non_ex1_r - non_ex1_s;
    non_ex1_l_rec = dot(non_ex1_near_proj,non_ex1_vline)...
        /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s;
    non_ex1_r_rec = dot(non_ex1_far_proj,non_ex1_vline)...
        /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s;
    non_ex1_l_rec = [non_ex1_l_rec; 1]; %Augment it
    non_ex1_r_rec = [non_ex1_r_rec; 1];
    %Optimization process
    w_hat = w_unit;
    lambda_l = (-ds - w_hat/2)/dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_l_rec);
    lambda_r = (-ds + w_hat/2)/dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_r_rec);

    init_guess = [lambda_l; lambda_r];
    syms lambda_l lambda_r;
    eqn1 = lambda_r*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_r_rec)...
        == -ds + 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec - lambda_l*inv(k_matrix*R)*non_ex1_l_rec);
    eqn2 = lambda_l*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_l_rec)...
        == -ds - 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec - lambda_l*inv(k_matrix*R)*non_ex1_l_rec);
    sol = vpasolve([eqn1, eqn2], [lambda_l, lambda_r], init_guess);
    lambda_l = sol.lambda_l;
    lambda_r = sol.lambda_r;

    if size(lambda_l,1) == 0
        lambda_l = (-ds - w_hat/2)/dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_l_rec);
        lambda_r = (-ds + w_hat/2)/dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_r_rec);
        X_final = fsolve(@(x)dfitgamma(x,k_matrix,R,non_ex1_l_rec,...
            non_ex1_r_rec,sym_normal_unit,ds),[lambda_l lambda_r 0 0],...
            optimset('display','off','TolFun',1e-9, 'Algorithm','trust-region-dogleg'));
        lambda_l = X_final(1);
        lambda_r = X_final(2);
    end

%     if size(lambda_l,1) == 0
%         vpa_cannot = true;
% %         break;
%     end 

    non_ex1_l_3D = lambda_l*inv(k_matrix*R)*non_ex1_l_rec;
    non_ex1_r_3D = lambda_r*inv(k_matrix*R)*non_ex1_r_rec;
    non_extremal_separations(k) = norm(non_ex1_r_3D-non_ex1_l_3D);

    all_3d_points(:,non_extremal_indice(k,1)) = non_ex1_l_3D; %Save the generated points
    all_3d_points(:,non_extremal_indice(k,2)) = non_ex1_r_3D;
    all_non_extremal_points = [all_non_extremal_points non_ex1_l_3D];
    all_non_extremal_points = [all_non_extremal_points non_ex1_r_3D];
end
% if vpa_cannot
%     continue;
% end

%% Check initial guesses
if backpoint
    all_3d_points_tire = [all_3d_points near_tground_3D near_tground_back_3D];
else
    all_3d_points_tire = [all_3d_points near_tground_3D];
end

%% Unit testing assertion
threshold = 1e-12;
assert(abs(all_3d_points_tire(2,11) - poleheight) < threshold, 'Tire contact must be on ground');
assert(abs(all_3d_points_tire(2,12) - poleheight) < threshold, 'Tire contact must be on ground');

assert(abs(sum(near_plane_normalized(1:3) == sym_plane_2ndway_normalized(1:3)) - 3) < threshold,...
    'Left/right plane has to be parallel to sym plane');
assert(abs(sum(near_plane_normalized(1:3) == far_plane_normalized(1:3)) - 3) < threshold,...
        'Left/right plane has to be parallel to sym plane');
assert(abs((near_plane_normalized(4) + far_plane_normalized(4))/2 -...
    sym_plane_2ndway_normalized(4)) < threshold, ...
    'Left/right plane has to be equally displaced from sym plane');
near_plane_normalized(4)
dot(all_3d_points_tire(:,11), sym_normal_unit)
assert(abs(dot(all_3d_points_tire(:,11), sym_normal_unit)+ near_plane_normalized(4))...
    < threshold,'near tire point on near plane');
assert(abs(dot(all_3d_points_tire(:,2), sym_normal_unit)+ near_plane_normalized(4))...
    < threshold,'near extremal point on near plane');
assert(abs(dot(all_3d_points_tire(:,12), sym_normal_unit)+ near_plane_normalized(4))...
    < threshold,'near tire point back on near plane');
assert(abs(dot(all_3d_points_tire(:,1), sym_normal_unit)+ far_plane_normalized(4))...
    < threshold,'far extremal point on far plane');
all_3d_points_tire
for k = 1: size(non_extremal_indice,1)
    non_ex1_l_3D = all_3d_points_tire(:,non_extremal_indice(k,1));
    non_ex1_r_3D = all_3d_points_tire(:,non_extremal_indice(k,2));
    assert(abs(dot(sym_normal_unit, non_ex1_r_3D-non_ex1_l_3D)-...
        norm(non_ex1_r_3D-non_ex1_l_3D))<threshold, 'Non extremal points on sym line');
    assert(abs(dot(sym_normal_unit, non_ex1_r_3D+non_ex1_l_3D)+2*...
        sym_plane_2ndway_normalized(4))<threshold,...
        'symmetry point pairs has to be equally displaced from sym plane');
end
%% Visualize on 2D
all_3d_points_to_2D = k_matrix*R*all_3d_points_tire;
all_3d_points_to_2D = all_3d_points_to_2D./all_3d_points_to_2D(3,:); %Normalize by third dimension 
if backpoint
    anno_sympoints_homo_and_ground = [anno_sympoints_homo near_tground near_tground_back];
else
    anno_sympoints_homo_and_ground = [anno_sympoints_homo near_tground];
end

reproj_error = ((sum((all_3d_points_to_2D-anno_sympoints_homo_and_ground).^2,'all'))...
    /size(anno_sympoints_homo_and_ground,2))...
    .^(0.5);

%Check alignment of initial guess
if true
    figure;
    hold on
    imshow(image)
    %Yaxis vanishing
    hold on
    scatter(anno_sympoints_homo_and_ground(1,:),anno_sympoints_homo_and_ground(2,:),40,'r','filled'); %Plot the sample 2D 
    scatter(all_3d_points_to_2D(1,:),all_3d_points_to_2D(2,:),20,'blue','filled');
    % axis equal
    % axis on
    % xlabel('X')
    % ylabel('Y')
end

%% Visualize all points 1D
% all_3d_points_tire_sym = project_onto_sym(all_3d_points_tire,ds, sym_normal_unit);
% all_3d_points_tire_sym_aug = [all_3d_points_tire_sym; ones(1,size(all_3d_points_tire_sym,2))]

all_3d_points_tire_aug = [all_3d_points_tire; ones(1,size(all_3d_points_tire,2))]
all_3d_points_tire_sym_coor = cMw*all_3d_points_tire_aug
all_3d_points_tire_on_sym = project_onto_sym(all_3d_points_tire_sym_coor(1:3,:),ds, [0 0 1]');

if true
    figure;
    txt = ['\leftarrow y = ' num2str(-poleheight) 'm'];
    t = text(tire_extremal_sym_coor(1,3),tire_extremal_sym_coor(2,3)-0.02,txt);
    t.FontSize = 14;
    hold on
    set(gca, 'YDir','reverse')
%     set(gca, 'XDir','reverse')
    scatter(lidar_points_sym_coor(1,:),lidar_points_sym_coor(2,:),3,'blue','filled');
    scatter(corners_bbox_sym_coor(1,:),corners_bbox_sym_coor(2,:),15,'red','filled');
    scatter(all_3d_points_tire_on_sym(1,:),all_3d_points_tire_on_sym(2,:),40,'green','filled');
    axis equal
    axis on
    xlabel('X')
    ylabel('Y')
end

%% Evaluate angle
[sym_with_x, bbox_with_x, angle_difference] = ...
    get_angle_error(corners_bbox_world,sym_normal_unit,near_side);

%% Evaluate position
% 
% [dist_to_mid_wheelbase_sym, dist_to_bbox_center, dist_difference] = ...
%     get_distance_error(corners_bbox_world,all_3d_points_tire_on_sym)

%% Evaluate width
% [extremal_width, bbox_width, width_difference] = get_width_error(all_3d_points_tire,corners_bbox_world);

%% Evaluatae wheelbase
if backpoint
    wheelbase = norm(near_tground_3D-near_tground_back_3D)
end

%% Evaluate iou
% cMw_tire = get_cMw(sym_normal_unit,[near_tground_3D;1])
all_3d_points_tire_tire_coor = cMw*[all_3d_points_tire; ones(1,size(all_3d_points_tire,2))];

predicted_corners_tire_coor = get_predicted_bbox(all_3d_points_tire_tire_coor);
predicted_corners = inv(cMw)*predicted_corners_tire_coor;
predicted_corners = predicted_corners(1:3,:);

predicted_ground_corners = [predicted_corners(1,1:4);predicted_corners(3,1:4)]
truth_ground_corners = [corners_bbox_world(1,1:4);corners_bbox_world(3,1:4)]


predicted_ground_corners = [0 0 1 1; 0 1 1 0]
truth_ground_corners =  [0 -0.5 0.5 3; 0 0.5 0.5 0.2]

figure
hold on
scatter(predicted_ground_corners(1,:), predicted_ground_corners(2,:),20,'green','filled')
scatter(truth_ground_corners(1,:), truth_ground_corners(2,:),20,'red','filled')


[xi,yi] = polyxpoly(predicted_ground_corners(1,:),predicted_ground_corners(2,:),...
    truth_ground_corners(1,:),truth_ground_corners(2,:))

polyarea(xi,yi)

area_intersection = get_area_intersection(predicted_ground_corners,truth_ground_corners)

height_overlap = get_height_overlap(predicted_corners, corners_bbox_world);

iou = get_iou_3D(area_intersection,height_overlap,corners_bbox_world,predicted_corners);
iou_bev = get_iou_bev(area_intersection,corners_bbox_world,predicted_corners);


%% Visualize your box
corners_bbox_2D = k_matrix*R*corners_bbox_world;
corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:);

predicted_corners_bbox_2D = k_matrix*R*predicted_corners;
predicted_corners_bbox_2D = predicted_corners_bbox_2D./predicted_corners_bbox_2D(3,:);

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
    figure;
    hold on
    imshow(image)
    %Yaxis vanishing
    hold on
    scatter(anno_sympoints_homo_and_ground(1,:),anno_sympoints_homo_and_ground(2,:),40,'yellow','filled'); %Plot the sample 2D 
    % scatter(all_3d_points_to_2D(1,:),all_3d_points_to_2D(2,:),20,'blue','filled');
    % scatter(corners_bbox_2D(1,1),corners_bbox_2D(2,1),50,'blue','filled');
    % scatter(corners_bbox_2D(1,2),corners_bbox_2D(2,2),50,'blue','filled');
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

%% Visualize initial guess 3D
all_3d_points_tire_aug = [all_3d_points_tire; ones(1,size(all_3d_points_tire,2))];
all_3d_points_tire_velo = inv(velo_to_world)*all_3d_points_tire_aug;

if true
    %Visualize only the car parts
    figure;
    pcshow([velo(good_rows,1),velo(good_rows,2),velo(good_rows,3)]);
    hold on
    pcshow([corners_velo(1,:)',corners_velo(2,:)',corners_velo(3,:)'],'red','MarkerSize',500);
    pcshow([all_3d_points_tire_velo(1,:)',all_3d_points_tire_velo(2,:)',...
        all_3d_points_tire_velo(3,:)'],'green','MarkerSize',500);
    hold on
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end

to_write = [poleheight non_extremal_separations	reproj_error sym_with_x bbox_with_x ...
    angle_difference dist_to_mid_wheelbase_sym 	dist_to_bbox_center ...
dist_difference	extremal_width 	bbox_width 	width_difference ...
wheelbase	iou	iou_bev]
% noise_x noise_y noise_x_back noise_y_back
%     if height_debug
%         fprintf(fileID,'%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n',to_write);

if height_debug || annotation_debug
    fprintf(fileID,'%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n',to_write);
end
% end

%% Before optimization - Test for offsets 
car_origin = all_3d_points_tire(:,11);
world2car_R = get_world2car_rotation(car_origin, sym_normal_unit)
world2car_T = -car_origin
all_3d_points_car = world2car_R*(all_3d_points_tire+world2car_T)


%% ========== PART 5 - REFINE THE POINTS ========== 
options = optimoptions('lsqnonlin','Display','iter');
options.StepTolerance = 1e-10;
options.FunctionTolerance = 1e-10;
options.Algorithm = 'levenberg-marquardt'
options.MaxFunctionEvaluations = 1.8e+05
options.MaxIterations = 1.0e+10
options

theta0 = [vp_car(1),vp_car(2),vp_car(3),w_unit,sym_plane_2ndway_normalized(4)...
    non_extremal_separations(1),...
    non_extremal_separations(2),...
    non_extremal_separations(3),...
    non_extremal_separations(4)]';
% Pass all the thing you can optimize through theta0
% Along with all the other things to calculate the reprojection error
% Return the optimal theta
[theta_final,resnorm,residual,exitflag,output]= lsqnonlin(@(p) lsqFun(p,k_matrix, R,anno_sympoints_homo_and_ground,...
    all_3d_points, non_extremal_indice,poleheight,backpoint), theta0,[],[],options);


%% ========== EVALUATION USING THE REFINED PARAMETERS ========== 
%Get the new parameter in
theta = theta_final;
%Calculate the new estimate
all_refined_points_3D = get_new_estimate(theta,k_matrix,R,anno_sympoints_homo_and_ground,...
    all_3d_points,non_extremal_indice, poleheight,backpoint);
%Calculate the prediction
all_refined_points = k_matrix*R*all_refined_points_3D;
all_refined_points = all_refined_points./all_refined_points(3,:);
%Calculate error
num_points = 12
disp('After LSQnonlin error')
fErr = sum(([anno_sympoints_homo_and_ground(:,:)]-[all_refined_points(:,:)]).^2,'all');
fErr = (fErr/num_points).^(0.5)

disp('Initial guess error')
initial_error = sum(([anno_sympoints_homo_and_ground(:,:)]-[all_3d_points_to_2D(:,:)]).^2,'all');
initial_error = (initial_error/num_points).^(0.5)

figure
hold on
imshow(image)
%Yaxis vanishing
hold on
scatter(anno_sympoints_homo_and_ground(1,:),anno_sympoints_homo_and_ground(2,:),40,'r','filled'); %Plot the sample 2D 
scatter(all_refined_points(1,:),all_refined_points(2,:),30,'green','filled');
% scatter(all_3d_points_to_2D(1,:),all_3d_points_to_2D(2,:),20,'blue','filled');
% axis equal
% axis on
% xlabel('X')
% ylabel('Y')

%% ========== OFFSETS AFTER OPTIMIZATIONS ========== 
vp_car_final = [theta_final(1),theta_final(2),1]';
vp_car_final(1) = vp_car_final(1)
sym_normal_final = inv(k_matrix*R)*vp_car_final;
sym_normal_final_unit = sym_normal_final/norm(sym_normal_final);
car_origin_final = all_refined_points_3D(:,11);
world2car_R_final = get_world2car_rotation(car_origin_final, sym_normal_final_unit);
world2car_T_final = -car_origin_final;
all_refined_points_car = world2car_R_final *(all_refined_points_3D +world2car_T_final );

%% =============== DO THE ROTATION AND DISPLAY 1D ===============
% back_near_point_car = [all_refined_points_car(1,11) all_refined_points_car(2,11) all_refined_points_car(3,2)]'
% all_near_points_car = [all_refined_points_car(:,2),all_refined_points_car(:,4),...
%     all_refined_points_car(:,6),all_refined_points_car(:,8),all_refined_points_car(:,10),...
%     all_refined_points_car(:,11) back_near_point_car]
% %Scatter plot based on yz
% figure
% hold on
% set(gca, 'YDir','reverse')
% % set(gca, 'XDir','reverse')
% scatter(all_near_points_car(3,1:6),all_near_points_car(2,1:6),120,'green','filled');
% for i = 1:size(all_near_points_car,2)-1
%     line([all_near_points_car(3,i), all_near_points_car(3,i+1)],[all_near_points_car(2,i),...
%      all_near_points_car(2,i+1)],'Color','blue','LineWidth',3,'LineStyle','--')
% end 
% axis equal
% axis on
% xlabel('Z')
% ylabel('Y')


%% KITTI 
% all_refined_points_3D
% 
% %Distance to the mean of all refined points - centroid of them
% mid_near = mean(all_refined_points_3D(:,11:12),2)
% %Get the refined sym plane
% vp_car = [theta(1),theta(2),1]';
% sym_normal = inv(k_matrix*R)*vp_car;
% sym_normal_unit = sym_normal/norm(sym_normal);corners_cam_aug
% 
% ds = theta(5);
% car_centroid = mid_near + (-ds-dot(mid_near,sym_normal_unit))*sym_normal_unit
% distance_to_car_centroid = norm(car_centroid)
% car_centroid_velo = inv(velo_to_world)*[car_centroid; 1];
% 
% 
% %Distance to the center of bounding_box
% corners_world = rect_to_world*corners_cam_aug;
% corners_world = corners_world(1:3,:);
% corners_world_center = mean(corners_world(:,1:4), 2);
% distance_to_car_bbox = norm(corners_world_center)
% corners_center_velo = inv(velo_to_world)*[corners_world_center; 1];
% 
% disp('Based on centroid')
% rmse = sqrt((distance_to_car_bbox-distance_to_car_centroid)^2)



%% Visualize in LiDAR space
% lidar_car_centroid = mean(all_car_points)'
% lidar_car_centroid = [lidar_car_centroid; 1]
% rect_to_world = [diag(ones(3,1)) t_vector; 0 0 0 1]
% velo_to_world = rect_to_world * Tr_velo_to_cam_aug
% lidar_car_centroid_world =  velo_to_world * lidar_car_centroid
% lidar_car_centroid_world = lidar_car_centroid_world(1:3)


% all_refined_points_3D_aug = [all_refined_points_3D; ones(1,size(all_refined_points_3D,2))]
% all_refined_points_3D_velo = inv(velo_to_world)*all_refined_points_3D_aug
% 
% 
% %Visualize only the car parts
% figure;
% pcshow([velo(good_rows,1),velo(good_rows,2),velo(good_rows,3)]);
% hold on
% pcshow([corners_velo(1,:)',corners_velo(2,:)',corners_velo(3,:)'],'red','MarkerSize',500);
% pcshow([all_refined_points_3D_velo(1,:)',all_refined_points_3D_velo(2,:)',...
%     all_refined_points_3D_velo(3,:)'],'green','MarkerSize',500);
% hold on
% pcshow([car_centroid_velo(1),car_centroid_velo(2),car_centroid_velo(3)],'blue','MarkerSize',800);
% pcshow([corners_center_velo(1),corners_center_velo(2),corners_center_velo(3)],'magenta','MarkerSize',800);
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');


% a = max(all_refined_points_3D_velo(1,:))
% b = min(all_refined_points_3D_velo(1,:))
% c = max(all_refined_points_3D_velo(2,:))
% d = min(all_refined_points_3D_velo(2,:))
% e = max(all_refined_points_3D_velo(3,:))
% f = min(all_refined_points_3D_velo(3,:))
% 
% corners_predicted = [a a a a b b b b;
%                      c d c d c d c d;
%                      e e f f e e f f;]
 

% %Visualize only the car parts
% figure;
% pcshow([velo(good_rows,1),velo(good_rows,2),velo(good_rows,3)]);
% hold on
% pcshow([corners_velo(1,:)',corners_velo(2,:)',corners_velo(3,:)'],'red','MarkerSize',500);
% pcshow([all_refined_points_3D_velo(1,:)',all_refined_points_3D_velo(2,:)',...
%     all_refined_points_3D_velo(3,:)'],'green','MarkerSize',500);
% % pcshow([corners_predicted(1,:)',corners_predicted(2,:)',...
% %     corners_predicted(3,:)'],'yellow','MarkerSize',500);
% hold on
% pcshow([lidar_car_centroid(1),lidar_car_centroid(2),lidar_car_centroid(3)],'magenta','MarkerSize',800);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');



