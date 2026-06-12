load('Q1798zoom0angle1_fisheye_params.mat');
image = imread("v.png");
%Showing distorted image
format long
%Before doing anything gotta correct the distortion first
[undistim,camIntrinsics] = undistortFisheyeImage(image,...
    Q1798zoom0angle1_fisheye.Intrinsics,'OutputView','valid');

%======= VISUALIZING DISTORT AND UNDISSTORT ==========
% imshowpair(image, undistim)
% tiledlayout(2,1);
% nexttile
% imshow(image)
% nexttile
% imshow(undistim)

%======= GET ROTATION MATRIX THROUGH VANISHING POINT ==========
% Provided by Yiming CVPR paper
load('cameraExtrinsic.mat');
vp_3D =vp_info.vp;
vp_3D

%======= GET INTRINSIC MATRIX ==========
load('model3D.mat'); %Template
focal=camIntrinsics.FocalLength(1);
pp=camIntrinsics.PrincipalPoint;
pp
pixelSize=0.001;

%You gotta go from World -> Optical -> Pixel, but this is compact form you
%go directly from 3D world to 2D pixel
% vppoints_2D=projectworld2Image(vp_3D',focal*pixelSize,pp,pixelSize)';
% vppoints_2D

%You dont have to convert the unit because it is already in pixels
%Watch out for changing directio of y from world to picture frame
k_matrix = [focal, 0, pp(1); 0, focal, pp(2); 0, 0, 1];
%k_matrix = [1/pixelSize, 0, pp(1); 0, 1/pixelSize, pp(2); 0, 0, 1];

%This kmatrix is already in pixel value, no need to use pixel size, Yiming
%code has this part redundant 
k_matrix

%======= BUG FIXING - DISCUSSION WITH DR ELDER ==========
vp_3D = [vp_3D(1,:); -vp_3D(2,:); vp_3D(3,:)]
vp_3D(:,2) = -vp_3D(:,2)
vp_3D

%======= BEGIN SECTION 1 - GET 2D VANISHING POINTS ==========
count = 3;
vppoints_2D_ver2 = zeros(3,3);
for i = 1:count
    temp = k_matrix*vp_3D(:,i); %Multiply with K for each point
    vppoints_2D_ver2(:,i) = temp/temp(3); %Divided by z, maintain homo form
end
vppoints_2D_ver2

%======= GET THE INTERSECTION WITH HORIZON AND TAKE THE MEAN ==========
horizon_line = cross(vppoints_2D_ver2(:,1),vppoints_2D_ver2(:,3))

%Load the sample 2D in
load('sample2Dpoints.mat'); 
%load('sample2Dpoints_refined.mat'); 

% sample2D = all_refined_points(1:2,1:10)'
% tireGround = all_refined_points(1:2,11)'
% save('sample2Dpoints_refined.mat','sample2D','tireGround')

sample2D_homo = [sample2D ones(size(sample2D,1),1)];
sample2D_homo = sample2D_homo';

%Find all the intersection
all_intersections = zeros(3,size(sample2D_homo,2)/2);
for i = 1:size(sample2D_homo,2)/2
    formed_line = cross(sample2D_homo(:,2*i-1), sample2D_homo(:,2*i)); %Forms from a pair of point
    intersection = cross(formed_line, horizon_line);
    intersection = intersection/intersection(3); %converting each homo to euclidean
    all_intersections(:,i) = intersection;
end
all_intersections
vp_car = mean(all_intersections, 2) %Taking the mean of all points 

%======= SETTING UP TIRE GROUND POINT ==========
left_tground = [1167.9 488.4 1]';
right_tground = [1261.32 460.82 1]'; %Eyeballing the second tire ground
left_tground
right_tground

backpoint = false

%======= VISUALIZING VANISHING POINT ==========
% figure
% hold on
% imshow(undistim)
% hold on
% scatter(vppoints_2D_ver2(1,1),vppoints_2D_ver2(2,1),100,'red','filled');
% % scatter(vppoints_2D_ver2(1,2),vppoints_2D_ver2(2,2),100,'green','filled');
% scatter(vppoints_2D_ver2(1,3),vppoints_2D_ver2(2,3),100,'blue','filled');
% line([vppoints_2D_ver2(1,1), vppoints_2D_ver2(1,3)],[vppoints_2D_ver2(2,1),...
%      vppoints_2D_ver2(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
% % hold on
% scatter(sample2D_homo(1,7:8),sample2D_homo(2,7:8),70,'y','filled'); %Plot the sample 2D 
% scatter(sample2D_homo(1,7:8),sample2D_homo(2,7:8),30,'r','filled'); %Plot the sample 2D 
% scatter(left_tground(1,:),left_tground(2,:),70,'green','filled'); %Plot the sample 2D 
% %%Plot the intersection of all vanishing lines
% scatter(vp_car(1,:),vp_car(2,:),100,'black','filled'); %Plot the sample 2D 
% lines
% axis equal
% axis on
% xlabel('X')
% ylabel('Y')

%======= GETTING THE SYMMETRY NORMAL ==========
%Get the symmetry plane orientation
R = vp_3D;
sym_normal = inv(k_matrix*R)*vp_car


%======= SECTION 2.1 ==========
%Get the left and right tire contact point
poleheight = 4.8;
temp =  inv(k_matrix*R)*left_tground;
lambda = poleheight/temp(2);
left_tground_3D = [temp(1)*lambda poleheight temp(3) * lambda];
left_tground_3D

temp =  inv(k_matrix*R)*right_tground;
lambda = poleheight/temp(2);
right_tground_3D = [temp(1)*lambda poleheight temp(3) * lambda];
right_tground_3D
%Find the symmetry point by taking the mean of the two above
sym_point_3D = mean([left_tground_3D ; right_tground_3D])' 
sym_point_2D = k_matrix*R*sym_point_3D
sym_point_2D = sym_point_2D/sym_point_2D(3)

%Selecting alpha to fix a b c
%Solvf for d through constraint T.M = 0, meaning lying on the plane itself
d = -dot(sym_point_3D,sym_normal); %Based on the constrainted that T.Ms = 0
sym_plane = [sym_normal; d];
sym_plane_normalized = sym_plane/norm(sym_plane(1:3));

%Doing the symmetry normal for the visualization
shifted_sym_normal = sym_normal
shifted_sym_normal_2D = k_matrix*R*shifted_sym_normal
shifted_sym_normal_2D = shifted_sym_normal_2D/shifted_sym_normal_2D(3)

%Visualize the vanishing points and sym normal
figure
%scatter(vphorizon(1),vphorizon(2),400,'blue','filled');
%hold on
imshow(undistim)
hold on
% scatter(vppoints_2D_ver2(1,1),vppoints_2D_ver2(2,1),400,'red','filled');
% %scatter(vppoints_2D_ver2(1,2),vppoints_2D_ver2(2,2),400,'yellow','filled');
% %%Yaxis vanishing
% scatter(vppoints_2D_ver2(1,3),vppoints_2D_ver2(2,3),400,'blue','filled');
% line([vppoints_2D_ver2(1,1), vppoints_2D_ver2(1,3)],[vppoints_2D_ver2(2,1),...
%     vppoints_2D_ver2(2,3)],'Color','g','LineWidth',2,'LineStyle','--')
% line([left_tground(1), shifted_sym_normal_2D(1)],[left_tground(2),shifted_sym_normal_2D(2)],'Color','green','LineWidth',2,'LineStyle','--')
% hold on
scatter(sample2D_homo(1,:),sample2D_homo(2,:),100,'r','filled'); %Plot the sample 2D 
scatter(left_tground(1,:),left_tground(2,:),100,'blue','filled'); %Plot the sample 2D 
scatter(right_tground(1,:),right_tground(2,:),100,'blue','filled'); %Plot the sample 2D 
%%Plot the intersection of all vanishing lines
% scatter(vp_car(1,:),vp_car(2,:),400,'yellow','filled'); %Plot the sample 2D 
% lines
% axis equal
% axis on
% xlabel('X')
% ylabel('Y')

%========== BEGIN 2.2 ==========
%======= FIND LEFT PLANE FIRST ==========
%Get the left plane equation 
left_tground_3D = left_tground_3D';
left_d = -dot(left_tground_3D,sym_normal); %Based on the constrainted that T.Ms = 0
left_plane = [sym_normal; left_d]

%======= USE THE LEFT PLANE TO FIND EXTREMAL POINT ==========
%Get the pair of extremal points
left_extremal = sample2D_homo(:,8)
right_extremal = sample2D_homo(:,7)
left_extremal_proj_dir = inv(k_matrix*R)*left_extremal;
left_lambda = -left_plane(4)/dot(left_extremal_proj_dir,left_plane(1:3));
left_extremal_3D = left_extremal_proj_dir*left_lambda; %Should this be left lambda????
%Based on the formulation that left_extremal dot Mleft = 0 
%Formulation direction of right point
right_extremal_proj_dir = inv(k_matrix*R)*right_extremal;
right_extremal_normal = cross(right_extremal_proj_dir, cross(sym_normal, right_extremal_proj_dir));

%Find the w THIS W HAVE TO REVERSE FIND OUT WHY - EQ 10
right_extremal_w = -dot(right_extremal_normal, left_extremal_3D) / dot(right_extremal_normal, sym_normal);

%Find the extremal point
w_unit = right_extremal_w*norm(sym_normal); %Fix this thing
sym_normal_unit = sym_normal/norm(sym_normal);
% right_extremal_3D = left_extremal_3D + right_extremal_w*sym_normal
right_extremal_3D = left_extremal_3D + w_unit*sym_normal_unit

%Normalize normal and w
%Figure out the plane based on track wwidth
left_plane_normalized = left_plane/norm(left_plane(1:3))
right_plane_normalized = [left_plane_normalized(1:3); left_plane_normalized(4)-w_unit]
sym_plane_2ndway_normalized = [left_plane_normalized(1:3); left_plane_normalized(4)-w_unit/2]
left_extremal_3D_to_2D = k_matrix*R*left_extremal_3D
left_extremal_3D_to_2D = left_extremal_3D_to_2D/left_extremal_3D_to_2D(3)
right_extremal_3D_to_2D = k_matrix*R*right_extremal_3D
right_extremal_3D_to_2D = right_extremal_3D_to_2D/right_extremal_3D_to_2D(3)

%========== VERIFYING 2.2 SYM NORMAL SOLUTION ==========
%Shifted to right for visualization
shifted_right_sym_normal = right_extremal_normal
shifted_right_sym_normal_2D = k_matrix*R*shifted_right_sym_normal
shifted_right_sym_normal_2D = shifted_right_sym_normal_2D/shifted_right_sym_normal_2D(3)

%Doing the symmetry normal for the visualization
shifted_sym_normal = sym_normal
shifted_sym_normal_2D = k_matrix*R*shifted_sym_normal
shifted_sym_normal_2D = shifted_sym_normal_2D/shifted_sym_normal_2D(3)

%Visualize those points on image to confirm
% figure
% %hold on
% imshow(undistim)
% hold on
% % scatter(vp_car(1,1),vp_car(2,1),400,'yellow','filled');
% scatter(sample2D_homo(1,5),sample2D_homo(2,5),60,'blue','filled'); %Plot the sample 2D 
% % scatter(left_tground(1,:),left_tground(2,:),40,'blue','filled'); %Plot the sample 2D 
% % scatter(normal_cross_right_project_2D(1,:),normal_cross_right_project_2D(2,:),40,'blue','filled'); %Plot the sample 2D 
% scatter(left_extremal_3D_to_2D(1,:),left_extremal_3D_to_2D(2,:),40,'red','filled');
% scatter(right_extremal_3D_to_2D(1,:),right_extremal_3D_to_2D(2,:),40,'green','filled'); 
% line([sample2D_homo(1,5), shifted_right_sym_normal_2D(1)],...
%     [sample2D_homo(2,5),shifted_right_sym_normal_2D(2)],...
%     'Color','green','LineWidth',1,'LineStyle','--')
% line([left_extremal_3D_to_2D(1), shifted_sym_normal_2D(1)],[left_extremal_3D_to_2D(2)...
%     ,shifted_sym_normal_2D(2)],'Color','red','LineWidth',1,'LineStyle','--')
% %%Plot the intersection of all vanishing lines
% axis equal
% axis on
% xlabel('X')
% ylabel('Y')


% ========== DEBUG THE EXTREMAL PART ========== 
disp('HAS TO BE THE SAME, ENFORCING THE EQUATION 18');
dot(sym_normal_unit,right_extremal_3D+left_extremal_3D)
-2*sym_plane_2ndway_normalized(4)

disp('HAS TO BE ZEROS, ENFORCING THE EQUATION 19');
vector_lr = right_extremal_3D-left_extremal_3D;
w_unit
dot(sym_normal_unit,vector_lr)
norm(vector_lr)

rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)/norm(sym_normal_unit)))
dot([left_extremal_3D; 1], left_plane_normalized)
dot([right_extremal_3D; 1], right_plane_normalized)





%% ========== SECTION 3 EXPERIMENTING WITH ONE POINT FIRST ========== 
%Get the two non extremal points
%Gotta fix this it gotta be augmented form 
non_ex1_l = sample2D(2,:)'
non_ex1_r = sample2D(1,:)'

%Get the centroid of it
non_ex1_s = mean([non_ex1_l , non_ex1_r], 2)
%Visualize the centroid of it
%Find rectified symmetry points
non_ex1_vline = vp_car(1:2) - non_ex1_s %Symmetry line

non_ex1_left_proj = non_ex1_l - non_ex1_s
non_ex1_right_proj = non_ex1_r - non_ex1_s

non_ex1_l_rec = dot(non_ex1_left_proj,non_ex1_vline)...
    /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s
non_ex1_r_rec = dot(non_ex1_right_proj,non_ex1_vline)...
    /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s

non_ex1_l_rec_aug = [non_ex1_l_rec; 1] %Augment it
non_ex1_r_rec_aug = [non_ex1_r_rec; 1]

disp('HAS TO BE the O, enforcing on the symline');
vector_lr = non_ex1_r_rec_aug-non_ex1_l_rec_aug;
vector_lr = vector_lr(1:2);
sym_line = vp_car-non_ex1_l_rec_aug;
sym_line = sym_line(1:2);
rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_line/norm(sym_line))))

%Visualize the symmetry point
figure
%scatter(vphorizon(1),vphorizon(2),400,'blue','filled');
hold on
imshow(undistim)
%Yaxis vanishing
hold on
scatter(sample2D_homo(1,:),sample2D_homo(2,:),40,'r','filled'); %Plot the sample 2D 
scatter(left_tground(1,:),left_tground(2,:),100,'blue','filled'); %Plot the sample 2D 
scatter(right_tground(1,:),right_tground(2,:),100,'blue','filled'); %Plot the sample 2D 
%%Plot the intersection of all vanishing lines
scatter(vp_car(1,:),vp_car(2,:),400,'yellow','filled'); %Plot the sample 2D 
scatter(non_ex1_l(1,:),non_ex1_l(2,:),40,'yellow','filled'); %Plot the sample 2D 
scatter(non_ex1_r(1,:),non_ex1_r(2,:),40,'yellow','filled'); %Plot the sample 2D 
line([non_ex1_s(1), vp_car(1)],[non_ex1_s(2),vp_car(2)],'Color','green','LineWidth',2,'LineStyle','--')
line([non_ex1_s(1), non_ex1_l(1)],[non_ex1_s(2),non_ex1_l(2)],'Color','yellow','LineWidth',2,'LineStyle','--')
line([non_ex1_s(1), non_ex1_r(1)],[non_ex1_s(2),non_ex1_r(2)],'Color','blue','LineWidth',2,'LineStyle','--')
scatter(non_ex1_l_rec(1,:),non_ex1_l_rec(2,:),30,'black','filled'); %Plot the sample 2D 
scatter(non_ex1_r_rec(1,:),non_ex1_r_rec(2,:),30,'black','filled'); %Plot the sample 2D 
% lines
axis equal
axis on
xlabel('X')
ylabel('Y')

%% Assuming trackwidth 
%Make the iterative convergence loop
ds = sym_plane_2ndway_normalized(4)

syms lambda_l lambda_r
eqn1 = lambda_r*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_r_rec_aug)...
    == -ds + 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec_aug - lambda_l*inv(k_matrix*R)*non_ex1_l_rec_aug);
eqn2 = lambda_l*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_l_rec_aug)...
    == -ds - 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec_aug - lambda_l*inv(k_matrix*R)*non_ex1_l_rec_aug);
sol = vpasolve([eqn1, eqn2], [lambda_l, lambda_r]);
lambda_l = sol.lambda_l
lambda_r = sol.lambda_r

non_ex1_l_3D = lambda_l*inv(k_matrix*R)*non_ex1_l_rec_aug
non_ex1_r_3D = lambda_r*inv(k_matrix*R)*non_ex1_r_rec_aug
disp('HAS TO BE CLOSE TO Zero, ENFORCING THE EQUATION 18');
dot(sym_normal_unit,non_ex1_r_3D+non_ex1_l_3D)+2*ds
disp('HAS TO BE the SAME, ENFORCING THE EQUATION 19');
vector_lr = non_ex1_r_3D-non_ex1_l_3D;
dot(sym_normal_unit,vector_lr)
norm(vector_lr)
rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)/norm(sym_normal_unit)))

%Check 3D points
non_ex1_l_3D_to_2D = k_matrix*R*non_ex1_l_3D;
non_ex1_r_3D_to_2D = k_matrix*R*non_ex1_r_3D;
non_ex1_l_3D_to_2D = non_ex1_l_3D_to_2D / non_ex1_l_3D_to_2D(3);
non_ex1_r_3D_to_2D = non_ex1_r_3D_to_2D / non_ex1_r_3D_to_2D(3);

disp('HAS TO BE the O, enforcing on the symline');
vector_lr = non_ex1_r_3D_to_2D-non_ex1_l_3D_to_2D;
vector_lr = vector_lr(1:2);
sym_line = vp_car-non_ex1_l_3D_to_2D;
sym_line = sym_line(1:2);
rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_line/norm(sym_line))))

%Visualize the 3D location onto 2D image
figure
%scatter(vphorizon(1),vphorizon(2),400,'blue','filled');
hold on
imshow(undistim)
%Yaxis vanishing
hold on
% scatter(sample2D_homo(1,:),sample2D_homo(2,:),60,'r','filled'); 
scatter(non_ex1_l(1,:),non_ex1_l(2,:),60,'yellow','filled'); 
scatter(non_ex1_r(1,:),non_ex1_r(2,:),60,'yellow','filled'); 
scatter(non_ex1_l_rec(1,:),non_ex1_l_rec(2,:),80,'black','filled'); 
scatter(non_ex1_r_rec(1,:),non_ex1_r_rec(2,:),80,'black','filled');
line([non_ex1_l_rec(1,:), non_ex1_r_rec(1,:)],[non_ex1_l_rec(2,:),...
 non_ex1_r_rec(2,:)],'Color','blue','LineWidth',2,'LineStyle','-')
scatter(non_ex1_l_3D_to_2D(1,:),non_ex1_l_3D_to_2D(2,:),40,'green','filled'); 
scatter(non_ex1_r_3D_to_2D(1,:),non_ex1_r_3D_to_2D(2,:),40,'green','filled'); 
% lines
axis equal
axis on
xlabel('X')
ylabel('Y')

%% ==========  PART 4 - POINTS ON THE SYMMETRY PLANE ========== 
%Get a particular symmetry point 2D
%sym_point_pair1 = mean([sample2D(1,:)', sample2D(2,:)'], 2)
sym_point_pair1 = [1099;354 ;1]
%Solving for lambda_s 
lambda_s = -ds/(dot(sym_normal_unit,inv(k_matrix*R)*sym_point_pair1))
sym_point_pair1_3D = lambda_s*inv(k_matrix*R)*sym_point_pair1
%Verify that the lambda wworks
sym_point_pair1_3D_to_2D = k_matrix*R*sym_point_pair1_3D
sym_point_pair1_3D_to_2D = sym_point_pair1_3D_to_2D/sym_point_pair1_3D_to_2D(3)

% figure
% hold on
% imshow(undistim)
% %Yaxis vanishing
% hold on
% scatter(sample2D_homo(1,9:10),sample2D_homo(2,9:10),40,'r','filled'); %Plot the sample 2D 
% scatter(sym_point_pair1(1,:),sym_point_pair1(2,:),40,'blue','filled');
% axis equal
% axis on
% xlabel('X')
% ylabel('Y')


%% ========== PART 3 - FOR ALL THE POINTS ========== 
non_extremal_indice = [2,1;
                       4,3;
                       6,5;
                       10,9];

all_3d_points = zeros(size(sample2D_homo,1),size(sample2D_homo,2))
all_3d_points(:,8) = left_extremal_3D
all_3d_points(:,7) = right_extremal_3D

all_non_extremal_points = []
non_extremal_separations = zeros(1, size(non_extremal_indice,1))
%PROCESSING FOR NONEXTREMAL INDICES
for k = 1: size(non_extremal_indice,1)
    non_ex1_l = sample2D(non_extremal_indice(k,1),:)';
    non_ex1_r = sample2D(non_extremal_indice(k,2),:)';
    %Get the centroid of it
    non_ex1_s = mean([non_ex1_l , non_ex1_r], 2);
    %Find rectified symmetry points
    non_ex1_vline = vp_car(1:2) - non_ex1_s; %Symmetry line
    non_ex1_left_proj = non_ex1_l - non_ex1_s;
    non_ex1_right_proj = non_ex1_r - non_ex1_s;
    non_ex1_l_rec = dot(non_ex1_left_proj,non_ex1_vline)...
        /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s;
    non_ex1_r_rec = dot(non_ex1_right_proj,non_ex1_vline)...
        /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s;
    non_ex1_l_rec = [non_ex1_l_rec; 1]; %Augment it
    non_ex1_r_rec = [non_ex1_r_rec; 1];

    syms lambda_l lambda_r;
    eqn1 = lambda_r*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_r_rec)...
        == -ds + 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec - lambda_l*inv(k_matrix*R)*non_ex1_l_rec);
    eqn2 = lambda_l*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_l_rec)...
        == -ds - 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec - lambda_l*inv(k_matrix*R)*non_ex1_l_rec);
    sol = vpasolve([eqn1, eqn2], [lambda_l, lambda_r]);
    lambda_l = sol.lambda_l
    lambda_r = sol.lambda_r

    non_ex1_l_3D = lambda_l*inv(k_matrix*R)*non_ex1_l_rec
    non_ex1_r_3D = lambda_r*inv(k_matrix*R)*non_ex1_r_rec
    non_extremal_separations(k) = norm(non_ex1_r_3D-non_ex1_l_3D)
    all_3d_points(:,non_extremal_indice(k,1)) = non_ex1_l_3D; %Save the generated points
    all_3d_points(:,non_extremal_indice(k,2)) = non_ex1_r_3D;
    all_non_extremal_points = [all_non_extremal_points non_ex1_l_3D]
    all_non_extremal_points = [all_non_extremal_points non_ex1_r_3D]
end

all_3d_points = [all_3d_points left_tground_3D]
all_3d_points_to_2D = k_matrix*R*all_3d_points
all_3d_points_to_2D = all_3d_points_to_2D./all_3d_points_to_2D(3,:) %Normalize by third dimension 

sample2D_homo_and_ground = [sample2D_homo left_tground]

reproj_error = ((sum((all_3d_points_to_2D-sample2D_homo_and_ground).^2,'all'))...
    /size(sample2D_homo_and_ground,2))...
    .^(0.5)

figure
hold on
imshow(undistim)
%Yaxis vanishing
hold on
scatter(sample2D_homo_and_ground(1,:),sample2D_homo_and_ground(2,:),40,'r','filled'); %Plot the sample 2D 
scatter(all_3d_points_to_2D(1,:),all_3d_points_to_2D(2,:),20,'blue','filled');
% axis equal
% axis on
% xlabel('X')
% ylabel('Y')



%=================================================
%Experimenting with the lagrangian matrix
% D = inv(k_matrix*R)*sample2D_homo_and_ground(:,11);
% D = D/norm(D)
% ng = [0,1,0]';
% dg = -poleheight;
% nl = sym_normal_unit;
% dl = sym_plane_2ndway_normalized(4) + w_unit/2;
% 
% M = [-2*D(1).^2*(1-D(1).^2) 2*D(1).^2*D(2)*D(1)      2*D(1).^2*D(3)*D(1) ng(1) nl(1);
%     2*D(2).^2*D(1)*D(2)   -2*D(2).^2*(1-D(2).^2)    2*D(2).^2*D(3)*D(2) ng(2) nl(2);
%     2*D(3).^2*D(1)*D(3)   2*D(3).^2*D(2)*D(3)   -2*D(3).^2*(1-D(3).^2) ng(3) nl(3);
%     ng(1) ng(2) ng(3) 0 0;
%     nl(1) nl(2) nl(3) 0 0;
%     ]
% N = [0 0 0 -dg -dl]'
% 
% lpoint = linsolve(M,N)
% lpoint = lpoint(1:3)
% dot([lpoint; 1], left_plane)
% dot([lpoint; 1], [0,1,0,-poleheight]')
% lpoint_2d = k_matrix*R*lpoint
% lpoint_2d = lpoint_2d/lpoint_2d(3)
% norm(lpoint_2d-sample2D_homo_and_ground(:,11))


%=================================================
%Experimenting with balancing extremal points for a better cost
% left_extremal = sample2D_homo_and_ground(:,6);
% left_extremal_proj_dir = inv(k_matrix*R)*left_extremal;
% left_lambda = -left_plane_normalized(4)/dot(left_extremal_proj_dir,left_plane_normalized(1:3));
% 
% right_extremal = sample2D_homo_and_ground(:,5);
% right_extremal_proj_dir = inv(k_matrix*R)*right_extremal;
% right_lambda = -right_plane_normalized(4)/dot(right_extremal_proj_dir,right_plane_normalized(1:3));
% 
% left_extremal_3D = left_extremal_proj_dir*left_lambda
% right_extremal_3D = right_extremal_proj_dir*right_lambda
% vector_lr = right_extremal_3D-left_extremal_3D
% dot(vector_lr, sym_normal_unit) %This should be zero actually
% t = (-sym_plane_2ndway_normalized(4)-dot(left_extremal_3D,sym_normal_unit))...
%     /(dot(vector_lr,sym_normal_unit))
% intersect_point = left_extremal_3D + t*vector_lr
% right_extremal_3D_corrected = intersect_point + ...
%     dot(right_extremal_3D-intersect_point,sym_normal_unit)*sym_normal_unit
% left_extremal_3D_corrected = intersect_point + ...
%     dot(left_extremal_3D-intersect_point,(-1)*sym_normal_unit)*(-1)*sym_normal_unit
% costA = norm(left_extremal_3D-left_extremal_3D_corrected) + ...
%     norm(right_extremal_3D-right_extremal_3D_corrected)
% left_extremal_3D_corrected = left_extremal_3D
% right_extremal_3D_corrected = left_extremal_3D + ...
%     dot(right_extremal_3D-left_extremal_3D,sym_normal_unit)*sym_normal_unit
% costB = norm(left_extremal_3D-left_extremal_3D_corrected) + ...
%     norm(right_extremal_3D-right_extremal_3D_corrected)

%check for d


% 
% 
% %=================================================
% %Figuring out rotation matrix
% world2cam_R = R
% car_origin = all_3d_points(:,11)
% x_end = car_origin+sym_normal_unit
% x_axis = x_end-car_origin;
% % x_axis = x_axis/norm(x_axis);
% y_end = [car_origin(1) car_origin(2)+1 car_origin(3)]';
% y_axis = y_end -car_origin;
% % y_axis = y_axis/norm(y_axis);
% z_end = car_origin+cross(x_axis,y_axis)
% z_axis = z_end - car_origin;
% % z_axis = z_axis/norm(z_axis);
% %Check if they are perpendicular
% rad2deg(acos(dot(x_axis, z_axis)))
% rad2deg(acos(dot(x_axis, y_axis)))
% car_frame = [x_axis y_axis z_axis]
% rotx_angle = acos(dot(car_frame(:,1), [1 0 0]'))
% world2car_R = [cos(rotx_angle) 0 sin(rotx_angle);
%         0 1 0;
%         -sin(rotx_angle) 0 cos(rotx_angle);];
% world2car_T = -car_origin %Move Car origin to world origin CONFIRMED
% 
% %A  different way but it works correctly different from PENN
% x_axis_car = world2car_R*(x_end+world2car_T)
% y_axis_car = world2car_R*(y_end+world2car_T)
% z_axis_car = world2car_R*(z_end+world2car_T)
% rad2deg(acos(dot(x_axis_car/norm(x_axis_car), y_axis_car/norm(y_axis_car))))
% rad2deg(acos(dot(x_axis_car/norm(x_axis_car), z_axis_car/norm(z_axis_car))))
% 
% 
% %We do not know vanishingpoint in 3D so cannot do sth like this
% % vpcar_car = world2car_R*(vp_car+world2car_T)
% 
% %Why this will result in different result gotta move the sym normal to new
% %origin first and then do transformation ???
% sym_normal_car = world2car_R*(car_origin+sym_normal+world2car_T)
% sym_normal_unit_car = sym_normal_car/norm(sym_normal_car)
% rad2deg(acos(dot(x_axis_car/norm(x_axis_car), sym_normal_unit_car)))
% 
% left_tground_3D_car = world2car_R*(left_tground_3D+world2car_T)
% 
% %After modifying the the origin
% dl_car = -dot(sym_normal_car,left_tground_3D_car)/norm(sym_normal_car)
% % dr_car = dl_car - w_unit
% ds_car = dl_car + w_unit/2
% % left_plane_car = [sym_normal_unit_car; dl_car]
% % right_plane_car = [sym_normal_unit_car; dr_car]
% 
% % disp("BOTH of these should be 0")
% % left_extremal_3D_car = world2car_R*(left_extremal_3D+world2car_T)
% % dot([left_extremal_3D_car; 1], left_plane_car)
% % right_extremal_3D_car = world2car_R*(right_extremal_3D+world2car_T)
% % dot([right_extremal_3D_car; 1], right_plane_car)

%% Before optimization
car_origin = all_3d_points(:,11);
world2car_R = get_world2car_rotation(car_origin, sym_normal_unit)
world2car_T = -car_origin
all_3d_points_car = world2car_R*(all_3d_points+world2car_T)

%% ========== PART 5 - REFINE THE POINTS ========== 
options = optimoptions('lsqnonlin','Display','iter');
options.StepTolerance = 1e-10;
options.FunctionTolerance = 1e-7;
options.Algorithm = 'levenberg-marquardt'
options

theta0 = [vp_car(1),vp_car(2),vp_car(3),w_unit,sym_plane_2ndway_normalized(4)...
    non_extremal_separations(1),...
    non_extremal_separations(2),...
    non_extremal_separations(3),...
    non_extremal_separations(4)]';
% Pass all the thing you can optimize through theta0
% Along with all the other things to calculate the reprojection error
% Return the optimal theta
[theta_final,resnorm,residual,exitflag,output]= lsqnonlin(@(p) lsqFun(p,k_matrix, R,sample2D_homo_and_ground,...
    all_3d_points, non_extremal_indice,poleheight,backpoint), theta0,[],[],options);

%% ========== EVALUATION USING THE REFINED PARAMETERS ========== 
%Get the new parameter in
theta = theta_final;
%Calculate the new estimate
all_refined_points_3D = get_new_estimate(theta,k_matrix,R,sample2D_homo_and_ground,...
    all_3d_points,non_extremal_indice, poleheight,backpoint);
%Calculate the prediction
all_refined_points = k_matrix*R*all_refined_points_3D;
all_refined_points = all_refined_points./all_refined_points(3,:);
%Calculate error
num_points = 11
disp('After LSQnonlin error')
fErr = sum(([sample2D_homo_and_ground(:,1:11)]-[all_refined_points(:,1:11)]).^2,'all')...
+0*sum(([sample2D_homo_and_ground(:,5:6)]-[all_refined_points(:,5:6)]).^2,'all');
fErr = (fErr/num_points).^(0.5)

disp('Initial guess error')
initial_error = sum(([sample2D_homo_and_ground(:,1:11)]-[all_3d_points_to_2D(:,1:11)]).^2,'all')...
+0*sum(([sample2D_homo_and_ground(:,5:6)]-[all_3d_points_to_2D(:,5:6)]).^2,'all');
initial_error = (initial_error/num_points).^(0.5)

figure
hold on
imshow(undistim)
%Yaxis vanishing
hold on
scatter(sample2D_homo_and_ground(1,:),sample2D_homo_and_ground(2,:),40,'r','filled'); %Plot the sample 2D 
scatter(all_refined_points(1,:),all_refined_points(2,:),30,'green','filled');
scatter(all_3d_points_to_2D(1,:),all_3d_points_to_2D(2,:),20,'blue','filled');
% axis equal
% axis on
% xlabel('X')
% ylabel('Y')


%% ========== OFFSETS AFTER OPTIMIZATIONS ========== 
vp_car_final = [theta_final(1),theta_final(2),1]';
vp_car_final(1) = vp_car_final(1)
sym_normal_final = inv(k_matrix*R)*vp_car_final;
sym_normal_final_unit = sym_normal_final/norm(sym_normal_final);
car_origin_final = all_refined_points_3D(:,11)
world2car_R_final = get_world2car_rotation(car_origin_final, sym_normal_final_unit)
world2car_T_final = -car_origin_final
all_refined_points_car = world2car_R_final *(all_refined_points_3D +world2car_T_final )



%% ========== EVALUATION ON THE CAR TO LOOK AT THE ORIENTATION OF SYM PLANE ========== 
vp_car_final = [theta_final(1),theta_final(2),1]';
vp_car_final(1) = vp_car_final(1)
sym_normal_final = inv(k_matrix*R)*vp_car_final;
sym_normal_final_unit = sym_normal_final/norm(sym_normal_final);
car_origin_final = all_refined_points_3D(:,11)
% car_origin_final = all_3d_points(:,11)  %Try using the original oringin

ds_final = theta_final(5)
dl_final = ds_final + w_unit/2;
left_plane_final = [sym_normal_final_unit; dl_final];
ground_plane_final = [0 1 0 -poleheight]';

%Debug
x_end = car_origin_final+sym_normal_final_unit;
x_axis = x_end-car_origin_final;
%2nd way for z
% z_end_z = car_origin_final(3) + 4
% z_end_x = (-dl_final - dot(sym_normal_final_unit(2:3),[poleheight z_end_z]))/sym_normal_final_unit(1)
% z_end = [z_end_x poleheight z_end_z]'
% y_end = car_origin_final+cross(z_axis,x_axis);
y_end = [car_origin_final(1) car_origin_final(2)+1 car_origin_final(3)]';
y_axis = y_end - car_origin_final;
z_end = car_origin_final+cross(x_axis,y_axis);
z_axis = z_end - car_origin_final;

% This angle is close to 0 or 180
left_extremal_axis = [all_refined_points_3D(1,8) all_refined_points_3D(2,11) all_refined_points_3D(3,8)]'...
    -all_refined_points_3D(:,11);
rad2deg(acos(dot(left_extremal_axis/norm(left_extremal_axis), z_axis/norm(z_axis))))
rad2deg(acos(dot(left_extremal_axis/norm(left_extremal_axis), sym_normal_final_unit/norm(sym_normal_final_unit))))
rad2deg(acos(dot(x_axis/norm(x_axis), z_axis/norm(z_axis))))
rad2deg(acos(dot(y_axis/norm(y_axis), z_axis/norm(z_axis))))
rad2deg(acos(dot(y_axis/norm(y_axis), x_axis/norm(x_axis))))
%Check if all these points are on the corresponding planes
dot([z_end; 1],left_plane_final)
dot([z_end; 1],ground_plane_final)
dot([all_refined_points_3D(:,11); 1],left_plane_final)
dot([all_refined_points_3D(:,8); 1],left_plane_final)

% Calculate to visualize on 2D image
z_end_2D = k_matrix*R*z_end
z_end_2D = z_end_2D/z_end_2D(3)
x_end_2D = k_matrix*R*x_end
x_end_2D = x_end_2D/x_end_2D(3)
y_end_2D = k_matrix*R*y_end
y_end_2D = y_end_2D/y_end_2D(3)
car_origin_final_2D = k_matrix*R*car_origin_final
car_origin_final_2D = car_origin_final_2D/car_origin_final_2D(3)


figure
hold on
imshow(undistim)
%Yaxis vanishing
hold on
for i = 1:2:size(all_refined_points,2)-1
    line([all_refined_points(1,i), all_refined_points(1,i+1)],[all_refined_points(2,i),...
     all_refined_points(2,i+1)],'Color','yellow','LineWidth',2,'LineStyle','-')
end
for i = 1:size(all_refined_points,2)-3
    line([all_refined_points(1,i), all_refined_points(1,i+2)],[all_refined_points(2,i),...
     all_refined_points(2,i+2)],'Color','blue','LineWidth',2,'LineStyle','-')
end
line([all_refined_points(1,10), all_refined_points(1,11)],[all_refined_points(2,10),...
 all_refined_points(2,11)],'Color','blue','LineWidth',2,'LineStyle','-')
scatter(all_refined_points(1,:),all_refined_points(2,:),40,'green','filled');
scatter(z_end_2D(1,:),z_end_2D(2,:),60,'red','filled');
scatter(car_origin_final_2D(1),car_origin_final_2D(2),20,'red','filled');
line([z_end_2D(1), car_origin_final_2D(1)],[z_end_2D(2),...
 car_origin_final_2D(2)],'Color','blue','LineWidth',4,'LineStyle','--')
line([y_end_2D(1), car_origin_final_2D(1)],[y_end_2D(2),...
 car_origin_final_2D(2)],'Color','green','LineWidth',4,'LineStyle','--')
line([x_end_2D(1), car_origin_final_2D(1)],[x_end_2D(2),...
 car_origin_final_2D(2)],'Color','red','LineWidth',4,'LineStyle','--')
line([vp_car_final(1), car_origin_final_2D(1)],[vp_car_final(2),...
 car_origin_final_2D(2)],'Color','red','LineWidth',1,'LineStyle','-')


%% =============== DO THE ROTATION AND DISPLAY 1D ===============
world2car_R_final = get_world2car_rotation(car_origin_final, sym_normal_final_unit)
world2car_T_final = -car_origin_final
all_refined_points_car = world2car_R_final *(all_refined_points_3D +world2car_T_final )
back_left_point_car = [all_refined_points_car(1,11) all_refined_points_car(2,11) all_refined_points_car(3,2)]'
all_left_points_car = [all_refined_points_car(:,2),all_refined_points_car(:,4),...
    all_refined_points_car(:,6),all_refined_points_car(:,8),all_refined_points_car(:,10),...
    all_refined_points_car(:,11) back_left_point_car]
%Scatter plot based on yz
figure
hold on
set(gca, 'YDir','reverse')
set(gca, 'XDir','reverse')
xlim([-0.8 3])
scatter(all_left_points_car(3,6),all_left_points_car(2,6),280,'red','filled');
scatter(all_left_points_car(3,1:6),all_left_points_car(2,1:6),120,'green','filled');
for i = 1:size(all_left_points_car,2)-1
    line([all_left_points_car(3,i), all_left_points_car(3,i+1)],[all_left_points_car(2,i),...
     all_left_points_car(2,i+1)],'Color','blue','LineWidth',3,'LineStyle','--')
end 
axis equal
axis on
xlabel('Z')
ylabel('Y')

%% =============== DISPLAY ON TOP OF THE CAR ===============
back_left_point = inv(world2car_R_final)*back_left_point_car-world2car_T_final
back_left_point_2D = k_matrix*R*back_left_point
back_left_point_2D = back_left_point_2D/back_left_point_2D(3)

figure
hold on
imshow(undistim)
%Yaxis vanishing
hold on
for i = 1:2:size(all_refined_points,2)-1
    line([all_refined_points(1,i), all_refined_points(1,i+1)],[all_refined_points(2,i),...
     all_refined_points(2,i+1)],'Color','yellow','LineWidth',2,'LineStyle','-')
end
for i = 1:size(all_refined_points,2)-3
    line([all_refined_points(1,i), all_refined_points(1,i+2)],[all_refined_points(2,i),...
     all_refined_points(2,i+2)],'Color','blue','LineWidth',2,'LineStyle','-')
end
line([all_refined_points(1,10), all_refined_points(1,11)],[all_refined_points(2,10),...
 all_refined_points(2,11)],'Color','blue','LineWidth',2,'LineStyle','-')
scatter(all_refined_points(1,:),all_refined_points(2,:),40,'green','filled');
% scatter(z_end_2D(1,:),z_end_2D(2,:),60,'red','filled');
% line([all_refined_points(1,11), back_left_point_2D(1)],[all_refined_points(2,11),...
%  back_left_point_2D(2)],'Color','blue','LineWidth',2,'LineStyle','-')
% line([vp_car_final(1), all_refined_points(1,11)],[vp_car_final(2),...
%  all_refined_points(2,11)],'Color','red','LineWidth',2,'LineStyle','-')
% line([z_end_2D(1), all_refined_points(1,11)],[z_end_2D(2),...
%  all_refined_points(2,11)],'Color','blue','LineWidth',2,'LineStyle','--')
% line([y_end_2D(1), all_refined_points(1,11)],[y_end_2D(2),...
%  all_refined_points(2,11)],'Color','green','LineWidth',2,'LineStyle','--')
% line([x_end_2D(1), all_refined_points(1,11)],[x_end_2D(2),...
%  all_refined_points(2,11)],'Color','red','LineWidth',2,'LineStyle','--')


%% Coding for the tire contact point
ng = [0,1,0]'
nl = sym_normal_unit
u = cross(nl, ng)
dg = -poleheight;
dl = ds + w_unit/2;
A = [ng(1:2)' dg;
    nl(1:2)' dl]
A_solve = rref(A)

% tire_point = all_3d_points(:,11)
% tire_point_2 = [9 poleheight 16]'
% 
% %Visualize the direction of the ground line
% u_2d = k_matrix*R*(5*u+tire_point)
% u_2d = u_2d/u_2d(3)
% u_2d_2 = k_matrix*R*(1*u+tire_point_2)
% u_2d_2 = u_2d_2/u_2d_2(3)
% 
% tire_point_2d = k_matrix*R*(tire_point)
% tire_point_2d = tire_point_2d/tire_point_2d(3)
% 
% tire_point_2d_2 = k_matrix*R*(tire_point_2)
% tire_point_2d_2 = tire_point_2d_2/tire_point_2d_2(3)
% figure
% hold on
% imshow(undistim)
% %Yaxis vanishing
% hold on
% scatter(u_2d(1,:),u_2d(2,:),40,'green','filled');
% scatter(u_2d_2(1,:),u_2d_2(2,:),40,'green','filled');
% scatter(tire_point_2d(1,:),tire_point_2d(2,:),40,'red','filled');
% scatter(tire_point_2d_2(1,:),tire_point_2d_2(2,:),40,'red','filled');
% line([u_2d(1,:), tire_point_2d(1,:)],[u_2d(2,:),...
%  tire_point_2d(2,:)],'Color','black','LineWidth',1,'LineStyle','-')
% line([u_2d_2(1,:), tire_point_2d_2(1,:)],[u_2d_2(2,:),...
%  tire_point_2d_2(2,:)],'Color','black','LineWidth',1,'LineStyle','-')

%% 2nd way
u

world_to_ground = [1 0 0; 0 0 1; 0 1/poleheight 0]
pg_world = [-A_solve(1,3) -A_solve(2,3) 0]'
pg_world_2 = pg_world+u
pg_ground = world_to_ground*pg_world
pg_2_ground = world_to_ground*pg_world_2
homo_l = cross(pg_ground,pg_2_ground)
H = k_matrix*R*inv(world_to_ground)
H_inv_transpose = inv(H)'
pg_ground_image = H*pg_ground
pg_ground_image_normalized = pg_ground_image/pg_ground_image(3)
homo_l_image = H_inv_transpose*homo_l
tg_orig = sample2D_homo_and_ground(:,11)-5

l_vector = [homo_l_image(2) -homo_l_image(1)]'
w = tg_orig(1:2) - pg_ground_image_normalized(1:2)
b = dot(w,l_vector)/dot(l_vector,l_vector)
closest_point = pg_ground_image_normalized(1:2) + b*l_vector


% %Wikiway Same result
% l_vector_2 = [-homo_l_image(2) homo_l_image(1)]';
% p_x = (homo_l_image(2) * dot(l_vector,tg_orig(1:2)) - homo_l_image(1)*...
%     homo_l_image(3))/(homo_l_image(1)^2+homo_l_image(2)^2);
% p_y = (homo_l_image(1) * dot(l_vector_2,tg_orig(1:2)) - homo_l_image(2)*...
%     homo_l_image(3))/(homo_l_image(1)^2+homo_l_image(2)^2);
% closest_point = [p_x p_y]'

% another_point = [-dot(homo_l_image(2:3),[400,1]')/homo_l_image(1) 400 1]'
% figure
% hold on
% imshow(undistim)
% %Yaxis vanishing
% hold on
% scatter(pg_ground_image_normalized(1,:),pg_ground_image_normalized(2,:),40,'red','filled');
% scatter(tg_orig(1,:),tg_orig(2,:),20,'red','filled');
% scatter(closest_point(1,:),closest_point(2,:),10,'green','filled');
% line([pg_ground_image_normalized(1,:), another_point(1,:)],[pg_ground_image_normalized(2,:),...
%  another_point(2,:)],'Color','green','LineWidth',1,'LineStyle','-')


% %% First way
% u_image = H*u_ground
% pg_image = k_matrix*R*pg_world
% pg_image_carte = pg_image/pg_image(3)
% 
% 
% w = tg_orig-pg_image_carte
% w_hat = pg_image + u_image
% w_hat = w_hat/w_hat(3) -pg_image_carte
% rad2deg(acos(dot(w/norm(w),w_hat/norm(w_hat))))
% 
% b = dot(w,w_hat)/dot(w_hat,w_hat)
% left_tire_refined_2D = pg_image_carte+b*w_hat
% left_tire_refined_2D = left_tire_refined_2D/left_tire_refined_2D(3);
% 
% left_tire_refined_2D_proj = inv(k_matrix*R)*left_tire_refined_2D;
% lambda_tg = poleheight/left_tire_refined_2D_proj(2);
% left_tire_refined_3D = lambda_tg*inv(k_matrix*R)*left_tire_refined_2D
% 
% left_tire_refined_3D_2D = k_matrix*R*left_tire_refined_3D;
% left_tire_refined_3D_2D = left_tire_refined_3D_2D/left_tire_refined_3D_2D(3)
% 
% ground_line_2D = pg_image+200*u_image;
% ground_line_2D = ground_line_2D/ground_line_2D(3);
% 
% 
% figure
% hold on
% imshow(undistim)
% %Yaxis vanishing
% hold on
% scatter(pg_image_carte(1,:),pg_image_carte(2,:),40,'black','filled');
% scatter(left_tire_refined_2D(1,:),left_tire_refined_2D(2,:),30,'green','filled');
% scatter(left_tire_refined_3D_2D(1,:),left_tire_refined_3D_2D(2,:),10,'blue','filled');
% 
% % scatter(tg_orig(1,:),tg_orig(2,:),10,'red','filled');
% % line([pg_image_carte(1,:), ground_line_2D(1,:)],[pg_image_carte(2,:),...
% %  ground_line_2D(2,:)],'Color','green','LineWidth',1,'LineStyle','-')
% % line([pg_image_carte(1,:), tg_orig(1,:)],[pg_image_carte(2,:),...
% %  tg_orig(2,:)],'Color','red','LineWidth',1,'LineStyle','-')
% 
