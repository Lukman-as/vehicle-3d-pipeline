%% Read the image
image = imread("data\image_2\000008.png");

%% READ THE CALIBERATION MATRIX
fid = fopen('data\calib\000008.txt','rt');
while ~feof(fid)
    tline = fgetl(fid);
    %disp(tline)
    data = split(tline);
    size(data);
    field = data(1,1);
    if strcmp(field,'P2:')
        disp(field)
        num = data(2:end,:);
        num = str2double(num);
        P2 = num'
    elseif strcmp(field,'R0_rect:')
        disp(field)
        num = data(2:end,:);
        num = str2double(num);
        R0_rect = num'
    elseif strcmp(field,'Tr_velo_to_cam:')
        disp(field)
        num = data(2:end,:);
        num = str2double(num);
        Tr_velo_to_cam = num'
    end
end
fclose(fid);

%======= GET THE VELO TO CAM ROTATION MATRIX ==========
Tr_velo_to_cam = reshape(Tr_velo_to_cam,[4,3])'
Tr_velo_to_cam_aug = [Tr_velo_to_cam; 0 0 0 1]

%======= READ ALL THE CARS IN ==========
all_cars = [];
%Get the 3D boxes in
fid = fopen('data\label_2\000008.txt','rt');
while ~feof(fid)
    tline = fgetl(fid);
    %disp(tline)
    data = split(tline);
    data = data';
    if strcmp(data(1,1),'Car') & str2double(data(1,2)) < 0.2 & ...
            (strcmp(data(1,3),'1') || strcmp(data(1,3),'0')) 
        num = data(1,5:end);
        num = str2double(num);
        all_cars = [all_cars ; num];
    end
end
fclose(fid);

%======= GET ALL THE INTRINSIC AND EXTRINSICS MATRIX ==========
P2_reshaped = reshape(P2,[4,3])';
R0_rect_aug = zeros([4,4]);
R0_rect_aug(1:3,1:3) = reshape(R0_rect,[3,3])';
%This transpose is to make up for the difference between python and matlab
R0_rect_aug(4,4) = 1;

%Generate the translation matrix from cam 0(reference cam to cam2)
t_z = P2_reshaped(3,4);
t_y =  (P2_reshaped(2,4)- t_z*P2_reshaped(2,3))/P2_reshaped(2,2);
t_x =  (P2_reshaped(1,4)- t_z*P2_reshaped(1,3))/P2_reshaped(1,1);
k_matrix = P2_reshaped(1:3,1:3);
t_vector = [t_x t_y t_z]';
% t_vector = [0 0 0]';
R = R0_rect_aug(1:3,1:3);

%Confirm these things have the same results
%Find location of cam 0 [1,1,1] in the cam 2 pic
% a = P2_reshaped*R0_rect_aug*[1 1 1 1]'
% a = a./a(3)
% b = k_matrix*(R*[1 1 1]'+t_vector)  %THIS IS THE WAY TO GO FROM CAM 0
% LOCATION TO CAM 2 YOU ROTATE FIRST AND THEN YOU TRANSLATE
% b = b./b(3)

edges_2D = [
     1 3;
     1 2;
     2 4;
     3 4;
];

acar = all_cars(1,:)
bbox_2D = [acar(1) acar(1) acar(3) acar(3); 
    acar(2) acar(4) acar(2) acar(4);
            ones(1,4)]


%% ======= CODE TO PICK THE SAMPLE POINTS AND SAVE ==========
figure
%hold on
imshow(image)
hold on
scatter(bbox_2D(1,:),bbox_2D(2,:),30,'green','filled'); %Plot the sample 2D
for i = 1:size(edges_2D,1)
    x_coor = [bbox_2D(1, edges_2D(i,1)) bbox_2D(1, edges_2D(i,2))]
    y_coor = [bbox_2D(2, edges_2D(i,1)) bbox_2D(2, edges_2D(i,2))]
    line(x_coor, y_coor,'Color','red','LineWidth',2)
end


[x,y] = getpts
sample2D = [x,y]


% save('picked2Dpoints.mat', 'sample2D')

%% Looping
% all_cars = all_cars(1:2,:)
annotations = zeros(2, 14*size(all_cars,1))
figure
hold on
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
%     [x,y] = getpts;
%     annotations(:,14*(k-1)+1:14*k) = [x,y]';
%     annotations
end


annotations
