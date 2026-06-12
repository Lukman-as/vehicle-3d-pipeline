function [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_hw7(raw_labels, obj_index)
    fieldname = sprintf("obj_%s",int2str(obj_index));
    object = raw_labels.(fieldname);

    %Skip the 2D bbox first four
%     h = object(5);
%     w = object(6);
%     l = object(7);

    %In HW7 dataset
    l = object(5);
    w = object(6);
    h = object(7);

%     x = object(8);
%     y = object(9);
%     z = object(10);

    x = object(8);
    y = object(9);
    z = object(10);

%     yaw_angle_cam = object(11);
%     theta = [cos(yaw_angle_cam) 0 -sin(yaw_angle_cam)]';
%     theta0 = M_velo_cam(1:3,1:3)*theta;
%     yaw_angle = atan2(theta0(2),theta0(1));
    
    yaw_angle = object(11); %This is different from ROPE, ROAD is in VELO alread
    %This is the heading angle with the axis of LiDAR virtual coordinate
    rz = yaw_angle;
    R_bounding = [cos(rz) -sin(rz) 0;
             sin(rz) cos(rz) 0;
             0 0 1];

        
%     corners = [
%             l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2;
%             -w/2,w/2,w/2,-w/2,-w/2,w/2,w/2,-w/2;
%             0  ,0  ,   0,   0, h, h,  h,  h;
%         ];


    corners = [
            -l/2,l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2;
            w/2,w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2;
            0  ,0  ,   0,   0, h, h,  h,  h;
        ];

    corners_velo = R_bounding*corners;
    %Translate by the location
%     object_center_cam = [x, y, z 1]';
%     object_center_world = M_velo_cam * object_center_cam;
%     object_center_world = object_center_world(1:3);
    object_center_velo = [x, y, z]'; %Already in velo coordinate, different from ROPE

    %Translating
    corners_velo = corners_velo + object_center_velo;
    corners_velo_aug = [corners_velo ; ones(1,size(corners_velo,2))];
    bbox_2D = [[object(1) object(2)];[object(1) object(4)];[object(3) object(2)];[object(3) object(4)]]';
    bbox_2D = [bbox_2D; ones(1,size(bbox_2D,2))];
end