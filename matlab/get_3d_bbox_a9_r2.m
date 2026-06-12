function [yaw_angle, corners_velo_aug] = get_3d_bbox_a9_r2(raw_labels, obj_name, dilating_factors)
    object = raw_labels.(obj_name).object_data.cuboid.val;

    %Gotta check the dimension
    l = object(8)*dilating_factors(1);
    w = object(9)*dilating_factors(2);
    h = object(10)*dilating_factors(3);

    x = object(1);
    y = object(2);
    z = object(3);

    quat = [object(7) object(4) object(5) object(6)]; %Matlab use scalar first format
    euler_angles = quat2eul(quat,'XYZ');
    yaw_angle = euler_angles(3); %Z should be yaw angle
    
    %This is the heading angle with the axis of LiDAR virtual coordinate
    rz = yaw_angle;
%     R_bounding = [cos(rz) -sin(rz) 0;
%              sin(rz) cos(rz) 0;
%              0 0 1];

    R_bounding = quat2rotm(quat);

    corners = [
            -l/2,-l/2,l/2,l/2,-l/2,-l/2,l/2,l/2;
            w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2;
            -h/2,-h/2,-h/2,-h/2,h/2,h/2,h/2,h/2;
        ];

    corners_velo = R_bounding*corners;
    object_center_velo = [x, y, z]';

    %Translating
    corners_velo = corners_velo + object_center_velo;
    corners_velo_aug = [corners_velo ; ones(1,size(corners_velo,2))];
%     bbox_2D = [[object(1) object(2)];[object(1) object(4)];[object(3) object(2)];[object(3) object(4)]]';
%     bbox_2D = [bbox_2D; ones(1,size(bbox_2D,2))];
end