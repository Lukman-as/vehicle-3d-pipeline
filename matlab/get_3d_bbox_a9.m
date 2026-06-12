function [yaw_angle, corners_road_aug] = get_3d_bbox_a9(raw_labels, car_index)
    dim = raw_labels.labels(car_index).box3d.dimension
    loc = raw_labels.labels(car_index).box3d.location
    ori = raw_labels.labels(car_index).box3d.orientation
    
    h = dim.height;
    w = dim.width;
    l = dim.length;
    x = loc.x;
    y = loc.y;
    z = loc.z;
    rz = ori.rotationYaw;
    yaw_angle = rz;
    R_bounding = [cos(rz) -sin(rz) 0;
             sin(rz) cos(rz) 0;
             0 0 1]
        
    corners = [
            l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2;
            -w/2,w/2,w/2,-w/2,-w/2,w/2,w/2,-w/2;
            0  ,0  ,   0,   0, h, h,  h,  h;
        ]
    corners_road = R_bounding*corners;
    corners_road(1,:) = corners_road(1,:) + x;
    corners_road(2,:) = corners_road(2,:) + y;
    corners_road(3,:) = corners_road(3,:) + z;
    corners_road_aug = [corners_road ; ones(1,size(corners_road,2))];

    %Take care of incorrect labeling of A9
%     corners_bbox_world = M_world_road*corners_road_aug;
%     corners_bbox_2D = k_matrix*R*corners_bbox_world(1:3,:);
%     corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:)
% 
%     bottom_left_front = raw_labels.labels(car_index).box3d_projected.bottom_left_front;
%     bottom_left_front_image = [bottom_left_front(1)*size(image, 2),bottom_left_front(2)*size(image, 1)]
%     threshold = 1e-12;
% 
%     if abs(bottom_left_front_image(1) - corners_bbox_2D(1,2))>=threshold || ...
%             abs(bottom_left_front_image(2) - corners_bbox_2D(2,2))>=threshold
%         rz = ori.rotationYaw+pi
%         R_bounding = [cos(rz) -sin(rz) 0;
%              sin(rz) cos(rz) 0;
%              0 0 1];
%         corners_road = R_bounding*corners;
%         corners_road(1,:) = corners_road(1,:) + x;
%         corners_road(2,:) = corners_road(2,:) + y;
%         corners_road(3,:) = corners_road(3,:) + z;
%         corners_road_aug = [corners_road ; ones(1,size(corners_road,2))];
%         yaw_angle = rz;
%         corners_bbox_world = M_world_road*corners_road_aug;
%         corners_bbox_2D = k_matrix*R*corners_bbox_world(1:3,:);
%         corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:);
%         assert(abs(bottom_left_front_image(1) - corners_bbox_2D(1,2))<threshold);
%          assert(abs(bottom_left_front_image(2) - corners_bbox_2D(2,2))<threshold);
%     end
end