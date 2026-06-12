function [dist_to_predicted_bbox_nearest, dist_to_gt_bbox_nearest, error] = ...
    get_distance_error_nearest(corners_bbox_world,predicted_corners)
    front_point = mean(corners_bbox_world(:,1:2),2);
    dist_to_front = norm([front_point(1) front_point(3)]);
    back_point = mean(corners_bbox_world(:,3:4),2);
    dist_to_back = norm([back_point(1) back_point(3)]);
    dist_to_gt_bbox_nearest = min([dist_to_front, dist_to_back]);

    %predicted
    front_point = mean(predicted_corners(:,1:2),2);
    dist_to_front = norm([front_point(1) front_point(3)]);
    back_point = mean(predicted_corners(:,3:4),2);
    dist_to_back = norm([back_point(1) back_point(3)]);
    dist_to_predicted_bbox_nearest = min([dist_to_front, dist_to_back]);

    error = dist_to_predicted_bbox_nearest-dist_to_gt_bbox_nearest;
end