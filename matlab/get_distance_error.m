function [dist_to_predicted_bbox_center, dist_to_gt_bbox_center, error] = ...
    get_distance_error(corners_bbox_world,predicted_corners);
    truth_point = mean(corners_bbox_world(:,1:4),2);
    truth_point = [truth_point(1) truth_point(3)]; %On the ground only
    predicted_point = mean(predicted_corners(:,1:4),2); %Base only
    predicted_point = [predicted_point(1) predicted_point(3)];
    dist_to_gt_bbox_center = norm(truth_point);
    dist_to_predicted_bbox_center = norm(predicted_point);
    error = dist_to_predicted_bbox_center-dist_to_gt_bbox_center;
end