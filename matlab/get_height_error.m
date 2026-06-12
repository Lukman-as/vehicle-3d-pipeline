function [predicted_height, bbox_height, height_difference] = get_height_error(predicted_corners,corners_bbox_world)
    predicted_height = norm(predicted_corners(:,1) - predicted_corners(:,5));
    bbox_height = norm(corners_bbox_world(:,1) - corners_bbox_world(:,5));
    height_difference = abs(predicted_height-bbox_height);
end