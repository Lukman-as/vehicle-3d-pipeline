function [predicted_length, bbox_length, length_difference] = get_length_error(predicted_corners,corners_bbox_world)
    predicted_length = norm(predicted_corners(:,1) - predicted_corners(:,4));
    bbox_length = norm(corners_bbox_world(:,1) - corners_bbox_world(:,4));
    length_difference = abs(predicted_length-bbox_length);
end