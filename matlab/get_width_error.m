function [predicted_width, bbox_width, width_difference] = get_width_error(predicted_corners,corners_bbox_world)
    predicted_width = norm(predicted_corners(:,1) - predicted_corners(:,2));
    bbox_width = norm(corners_bbox_world(:,1) - corners_bbox_world(:,2));
    width_difference = abs(predicted_width-bbox_width);
end