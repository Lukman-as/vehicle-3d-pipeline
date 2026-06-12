function height_overlap = get_height_overlap(predicted_corners, corners_bbox_world)
    a = min(predicted_corners(2,:));
    b = max(predicted_corners(2,:));
    c = min(corners_bbox_world(2,:));
    d = max(corners_bbox_world(2,:));
    if b >= c & d >= a %Only overlab in between
        height_overlap = abs(max(a, c)-min(b, d));
    else
        height_overlap = 0;
    end
end