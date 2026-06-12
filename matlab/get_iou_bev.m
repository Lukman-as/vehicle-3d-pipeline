function iou_bev = get_iou_bev(area_intersection,area_truth,area_predicted)
%     area_truth = get_base_bbox(corners_bbox_world)
%     area_predicted = get_base_bbox(predicted_corners)
    iou_bev = area_intersection/(area_truth+area_predicted-area_intersection);
end