function iou = get_iou_3D(area_truth, area_predicted, area_intersection,...
    height_overlap,corners_bbox_world,predicted_corners)
    volume_bbox_intersection = area_intersection*height_overlap;
    height_truth = abs(max(corners_bbox_world(2,:))- min(corners_bbox_world(2,:)));
    volume_truth =  height_truth*area_truth;
    heigh_predicted = abs(max(predicted_corners(2,:))- min(predicted_corners(2,:)));
    volume_predicted = heigh_predicted*area_predicted;
    iou = volume_bbox_intersection/(volume_truth+volume_predicted-volume_bbox_intersection);
end