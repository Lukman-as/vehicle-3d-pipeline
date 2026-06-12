function all_annos = eval_position(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    [dist_to_pred_center, dist_to_gt_center, dist_center_diff] = ...
    get_distance_error(all_annos.(fieldname).corners_bbox_world,all_annos.(fieldname).predicted_corners);
    [dist_to_pred_nearest, dist_to_gt_nearest, dist_nearest_diff] = ...
        get_distance_error_nearest(all_annos.(fieldname).corners_bbox_world,all_annos.(fieldname).predicted_corners);
    all_annos.(fieldname).dist_to_pred_center = dist_to_pred_center;
    all_annos.(fieldname).dist_to_gt_center = dist_to_gt_center;
    all_annos.(fieldname).dist_center_diff = dist_center_diff;
    all_annos.(fieldname).dist_to_pred_nearest = dist_to_pred_nearest;
    all_annos.(fieldname).dist_to_gt_nearest = dist_to_gt_nearest;
    all_annos.(fieldname).dist_nearest_diff = dist_nearest_diff;
end