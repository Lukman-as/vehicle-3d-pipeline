function all_annos = eval_iou(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    %This is different from python pipeline, this calculate again, while
    %python pipline used the reused
    predicted_ground_corners = [all_annos.(fieldname).pred_bbox(1,1:4);...
        all_annos.(fieldname).pred_bbox(3,1:4)];
    truth_ground_corners = [all_annos.(fieldname).corners_bbox_world(1,1:4);...
        all_annos.(fieldname).corners_bbox_world(3,1:4)];
    [area_truth, area_predicted, area_intersection] = ...
        get_area_intersection(predicted_ground_corners,truth_ground_corners);
    height_overlap = get_height_overlap(all_annos.(fieldname).pred_bbox,...
        all_annos.(fieldname).corners_bbox_world);
    iou_bev = get_iou_bev(area_intersection,area_truth,area_predicted);
    iou = get_iou_3D(area_truth, area_predicted, area_intersection,...
        height_overlap,all_annos.(fieldname).corners_bbox_world,all_annos.(fieldname).pred_bbox);
    
    all_annos.(fieldname).iou = iou;
    all_annos.(fieldname).iou_bev = iou_bev;
    all_annos.(fieldname).pred_bbox_bev = predicted_ground_corners;    
    [M,I] = min(vecnorm(all_annos.(fieldname).truth_ground_corners));
    all_annos.(fieldname).dist_nearest_corner_gt_bbox = M;
    all_annos.(fieldname).dist_nearest_corner_pred_bbox = vecnorm(predicted_ground_corners(:,I));
    all_annos.(fieldname).dist_nearest_corner_diff = all_annos.(fieldname).dist_nearest_corner_pred_bbox - all_annos.(fieldname).dist_nearest_corner_gt_bbox;
    all_annos.(fieldname).dist_base_pred_bbox = norm(mean(predicted_ground_corners,2));
    all_annos.(fieldname).dist_base_bbox_diff = all_annos.(fieldname).dist_base_pred_bbox - all_annos.(fieldname).dist_base_gt_bbox; %Changed to sign error
end