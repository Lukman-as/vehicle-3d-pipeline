function all_annos = eval_height(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));

    %Eval height of complete box
    [pred_height, gt_height, height_diff] = ...
    get_height_error(all_annos.(fieldname).pred_bbox,all_annos.(fieldname).corners_bbox_world);
    all_annos.(fieldname).pred_height_complete_box = pred_height;
    all_annos.(fieldname).gt_height = gt_height;
    all_annos.(fieldname).height_diff_complete_box  = height_diff;

    %Eval height of points only
    [pred_height, ~ , height_diff] = ...
    get_height_error(all_annos.(fieldname).pred_bbox_points_only,all_annos.(fieldname).corners_bbox_world);
    all_annos.(fieldname).pred_height_points_only = pred_height;
    all_annos.(fieldname).height_diff_points_only = height_diff;
end