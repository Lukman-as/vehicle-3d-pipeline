function all_annos = eval_length(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));

    %Eval length of complete box
    [pred_length, gt_length, length_diff] = ...
    get_length_error(all_annos.(fieldname).pred_bbox,all_annos.(fieldname).corners_bbox_world);
    all_annos.(fieldname).pred_length_complete_box = pred_length;
    all_annos.(fieldname).gt_length = gt_length;
    all_annos.(fieldname).length_diff_complete_box  = length_diff;

    %Eval length of points only
    [pred_length, ~ , length_diff] = ...
    get_length_error(all_annos.(fieldname).pred_bbox_points_only,all_annos.(fieldname).corners_bbox_world);
    all_annos.(fieldname).pred_length_points_only = pred_length;
    all_annos.(fieldname).length_diff_points_only = length_diff;
end