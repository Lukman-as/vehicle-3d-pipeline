function all_annos = eval_width(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    
    %Eval width of complete box
    [pred_width, gt_width, width_diff] = get_width_error(all_annos.(fieldname).pred_bbox,all_annos.(fieldname).corners_bbox_world);
    all_annos.(fieldname).pred_width_complete_box = pred_width;
    all_annos.(fieldname).gt_width = gt_width;
    all_annos.(fieldname).width_diff_complete_box = width_diff;

    %Eval width of points only
    [pred_width, ~ , width_diff] = get_width_error(all_annos.(fieldname).pred_bbox_points_only,all_annos.(fieldname).corners_bbox_world);
    all_annos.(fieldname).pred_width_points_only = pred_width;
    all_annos.(fieldname).width_diff_points_only = width_diff;

    %Add a new width to output
    all_annos.(fieldname).pred_width_wo_mirrors = all_annos.(fieldname).w_unit;
end