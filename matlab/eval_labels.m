function all_annos = eval_labels(annotated_car_id,all_annos,M_world_velo)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    [loc, dim] = get_predicted_labels_rope(all_annos.(fieldname).predicted_corners, M_world_velo);
    all_annos.(fieldname).pred_loc = loc;
    all_annos.(fieldname).pred_dim = dim;
    [loc, dim] = get_predicted_labels_rope(all_annos.(fieldname).corners_bbox_world, M_world_velo);
    all_annos.(fieldname).gt_loc = loc;
    all_annos.(fieldname).gt_dim = dim;
end