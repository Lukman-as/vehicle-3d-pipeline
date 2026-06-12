function all_annos = get_bbox_world(all_annos, annotated_car_id,preid_to_gtid,raw_labels,M_velo_cam,M_world_velo)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    obj_index = preid_to_gtid(annotated_car_id);
    [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_rope(raw_labels, obj_index, M_velo_cam);
    corners_bbox_world = M_world_velo*corners_velo_aug;
    all_annos.(fieldname).corners_bbox_world = corners_bbox_world(1:3,:);
    all_annos.(fieldname).gt_yaw_angle = yaw_angle;
    all_annos.(fieldname).bbox_2D = bbox_2D;
    all_annos.(fieldname).mounting_height = max(corners_bbox_world(2,:));
    all_annos.(fieldname).mounting_height = 7.4;
end