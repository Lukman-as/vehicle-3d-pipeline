function all_annos = get_bbox_world_hw7(all_annos, annotated_car_id,raw_labels, M_world_velo)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_hw7(raw_labels, annotated_car_id);
    corners_bbox_world = M_world_velo*corners_velo_aug;
    all_annos.(fieldname).corners_bbox_world = corners_bbox_world(1:3,:);
    truth_ground_corners = [all_annos.(fieldname).corners_bbox_world(1,1:4);...
        all_annos.(fieldname).corners_bbox_world(3,1:4)];
    all_annos.(fieldname).dist_base_gt_bbox = norm(mean(truth_ground_corners,2));
    all_annos.(fieldname).truth_ground_corners = truth_ground_corners;
    all_annos.(fieldname).bbox_2D = bbox_2D;
    all_annos.(fieldname).bbox_2D_height = (max(bbox_2D(2,:))-min(bbox_2D(2,:)));
end