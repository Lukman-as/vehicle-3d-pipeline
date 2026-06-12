function all_annos = get_bbox_world_coop(all_annos, annotated_car_id,raw_labels,...
    M_world_velo,using_rectified)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, annotated_car_id);
    corners_bbox_world = M_world_velo*corners_velo_aug;
    %Fixing bounding box discrepancy due to improper calibration
    if ~using_rectified
        corners_bbox_world(2,1:4) = mean(corners_bbox_world(2,1:4));
        corners_bbox_world(2,5:8) = mean(corners_bbox_world(2,5:8));
        temp = mean([corners_bbox_world(1,1:4);corners_bbox_world(1,5:8)],1);
        corners_bbox_world(1,1:4) = temp;
        corners_bbox_world(1,5:8) = temp;
        temp = mean([corners_bbox_world(3,1:4);corners_bbox_world(3,5:8)],1);
        corners_bbox_world(3,1:4) = temp;
        corners_bbox_world(3,5:8) = temp;
    end
    all_annos.(fieldname).corners_bbox_world = corners_bbox_world(1:3,:);

    truth_ground_corners = [all_annos.(fieldname).corners_bbox_world(1,1:4);...
        all_annos.(fieldname).corners_bbox_world(3,1:4)];
    all_annos.(fieldname).dist_base_gt_bbox = norm(mean(truth_ground_corners,2));
    all_annos.(fieldname).truth_ground_corners = truth_ground_corners;
%     all_annos.(fieldname).gt_yaw_angle = yaw_angle;
    all_annos.(fieldname).bbox_2D = bbox_2D;
    all_annos.(fieldname).bbox_2D_height = (max(bbox_2D(2,:))-min(bbox_2D(2,:)));
end