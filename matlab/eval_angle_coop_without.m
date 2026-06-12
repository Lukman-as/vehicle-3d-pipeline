function all_annos = eval_angle_coop_without(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    vec2 = all_annos.(fieldname).corners_bbox_world(:,2) - all_annos.(fieldname).corners_bbox_world(:,3);
    vec2 = vec2/norm(vec2);
    all_annos.(fieldname).gt_yaw_angle  = acos(dot(vec2,[1,0,0]'))*...
        dot(cross(vec2,[1,0,0]'),[0,1,0]);
    angle_difference = abs(all_annos.(fieldname).heading_angle-all_annos.(fieldname).gt_yaw_angle);
    all_annos.(fieldname).angle_difference = angle_difference;
    origin = all_annos.(fieldname).corners_bbox_world(:,3);
    vec1 = all_annos.(fieldname).heading_dir;
    end_point_pred = origin+8*vec1;
    end_point_gt = origin+8*vec2;
    all_annos.(fieldname).end_point_pred = end_point_pred;
    all_annos.(fieldname).end_point_gt = end_point_gt;
    all_annos.(fieldname).origin_angle = origin;
end