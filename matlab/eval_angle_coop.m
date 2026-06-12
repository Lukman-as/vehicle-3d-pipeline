function all_annos = eval_angle_coop(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    %Change this to heading angle
    vec1 = all_annos.(fieldname).pred_bbox(:,2) - all_annos.(fieldname).pred_bbox(:,3);
    vec1 = vec1/norm(vec1);
    x_axis = [1,0,0]';
    x = dot([x_axis(1);x_axis(3)],[vec1(1);vec1(3)]);
    y = det([[x_axis(1);x_axis(3)],[vec1(1);vec1(3)]]);
    all_annos.(fieldname).pred_heading_angle = rad2deg(atan2(y,x)); %With the x-axis
%     if all_annos.(fieldname).pred_heading_angle < 0
%         all_annos.(fieldname).pred_heading_angle = all_annos.(fieldname).pred_heading_angle + 360;
%     end

    vec2 = all_annos.(fieldname).corners_bbox_world(:,2) - all_annos.(fieldname).corners_bbox_world(:,3);
    vec2 = vec2/norm(vec2);
    x = dot([x_axis(1);x_axis(3)],[vec2(1);vec2(3)]);
    y = det([[x_axis(1);x_axis(3)],[vec2(1);vec2(3)]]);
    all_annos.(fieldname).gt_heading_angle = rad2deg(atan2(y,x)); %With the x-axis
%     if all_annos.(fieldname).gt_heading_angle < 0
%         all_annos.(fieldname).gt_heading_angle = all_annos.(fieldname).gt_heading_angle + 360;
%     end

    x = dot([vec1(1);vec1(3)],[vec2(1);vec2(3)]);
    y = det([[vec1(1);vec1(3)],[vec2(1);vec2(3)]]);
    angle_difference = abs(rad2deg(atan2(y,x)));
%     angle_difference = abs(all_annos.(fieldname).pred_heading_angle-all_annos.(fieldname).gt_heading_angle);
    all_annos.(fieldname).angle_difference = angle_difference;
%     assert(angle_difference < 90, 'If there is too much of a difference in angle, it is a bug')
%     origin = all_annos.(fieldname).corners_bbox_world(:,3);
%     end_point_pred = origin+8*vec1;
%     end_point_gt = origin+8*vec2;
%     all_annos.(fieldname).end_point_pred = end_point_pred;
%     all_annos.(fieldname).end_point_gt = end_point_gt;
%     all_annos.(fieldname).origin_angle = origin;
end