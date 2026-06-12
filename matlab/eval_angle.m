function all_annos = eval_angle(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    [pred_yaw_angle, angle_difference] = ...
        get_angle_error_v2(all_annos.(fieldname).predicted_corners,all_annos.(fieldname).gt_yaw_angle);
    all_annos.(fieldname).pred_yaw_angle = pred_yaw_angle;
    all_annos.(fieldname).angle_difference = angle_difference;
end