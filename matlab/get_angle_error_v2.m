function [pred_yaw_angle, angle_difference] = ...
    get_angle_error_v2(predicted_corners,gt_yaw_angle)
    vec = predicted_corners(:,2) - predicted_corners(:,3);
    pred_yaw_angle = atan2(vec(3),vec(1));
    angle_difference = abs(pred_yaw_angle-gt_yaw_angle);
end