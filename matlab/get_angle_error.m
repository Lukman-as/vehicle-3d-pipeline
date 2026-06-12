function [gt_yaw, predicted_yaw, angle_difference] = ...
    get_angle_error(predicted_corners,yaw_angle)
    vec = predicted_corners(:,2) - predicted_corners(:,3);
    predicted_yaw = acos(dot([0 0 1]',vec/norm(vec)));
    gt_yaw = yaw_angle;
    angle_difference = abs(predicted_yaw-gt_yaw);
end