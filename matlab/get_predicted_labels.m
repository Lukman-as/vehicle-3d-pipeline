function [location, dimension, yaw_angle] = get_predicted_labels(predicted_corners, M_world_velo)
    predicted_point = mean(predicted_corners(:,1:4),2); %Need to adjust for each dataset, base or center
    predicted_point = [predicted_point; 1];
    location = inv(M_world_velo)*predicted_point;
    
    predicted_width = norm(predicted_corners(:,1) - predicted_corners(:,2));
    predicted_length = norm(predicted_corners(:,2) - predicted_corners(:,3));
    predicted_height = norm(predicted_corners(:,1) - predicted_corners(:,5));
    dimension = [predicted_width predicted_length predicted_height];

    vec = predicted_corners(:,2) - predicted_corners(:,3);
    yaw_angle = acos(dot([0 0 1]',vec/norm(vec)));
end

