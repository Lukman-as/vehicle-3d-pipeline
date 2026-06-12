function [loc, dim] = get_predicted_labels_rope(predicted_corners, M_world_velo)
    pred_w = norm(predicted_corners(:,1) - predicted_corners(:,2));
    pred_l = norm(predicted_corners(:,2) - predicted_corners(:,3));
    pred_h = norm(predicted_corners(:,1) - predicted_corners(:,5));
    pred_point = mean(predicted_corners(:,1:4),2); %Need to adjust for each dataset, base or center
    pred_point = pred_point + [0 pred_h/2 0]';
    pred_point = [pred_point; 1];
    loc = inv(M_world_velo)*pred_point;
    assert(loc(4)==1);
    loc = loc(1:3);
    dim = [pred_w pred_l pred_h];
end

