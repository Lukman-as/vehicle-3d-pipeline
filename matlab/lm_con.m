function [c,ceq] = lm_con(R_horizon,horizon_line,all_annos,annotated_car_id,x0_order,id_to_tire, x0,car_model_param)
    %Enforcing soft constraints
    [vp_car, origin, extracted_3D_points_car_coor] = extract_info_from_x(...
        all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon);

    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    [extreme_front_point, extreme_rear_point, extreme_height_point] = get_soft_constraints_gaussian(x0_order, ...
        x0, extracted_3D_points_car_coor,all_annos, fieldname,car_model_param,id_to_tire);
    front_ineq = extracted_3D_points_car_coor(1,:) - max([extreme_front_point(1), extreme_rear_point(1)]);
    rear_ineq = min([extreme_front_point(1), extreme_rear_point(1)]) - extracted_3D_points_car_coor(1,:);

    max_height_ineq = extreme_height_point(2) - extracted_3D_points_car_coor(2,:);
    min_height_ineq = extracted_3D_points_car_coor(2,:);

    ceq = [];
    c = [front_ineq rear_ineq max_height_ineq min_height_ineq];
end