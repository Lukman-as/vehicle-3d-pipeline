function fErr = coop_fun_lsq(k_matrix,R,all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    [vp_car, origin, extracted_3D_points_car_coor] = ...
        extract_info_from_x(all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon);
    %Seclecting the correct transformation
    cMw = get_cMw_using_car_points(k_matrix,R,vp_car, origin,all_annos,annotated_car_id, extracted_3D_points_car_coor);
    estimated_3d_points = inv(cMw)*extracted_3D_points_car_coor;
    estimated_3d_points = estimated_3d_points(1:3,:);
    all_refined_points = k_matrix*R*estimated_3d_points;
    all_refined_points = all_refined_points./all_refined_points(3,:);
    fErr = all_refined_points-all_annos.(fieldname).all_annotated_points;
end