function fErr = myfun(k_matrix,R,sample2D_gt,num_sympairs,near_side,poleheight, x0)    
    [cMw, sym_normal_unit, all_3d_points_tire_tire_coor] = get_3Dpoints_car_from_x(k_matrix,R,num_sympairs,near_side,poleheight,x0);
    all_3d_points_tire_tire_coor = [all_3d_points_tire_tire_coor; ones(1,size(all_3d_points_tire_tire_coor,2))];

    estimated_3d_points = inv(cMw)*all_3d_points_tire_tire_coor;
    estimated_3d_points = estimated_3d_points(1:3,:);
  
    all_refined_points = k_matrix*R*estimated_3d_points;
    all_refined_points = all_refined_points./all_refined_points(3,:);

    fErr = ((sum((all_refined_points-sample2D_gt).^2,'all'))...
    /size(sample2D_gt,2))...
    .^(0.5);
end