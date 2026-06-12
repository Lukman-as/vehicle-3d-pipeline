function [all_3d_points_tire_after,ds_after] = get_3D_after(k_matrix,R, num_sympairs, num_tire_points, near_side, poleheight, x0)
    [cMw, sym_normal_unit, all_3d_points_tire_tire_coor]  = get_3Dpoints_car_from_x(k_matrix,R,num_sympairs,near_side,poleheight,x0);
    all_3d_points_tire_tire_coor = [all_3d_points_tire_tire_coor; ones(1,size(all_3d_points_tire_tire_coor,2))];
    estimated_3d_points = inv(cMw)*all_3d_points_tire_tire_coor;
    all_3d_points_tire_after = estimated_3d_points(1:3,:);
    ds_after = -dot(mean(all_3d_points_tire_after(:,1:2),2),sym_normal_unit);
end