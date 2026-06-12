function new_estimates = get_new_estimate(theta,k_matrix,R,sample2D_gt, estimated_3d_points,...
    non_extremal_indice,poleheight,backpoint)
    %All of the parameters or intial guess
    vp_car = [theta(1),theta(2),1]';
    sym_normal = inv(k_matrix*R)*vp_car;
    sym_normal_unit = sym_normal/norm(sym_normal);
    w_unit = theta(4);
    ds = theta(5); %d of sym plane #Worth thinking about though
    dl = ds + w_unit/2;
    dr = ds - w_unit/2;
    left_plane = [sym_normal_unit; dl];
    right_plane = [sym_normal_unit; dr];
    non_extremal_separations =[theta(6) theta(7) theta(8) theta(9)];
    %  Only change certain things, other than keep as usual
   all_refined_points = estimated_3d_points;
%     all_refined_points = zeros(size(estimated_3d_points,1), size(estimated_3d_points,2));

    %Rotation matrix to optimize
%     car_origin = estimated_3d_points(:,11);
%     world2car_R = get_world2car_rotation(car_origin, sym_normal_unit);
%     world2car_T = -car_origin;

    %Optimizing extremal points:
    pair_left = sample2D_gt(:,8);
    pair_right = sample2D_gt(:,7);
    [pair_left_3D pair_right_3D] = refine_non_extremal(ds,sym_normal_unit,...
        k_matrix,R, w_unit, pair_left, pair_right);
    all_refined_points(:,8) = pair_left_3D;
    all_refined_points(:,7) = pair_right_3D;

    for i = 1:size(non_extremal_indice,1)
        pair_left = sample2D_gt(:,non_extremal_indice(i,1));
        pair_right = sample2D_gt(:,non_extremal_indice(i,2));
        [pair_left_3D pair_right_3D] = refine_non_extremal(ds,sym_normal_unit,...
            k_matrix,R, non_extremal_separations(i), pair_left, pair_right);
        all_refined_points(:,non_extremal_indice(i,1)) = pair_left_3D;
        all_refined_points(:,non_extremal_indice(i,2)) = pair_right_3D;
    end
   
    %=================================================
    %Optmizing tireground
    tground_3D = get_refined_tg_homo(k_matrix,R,ds,sym_normal_unit, w_unit,...
        poleheight,sample2D_gt(:,11:end),backpoint);
    all_refined_points(:,11) = tground_3D(:,1);
    if backpoint
        all_refined_points(:,12) = tground_3D(:,2);
    end
    new_estimates = all_refined_points;
end