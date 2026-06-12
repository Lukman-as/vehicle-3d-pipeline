function tground_3D = get_refined_tg_homo(k_matrix,R,ds,sym_normal_unit,...
    w_unit,poleheight,annotated_tgs,backpoint)
    ng = [0,1,0]';
    nl = sym_normal_unit;
    u = cross(nl, ng);
    dg = -poleheight;
    dl = ds + w_unit/2;
    A = [ng(1:2)' dg;
        nl(1:2)' dl];
    A_solve = rref(A);
    world_to_ground = [1 0 0; 0 0 1; 0 1/poleheight 0];
    H = k_matrix*R*inv(world_to_ground);
    pg_world = [-A_solve(1,3) -A_solve(2,3) 0]'; %setting z = 0
    pg_world_2 = pg_world+u;
    pg_ground = world_to_ground*pg_world;
    pg_2_ground = world_to_ground*pg_world_2;
    homo_l = cross(pg_ground,pg_2_ground);
    H_inv_transpose = inv(H)';
    pg_ground_image = H*pg_ground;
    pg_ground_image_normalized = pg_ground_image/pg_ground_image(3);
    homo_l_image = H_inv_transpose*homo_l;
    l_vector = [homo_l_image(2) -homo_l_image(1)]';

    tg_orig = annotated_tgs(:,1);
    w = tg_orig(1:2) - pg_ground_image_normalized(1:2);
    b = dot(w,l_vector)/dot(l_vector,l_vector);
    closest_point = pg_ground_image_normalized(1:2) + b*l_vector;
    left_tire_refined_2D = [closest_point; 1];
    left_tire_refined_2D_proj = inv(k_matrix*R)*left_tire_refined_2D;
    lambda_tg = poleheight/left_tire_refined_2D_proj(2);
    left_tground_3D = lambda_tg*inv(k_matrix*R)*left_tire_refined_2D;
    tground_3D = [left_tground_3D];

    if backpoint
        tg_orig = annotated_tgs(:,2);
        w = tg_orig(1:2) - pg_ground_image_normalized(1:2);
        b = dot(w,l_vector)/dot(l_vector,l_vector);
        closest_point = pg_ground_image_normalized(1:2) + b*l_vector;
        left_tire_refined_2D = [closest_point; 1];
        left_tire_refined_2D_proj = inv(k_matrix*R)*left_tire_refined_2D;
        lambda_tg = poleheight/left_tire_refined_2D_proj(2);
        left_tground_back_3D = lambda_tg*inv(k_matrix*R)*left_tire_refined_2D;
        tground_3D = [tground_3D left_tground_back_3D];
    end 
end