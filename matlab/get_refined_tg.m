function left_tground_3D = get_refined_tg(k_matrix,R,ds,sym_normal_unit, w_unit,poleheight,annotated_tg)
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
    u_ground = world_to_ground*u;
    pg_world = [-A_solve(1,3) -A_solve(2,3) 0]';
    u_image = H*u_ground;
    pg_image = k_matrix*R*pg_world;
    pg_image_normalized = pg_image/pg_image(3);
    w = annotated_tg-pg_image_normalized;
    w_hat = pg_image + u_image;
    w_hat = w_hat/w_hat(3) - pg_image_normalized;
    b = dot(w,w_hat)/dot(w_hat,w_hat);
    left_tire_refined_2D = pg_image_normalized+b*w_hat;
    left_tire_refined_2D = left_tire_refined_2D/left_tire_refined_2D(3);
    left_tire_refined_2D_proj = inv(k_matrix*R)*left_tire_refined_2D;
    lambda_tg = poleheight/left_tire_refined_2D_proj(2);
    left_tground_3D = lambda_tg*inv(k_matrix*R)*left_tire_refined_2D;
end