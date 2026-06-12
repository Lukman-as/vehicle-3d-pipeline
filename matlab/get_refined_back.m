function tground_3D = get_refined_back(k_matrix,R,~,sym_normal_unit,...
    poleheight,annotated_tgs,backpoint,image,tground_centroid,near_tground,near_tground_back)
    ng = [0,1,0]';
    nl = sym_normal_unit;
    u = cross(nl, ng);
    u = u/norm(u);

    dg = -poleheight;

% No need you got a point of tground centroid already
%     A = [ng(1:2)' dg;
%         nl(1:2)' dl];
%     A_solve = rref(A);

    world_to_ground = [1 0 0; 0 0 1; 0 1/poleheight 0];
    H = k_matrix*R*inv(world_to_ground);

%     pg_world = [-A_solve(1,3) -A_solve(2,3) 0]'; %setting z = 0;
    pg_world = tground_centroid;
    pg_world_2 = pg_world+30*u;
    pg_ground = world_to_ground*(pg_world); %Degenerate problem
    pg_2_ground = world_to_ground*pg_world_2;
    homo_l = cross(pg_ground,pg_2_ground);
    H_inv_transpose = inv(H)';

    pg_ground_image = H*pg_ground;
    pg_ground_image_normalized = pg_ground_image/pg_ground_image(3);
    homo_l_image = H_inv_transpose*homo_l;


%   DEBUG purpose
    pg_world_image = k_matrix*R*pg_world;
    pg_world_image = pg_world_image./pg_world_image(3);
    pg_world_2_image = k_matrix*R*pg_world_2;
    pg_world_2_image = pg_world_2_image./pg_world_2_image(3);
    
    vec = pg_world_2-pg_world;
    rad2deg(acos(dot(sym_normal_unit,vec/norm(vec))))
    rad2deg(acos(dot(ng,vec/norm(vec))))

    rad2deg(acos(dot(sym_normal_unit,u/norm(u))))
    rad2deg(acos(dot(ng,u/norm(u))))

    sym_unit = pg_world+5*sym_normal_unit;
    sym_unit_image = k_matrix*R*sym_unit;
    sym_unit_image = sym_unit_image./sym_unit_image(3);

    gravity = pg_world+5*ng;
    gravity_image = k_matrix*R*gravity;
    gravity_image = gravity_image./gravity_image(3);

    if true
        figure
        %hold on
        imshow(image)
        hold on
        scatter(near_tground(1,:),near_tground(2,:),40,'r','filled');
        scatter(near_tground_back(1,:),near_tground_back(2,:),40,'r','filled'); 
        scatter(pg_world_image(1,:),pg_world_image(2,:),40,'blue','filled'); 

        line([pg_world_image(1), sym_unit_image(1)], [pg_world_image(2), sym_unit_image(2)],'Color','red','LineWidth',1,'LineStyle','--');
        line([pg_world_image(1), pg_world_2_image(1)], [pg_world_image(2), pg_world_2_image(2)],'Color','green','LineWidth',1,'LineStyle','--');
        line([pg_world_image(1), gravity_image(1)], [pg_world_image(2), gravity_image(2)],'Color','blue','LineWidth',1,'LineStyle','--');

        axis equal
        axis on
        xlabel('X')
        ylabel('Y')
    end


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