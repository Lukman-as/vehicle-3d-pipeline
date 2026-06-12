function cMw = get_cMw_using_world_points(k_matrix,R,vp_car, origin_sym_coor,all_annos,annotated_car_id, all_3D_points,image)
    sym_normal_unit = flip_sym_normal_vpcar(k_matrix,R,all_annos,annotated_car_id,vp_car,image);
    %Scenario 1
    z_axis = sym_normal_unit;

    vec = [0 0 1]';
    c = vec(1)*z_axis(3)-vec(3)*z_axis(1);
    assert(c~=0);
    cos_Ry = dot(z_axis,vec); %Because both has norm 1 already

    if c > 0 
        sin_Ry = -norm(cross(z_axis,vec)); 
    else
        sin_Ry = norm(cross(z_axis,vec));
    end
    %Maintain z axis
    Rwc = [cos_Ry 0 sin_Ry;
        0 1 0;
        -sin_Ry 0 cos_Ry];

    threshold = 1e-6;
    assert(abs(det(Rwc) - 1)<threshold);
    Rcw = inv(Rwc);
    t_vector = -Rcw*origin_sym_coor;
    cMw = [Rcw t_vector; 0 0 0 1];
    cMw = double(cMw);

%     %Scenario 2
%     z_axis = -sym_normal_unit;
%     vec = [0 0 1]';
%     c = vec(1)*z_axis(3)-vec(3)*z_axis(1);
%     assert(c~=0);
%     cos_Ry = dot(z_axis,vec); %Because both has norm 1 already
%     if c > 0 
%         sin_Ry = -norm(cross(z_axis,vec)); 
%     else
%         sin_Ry = norm(cross(z_axis,vec));
%     end
%     %Maintain z axis
%     Rwc = [cos_Ry 0 sin_Ry;
%         0 1 0;
%         -sin_Ry 0 cos_Ry];
% 
%     threshold = 1e-6;
%     assert(abs(det(Rwc) - 1)<threshold);
%     Rcw = inv(Rwc);
%     t_vector = -Rcw*origin_sym_coor;
%     cMw = [Rcw t_vector; 0 0 0 1];
%     cMw2 = double(cMw);
% 
    %Selecting
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    start_pos = size(all_annos.(fieldname).tire_ids,2);
    ex_3D_1 = all_3D_points(:,start_pos+1);
    ex_3D_2 = all_3D_points(:,start_pos+2);
    ex_vec = ex_3D_2 - ex_3D_1;
    ex_vec = ex_vec/norm(ex_vec);
    ex_sym_angle = acos(dot(ex_vec, sym_normal_unit));

%     if abs(ex_sym_angle) >= 1e-3
%         ex_3D_1 = all_3D_points(:,start_pos+1)
%         ex_3D_2 = all_3D_points(:,start_pos+2)
%         ex_sym_angle
%     end
%     assert(abs(ex_sym_angle) < 1e-3, 'Alwasy valid angle auto')

    if abs(rad2deg(ex_sym_angle)) > 10
        'currently'
        rad2deg(ex_sym_angle)
        'if flipped then angle'
        rad2deg(acos(dot(ex_vec, -sym_normal_unit)))
    end

%     %Select to correct transformation
%     if abs(ex_sym_angle) < 1e-3
%         cMw = cMw1;
%     elseif abs(ex_sym_angle-pi) < 1e-3
%         cMw = cMw2;
%     end
end