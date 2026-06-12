function cMw = get_cMw_v3(sym_normal_unit, origin_sym_coor, ex_3D_points, during_opt)
    ex_vec = ex_3D_points(:,2) - ex_3D_points(:,1); %Vector going from driver to passenger
    ex_vec = ex_vec/norm(ex_vec)
    angle = acos(dot(sym_normal_unit, ex_vec))

    %Sym normal unit will become z_axis
    z_axis = sym_normal_unit;
    if abs(angle) > pi/4 %Gotta flip when different angle 
        z_axis = -z_axis; %Always into the car so gotta reverse to make driver to passenger
    end
    if ~during_opt
        angle = acos(dot(z_axis, ex_vec));
        assert(abs(angle) < 1e-1, "Z axis of car has to go from driver to passenger");
    end
%     else
%         pass
% %         'during opt'
% %         angle = acos(dot(z_axis, ex_vec));
%     end
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
end