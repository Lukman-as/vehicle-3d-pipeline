function cMw = get_cMw_v2(sym_normal_unit, origin_sym_coor, visible)
    %Sym normal unit will become z_axis
    z_axis = sym_normal_unit;
    if visible == "P"
        z_axis = -z_axis; %Always into the car so gotta reverse to make driver to passenger
    end
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