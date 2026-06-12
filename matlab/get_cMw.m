function cMw = get_cMw(sym_normal_unit, origin_sym_coor,near_side)
%     origin_sym_coor = tire_extremal(1:3,3); %Near tireground first
%     x_axis = tire_extremal(1:3,4)-origin_sym_coor; %Always front to back
%     x_axis = x_axis/norm(x_axis);
    if  strcmp(near_side,'driver')
        z_axis = sym_normal_unit;
    else
        z_axis = -sym_normal_unit; %Always into the car so gotta reverse to make driver to passenger
    end


    Ry = acos(dot(z_axis, [0 0 1]'));
    
    Rwc = [cos(Ry) 0 sin(Ry);
        0 1 0;
        -sin(Ry) 0 cos(Ry)];
  
%     y_axis = [0 1 0]';
%     z_axis = z_axis/norm(z_axis);
%     x_axis = cross(y_axis, z_axis);
%     x_axis = x_axis/norm(x_axis);
%     Rwc = [x_axis y_axis z_axis];

%       x_axis = Rwc(:,1
%       y_axis = Rwc(:,2)
%       z_axis = Rwc(:,3)
%     rad2deg(acos(dot(x_axis/norm(x_axis),y_axis/norm(y_axis))))
%     rad2deg(acos(dot(y_axis/norm(y_axis),z_axis/norm(z_axis))))
%     norm(x_axis)
%     norm(y_axis)
%     norm(z_axis)

    threshold = 1e-12;
    assert(abs(det(Rwc) - 1)<threshold) 
    Rcw = inv(Rwc);
    t_vector = -Rcw*origin_sym_coor;
    cMw = [Rcw t_vector; 0 0 0 1];
end