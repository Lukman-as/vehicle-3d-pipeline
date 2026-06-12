function world2car_R = get_world2car_rotation(car_origin, sym_normal_unit)
    x_end = car_origin+sym_normal_unit;
    x_axis = x_end-car_origin;
    y_end = [car_origin(1) car_origin(2)+1 car_origin(3)]';
    y_axis = y_end -car_origin;
    % y_axis = y_axis/norm(y_axis);
    z_end = car_origin+cross(x_axis,y_axis);
    z_axis = z_end - car_origin;
    car_frame = [x_axis y_axis z_axis];
    rotx_angle = acos(dot(car_frame(:,1), [1 0 0]'));
    world2car_R = [cos(rotx_angle) 0 sin(rotx_angle);
            0 1 0;
        -sin(rotx_angle) 0 cos(rotx_angle);];
    disp("UNIT Test: Check the new sym normal and rotation - To be 0 and 90")
    world2car_T = -car_origin;
    x_axis_car = world2car_R*(x_end+world2car_T);
    sym_normal_car = world2car_R*(car_origin+sym_normal_unit+world2car_T);
    sym_normal_unit_car = sym_normal_car/norm(sym_normal_car);
    rad2deg(acos(dot(x_axis_car/norm(x_axis_car), sym_normal_unit_car)))
    z_axis_car = world2car_R*(z_end+world2car_T);
    rad2deg(acos(dot(z_axis_car/norm(z_axis_car), sym_normal_unit_car)))
end