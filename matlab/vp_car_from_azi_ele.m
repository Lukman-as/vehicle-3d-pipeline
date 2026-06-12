function vp_car = vp_car_from_azi_ele(azi,ele,R_horizon)
    vp_car = [cos(ele)*cos(azi),cos(ele)*sin(azi),sin(ele)]';
    vp_car = inv(R_horizon)*vp_car;
    if vp_car(3) < 0
       vp_car = -vp_car;
    end
    vp_car = vp_car/norm(vp_car);
    vp_car = vp_car./vp_car(3);
    assert(abs(vp_car(3)-1)<1e-4);
%     vp_car = 1/(vp_car(3))*vp_car;
end