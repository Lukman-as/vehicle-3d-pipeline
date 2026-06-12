function [cMw, sym_normal_unit, all_3d_points_tire_tire_coor] = get_3Dpoints_car_from_x(k_matrix,R,num_sympairs,near_side,poleheight,x0)
    vp_car = [x0(1:2) 1]';
    sym_normal = inv(k_matrix*R)*vp_car;
    sym_normal_unit = sym_normal/norm(sym_normal);
    origin = [x0(3) poleheight x0(4)]';

    cMw = get_cMw(sym_normal_unit,origin,near_side);
    num_sympoints = num_sympairs*2;
    
    temp = [x0(5:5+num_sympairs-1);x0(5+num_sympairs:5+2*num_sympairs-1)];
    rearrange = [];
    for i = 1:size(temp,2)
        rearrange = [rearrange, repmat(temp(:,i),1,2)];
    end
    rearrange = [rearrange; zeros(1, size(rearrange,2))];
    seps = x0(5+2*num_sympairs:end-1);

    %This need to change when different origin is used, origin currently at
    % 2 in the map
    midpoint_z_coor = -seps(1)/2;
    
    %This gotta change when orgin change too
    for j = 1:size(rearrange, 2)/2
         rearrange(3,2*j-1) = midpoint_z_coor+seps(j)/2;
         rearrange(3,2*j) = midpoint_z_coor-seps(j)/2;
    end

    all_3d_points_tire_tire_coor = [rearrange [0 0 0]' [x0(end) 0 0]'];
end    