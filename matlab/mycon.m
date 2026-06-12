function [c,ceq] = mycon(k_matrix,R, num_sympairs, num_tire_points, near_side, poleheight, horizon_line,x0)
    [cMw, sym_normal_unit, all_3d_points_tire_tire_coor]  = get_3Dpoints_car_from_x(k_matrix,R,num_sympairs,near_side,poleheight,x0);
    
    car_z_axis = [0 0 1]';
    %Get all the info
    midpoint = [0 0 mean(all_3d_points_tire_tire_coor(3,1:2))]';
    ds_tire_coor = -dot(car_z_axis,midpoint);
    d_driver_tire_coor = -dot(car_z_axis,all_3d_points_tire_tire_coor(:,2));
    d_passenger_tire_coor = -dot(car_z_axis,all_3d_points_tire_tire_coor(:,1));

    ceq = [];
    %Enforcing sym unit equal 0
    ceq = [ceq dot(k_matrix*R*sym_normal_unit,horizon_line)];
%     ceq = [ceq sym_normal_unit(2)];

%     %Enforcing constraint 18
%     for i = 1:num_sympairs
%         ceq = [ceq dot(car_z_axis, all_3d_points_tire_tire_coor(:,2*i-1) + all_3d_points_tire_tire_coor(:,2*i)) + 2*ds_tire_coor];
%     end
% 
%     %Enforcing constraint 19
%     sym_indice = [2,1;
%                    4,3;
%                    6,5;
%                    8,7;
%                    10,9];
% 
%     for i = 1:size(sym_indice,1)
%         vec = all_3d_points_tire_tire_coor(:,sym_indice(i,2))-all_3d_points_tire_tire_coor(:,sym_indice(i,1));
%         re = dot(car_z_axis, vec) - norm(vec);
%         ceq = [ceq re];
%     end
%     
%     %Enforcing near tire points on near plane, far tire points on far plane
%     passenger_indices = [1 11 12];
%     for i = 1:size(passenger_indices,2)
%         ceq = [ceq dot(car_z_axis, all_3d_points_tire_tire_coor(:,passenger_indices(i))) + d_passenger_tire_coor];
%     end
%     
%     driver_indices = [2];
%     for i = 1:size(driver_indices,2)
%         ceq = [ceq dot(car_z_axis, all_3d_points_tire_tire_coor(:,driver_indices(i))) + d_driver_tire_coor];
%     end

    %Enforcing tire point must remain on the ground
    all_3d_points_tire_tire_coor = [all_3d_points_tire_tire_coor; ones(1,size(all_3d_points_tire_tire_coor,2))];
    estimated_3d_points = inv(cMw)*all_3d_points_tire_tire_coor;
    estimated_3d_points = estimated_3d_points(1:3,:);

    ground_indices = [11 12];
    for i = 1:size(ground_indices,2)
        ceq = [ceq estimated_3d_points(2,ground_indices(i)) - poleheight];
    end
    c = 0;
end