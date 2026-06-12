function new_estimates = get_new_estimate(theta,k_matrix,R,sample2D_gt, estimated_3d_points,...
    non_extremal_indice, non_extremal_separations)
     %Parameters to tune
    vp_car = [theta(1),theta(2),theta(3)]';
    sym_normal = inv(k_matrix*R)*vp_car;
    sym_normal_unit = sym_normal/norm(sym_normal);

    %Assuming this is in CAR coordinate already
    w_unit = theta(4);
    ds = theta(5); %d of sym plane #Worth thinking about though
    dl = ds + w_unit/2;
    dr = ds - w_unit/2;
    left_plane = [sym_normal_unit; dl];
    right_plane = [sym_normal_unit; dr];


    %Fixed the estimated 3d_points for now
    %Getting a new set of 3D points
%     estimated_3d_points_to_2D = k_matrix*R*estimated_3d_points;
%     estimated_3d_points_to_2D = estimated_3d_points_to_2D./estimated_3d_points_to_2D(3,:);

    %Gotta change to car coordinate here
    car_origin = estimated_3d_points(:,11);
    world2car_R = get_world2car_rotation(car_origin, sym_normal_unit);
    world2car_T = -car_origin;
    

    %Enforcing constraint on left and right extremal
    left_extremal = sample2D_gt(:,6);
    left_extremal_proj_dir = inv(k_matrix*R)*left_extremal;
    left_lambda = -left_plane(4)/dot(left_extremal_proj_dir,left_plane(1:3));

    right_extremal = sample2D_gt(:,5);
    right_extremal_proj_dir = inv(k_matrix*R)*right_extremal;
    right_lambda = -right_plane(4)/dot(right_extremal_proj_dir,right_plane(1:3));

    %Get the initial guess, they are not nessarifily parallel to the sym
    %normal yet
    left_extremal_3D = left_extremal_proj_dir*left_lambda;
    right_extremal_3D = right_extremal_proj_dir*right_lambda;
%     YOU CAN ADD THE ADJUST ALONG TWO DIMENSION HERE
%     left_extremal_3D_car = world2car_R*(left_extremal_3D+world2car_T);
%     right_extremal_3D_car = world2car_R*(right_extremal_3D+world2car_T);
%     left_extremal_3D_car = left_extremal_3D_car + [0 theta(6) theta(7)]';
%     right_extremal_3D_car = right_extremal_3D_car + [0 theta(6) theta(7)]';
%     left_extremal_3D = inv(world2car_R)*left_extremal_3D_car - world2car_T;
%     right_extremal_3D = inv(world2car_R)*right_extremal_3D_car - world2car_T;

    %Change to coordinate system and then adjust more 
    %Enforcing parellel to normal
    vector_lr = right_extremal_3D-left_extremal_3D;
    t = (-ds-dot(left_extremal_3D,sym_normal_unit))...
        /(dot(vector_lr,sym_normal_unit));
    intersect_point = left_extremal_3D + t*vector_lr;
    right_extremal_3D_corrected = intersect_point + ...
        dot(right_extremal_3D-intersect_point,sym_normal_unit)*sym_normal_unit;
    left_extremal_3D_corrected = intersect_point + ...
        dot(left_extremal_3D-intersect_point,(-1)*sym_normal_unit)*(-1)*sym_normal_unit;
    left_extremal_3D = left_extremal_3D_corrected;
    right_extremal_3D = right_extremal_3D_corrected;

  
    %ORIGINAL WAy
%     left_extremal_3D = left_extremal_proj_dir*left_lambda;
%     right_extremal_3D = left_extremal_3D + w_unit*sym_normal_unit;

%     Unit test    
%     disp('HAS TO BE ZEROS');
%     vector_lr = right_extremal_3D-left_extremal_3D;
%     rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)/norm(sym_normal_unit)))
%     dot([left_extremal_3D; 1], left_plane)
%     dot([right_extremal_3D; 1], right_plane)


%     vector_lr = estimated_3d_points(:, 5)-estimated_3d_points(:, 6);
%     disp('THese may not be zeroes anymore');
%     rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)/norm(sym_normal_unit)))
%     dot([estimated_3d_points(:, 6); 1], left_plane)
%     dot([estimated_3d_points(:, 5); 1], right_plane)


    %=================================================
    %Experimenting with the lagrangian matrix
    D = inv(k_matrix*R)*sample2D_gt(:,11);
    D = D/norm(D);
    ng = [0,1,0]';
    dg = -4.8;
    nl = sym_normal_unit;
    dl = ds + w_unit/2;
    M = [-2*D(1).^2*(1-D(1).^2) 2*D(1).^2*D(2)*D(1)      2*D(1).^2*D(3)*D(1) ng(1) nl(1);
        2*D(2).^2*D(1)*D(2)   -2*D(2).^2*(1-D(2).^2)    2*D(2).^2*D(3)*D(2) ng(2) nl(2);
        2*D(3).^2*D(1)*D(3)   2*D(3).^2*D(2)*D(3)   -2*D(3).^2*(1-D(3).^2) ng(3) nl(3);
        ng(1) ng(2) ng(3) 0 0;
        nl(1) nl(2) nl(3) 0 0;
        ];
    N = [0 0 0 -dg -dl]';
    lpoint = linsolve(M,N);
    left_tground_3D = lpoint(1:3);
%     left_tground_3D_car = world2car_R*(left_tground_3D+world2car_T);
%     left_tground_3D_car = left_tground_3D_car + [0 0 theta(8)]';
%     left_tground_3D = inv(world2car_R)*left_tground_3D_car - world2car_T;

    
    %Unit test
%     disp('HAS TO BE ZEROS Both')
%     dot([left_tground_3D; 1], left_plane)
%     dot([left_tground_3D; 1], [ng; dg])

    %Tireground enforcing, enforcing on left plane and height
    %This one is not correct as y change
%     left_extremal = estimated_3d_points_to_2D(:,11);
%     left_extremal_proj_dir = inv(k_matrix*R)*left_extremal;
%     left_lambda = -left_plane(4)/dot(left_extremal_proj_dir,left_plane(1:3));
%     left_tground_3D = left_extremal_proj_dir*left_lambda;
   
%    Way 2 estimate
%     left_tire_ground_x = (-left_plane(4) - 4.8*left_plane(2) -...
%         estimated_3d_points(3,11)*left_plane(3))/left_plane(1);
%     left_tground_3D = [left_tire_ground_x, 4.8, estimated_3d_points(3,11)]';
%      left_tground_3D = estimated_3d_points(:,11)
%     dot([left_tground_3D 1], left_plane) = 0



    %OPTMIZING for one non extremal pair
    pair2_dl = ds + theta(9)/2;
    pair2_dr = ds - theta(9)/2;
    pair2_left_plane = [sym_normal_unit; pair2_dl];
    pair2_right_plane = [sym_normal_unit; pair2_dr];


    pair2_left = sample2D_gt(:,4);
    left_proj_dir = inv(k_matrix*R)*pair2_left;
    left_lambda = -pair2_left_plane(4)/dot(left_proj_dir,pair2_left_plane(1:3));

    pair2_right = sample2D_gt(:,3);
    right_proj_dir = inv(k_matrix*R)*pair2_right;
    right_lambda = -pair2_right_plane(4)/dot(right_proj_dir,pair2_right_plane(1:3));

    %Get the initial guess, they are not nessarifily parallel to the sym
    %normal yet
    pair2_left_3D = left_proj_dir*left_lambda;
    pair2_right_3D = right_proj_dir*right_lambda;

%     YOU CAN ADD THE ADJUST ALONG TWO DIMENSION HERE
%     pair2_left_3D_car = world2car_R*(pair2_left_3D+world2car_T);
%     pair2_right_3D_car = world2car_R*(pair2_right_3D+world2car_T);
%     pair2_left_3D_car = pair2_left_3D_car + [0 theta(10) theta(11)]';
%     pair2_right_3D_car = pair2_right_3D_car + [0 theta(10) theta(11)]';
%     pair2_left_3D = inv(world2car_R)*pair2_left_3D_car - world2car_T;
%     pair2_right_3D = inv(world2car_R)*pair2_right_3D_car - world2car_T;

    %Change to coordinate system and then adjust more 
    %Enforcing parellel to normal
    vector_lr = pair2_left_3D-pair2_right_3D;
    t = (-ds-dot(pair2_left_3D,sym_normal_unit))...
        /(dot(vector_lr,sym_normal_unit));
    intersect_point = pair2_left_3D + t*vector_lr;
    pair2_right_3D = intersect_point + ...
        dot(pair2_right_3D-intersect_point,sym_normal_unit)*sym_normal_unit;
    pair2_left_3D = intersect_point + ...
        dot(pair2_left_3D-intersect_point,(-1)*sym_normal_unit)*(-1)*sym_normal_unit;

        %Unit test
    disp('HAS TO BE ZEROS');
    vector_lr = pair2_right_3D-pair2_left_3D;
    rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)/norm(sym_normal_unit)))
    dot([pair2_left_3D; 1], pair2_left_plane)
    dot([pair2_right_3D; 1], pair2_right_plane)


%  Only change certain things, other than keep as usual
   all_refined_points = estimated_3d_points;
    all_refined_points(:,6) = left_extremal_3D;
    all_refined_points(:,5) = right_extremal_3D;
%     all_refined_points(:,4) = pair2_left_3D;
%     all_refined_points(:,3) = pair2_right_3D;
    all_refined_points(:,11) = left_tground_3D;

%    %Tackling the non extremal ones
%    for k = 1: size(non_extremal_indice,1)
%          a = estimated_3d_points(:,non_extremal_indice(k,1));
%          b = estimated_3d_points(:,non_extremal_indice(k,2));
% %          a(2) = theta(8+2*k-1);
% %          b(2) = theta(8+2*k);
% %          a(3) = theta(16+2*k-1);
% %          b(3) = theta(16+2*k); 
%          non_ex1_l_3D = a;
%          non_ex1_r_3D = b;
%          non_ex_left_x = -ds+0.5*(non_extremal_separations(k)) - ...
%              sym_normal_unit(2)*non_ex1_l_3D(2) - sym_normal_unit(3)*non_ex1_l_3D(3);
%          non_ex_left_x = non_ex_left_x/sym_normal_unit(1);
%          non_ex_right_x = -ds+0.5*(non_extremal_separations(k)) - ...
%              sym_normal_unit(2)*non_ex1_r_3D(2) - sym_normal_unit(3)*non_ex1_r_3D(3);
%          non_ex_right_x = non_ex_right_x/sym_normal_unit(1);
%          %Create new points in
%          non_ex1_l_3D = [non_ex_left_x; non_ex1_l_3D(2:3)];
%          non_ex1_r_3D = [non_ex_right_x; non_ex1_r_3D(2:3)];    
%          all_refined_points(:,non_extremal_indice(k,1)) = non_ex1_l_3D; %Save the generated points
%          all_refined_points(:,non_extremal_indice(k,2)) = non_ex1_r_3D;
%    end
   new_estimates = all_refined_points;
end