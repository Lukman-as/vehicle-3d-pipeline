function gamma = fitgamma(X,k_matrix,R,non_ex1_l_rec,non_ex1_r_rec,sym_normal_unit,ds,mounting_height)
    lambda_l = X(1);
    lambda_r = X(2);
    non_ex1_l_3D = lambda_l*inv(k_matrix*R)*non_ex1_l_rec;
    non_ex1_r_3D = lambda_r*inv(k_matrix*R)*non_ex1_r_rec;
    sigma1 = X(3);
    sigma2 = X(4);
    non_ex1_l_3D_2D = k_matrix*R*non_ex1_l_3D;
    non_ex1_l_3D_2D = non_ex1_l_3D_2D/non_ex1_l_3D_2D(3);
    non_ex1_r_3D_2D = k_matrix*R*non_ex1_r_3D;
    non_ex1_r_3D_2D = non_ex1_r_3D_2D/non_ex1_r_3D_2D(3);
    cost = norm(non_ex1_l_3D_2D - non_ex1_l_rec)+ norm(non_ex1_r_3D_2D - non_ex1_r_rec);
    a = dot(sym_normal_unit, (non_ex1_l_3D+non_ex1_r_3D)) + 2*ds;
    b = dot(sym_normal_unit, (non_ex1_r_3D-non_ex1_l_3D)) - norm(non_ex1_r_3D-non_ex1_l_3D);
%     disp('HAS TO BE CLOSE TO Zero, ENFORCING THE EQUATION 18');
%     dot(sym_normal_unit,non_ex1_r_3D+non_ex1_l_3D)+2*ds
%     disp('HAS TO BE CLOSE THE SAME, ENFORCING THE EQUATION 19');
%     vector_lr = non_ex1_r_3D-non_ex1_l_3D;
%     dot(sym_normal_unit,vector_lr)
%     norm(vector_lr)
%     dot(vector_lr/norm(vector_lr),sym_normal_unit)
%     rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)))
% 
%     %Enforcing symline
%     vector_lr = non_ex1_r_3D_2D-non_ex1_l_3D_2D;
%     vector_lr = vector_lr(1:2);
%     vp_car = k_matrix*R*sym_normal_unit;
%     vp_car = vp_car/vp_car(3);
%     sym_line = vp_car-non_ex1_l_3D_2D;
%     sym_line = sym_line(1:2);
%     disp('HAS TO BE CLOSE TO 1 TO ENFORCE SYMLINE');
%     dot(vector_lr/norm(vector_lr),sym_line/norm(sym_line))
    gamma = cost + sigma1*a + sigma2*b;
end