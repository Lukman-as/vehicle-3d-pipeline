function [pair_left_3D pair_right_3D] = refine_non_extremal(ds,sym_normal_unit, k_matrix,R,...
    separation, pair_left, pair_right,backpoint)
    %SECTION 1: Find all the symmetry point pairs
    pair_dl = ds + separation/2;
    pair_dr = ds - separation/2;
    pair_left_plane = [sym_normal_unit; pair_dl];
    pair_right_plane = [sym_normal_unit; pair_dr];

    left_proj_dir = inv(k_matrix*R)*pair_left;
%     left_lambda = -pair_left_plane(4)/dot(left_proj_dir,pair_left_plane(1:3));
    left_lambda = (-ds-separation/2)/dot(left_proj_dir,sym_normal_unit);

    right_proj_dir = inv(k_matrix*R)*pair_right;
%     right_lambda = -pair_right_plane(4)/dot(right_proj_dir,pair_right_plane(1:3));
    right_lambda = (-ds+separation/2)/dot(right_proj_dir,sym_normal_unit);

    %Get the initial guess, they are not nessarifily parallel to the sym
    %normal yet
    pair_left_3D = left_proj_dir*left_lambda;
    pair_right_3D = right_proj_dir*right_lambda;

%     disp('CALCULATE EQUATION 19 TO SEE');
%     dot(sym_normal_unit, pair_right_3D) - dot(sym_normal_unit, pair_left_3D)
%     (-ds + separation/2) - (-ds - separation/2)
%     norm(pair_right_3D-pair_left_3D)

%     disp('HAS TO BE ZEROS, ENFORCING THE EQUATION 18');
%     dot(sym_normal_unit,pair_right_3D+pair_left_3D)+2*ds
% 
%     disp('HAS TO BE ZEROS, ENFORCING THE EQUATION 19');
%     vector_lr = pair_right_3D-pair_left_3D;
%     dot(sym_normal_unit,vector_lr)-norm(vector_lr)
%     disp('HAS TO BE ZEROS, ANGLE BETWEEN VECTOR LEFT RIGHT AND SYM NORMAL');
%     rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)/norm(sym_normal_unit)))
% 
%     disp('HAS TO BE ZEROS AS THEY HAVE TO BE ON LEFT AND RIGHT PLANE');
%     dot([pair_left_3D; 1], pair_left_plane)
%     dot([pair_right_3D; 1], pair_right_plane)

 
    %Enforcing parellel to normal
    vector_lr = pair_right_3D-pair_left_3D;
    t = (-ds-dot(pair_left_3D,sym_normal_unit))...
        /(dot(vector_lr,sym_normal_unit));
    intersect_point = pair_left_3D + t*vector_lr;
    pair_right_3D_corrected = intersect_point + ...
        dot(pair_right_3D-intersect_point,sym_normal_unit)*sym_normal_unit;
    pair_left_3D_corrected = intersect_point + ...
        dot(pair_left_3D-intersect_point,sym_normal_unit)*sym_normal_unit;
%     disp('Gotto be small');
%     norm(pair_left_3D_corrected-pair_left_3D)
%     norm(pair_right_3D_corrected-pair_right_3D)
    pair_left_3D = pair_left_3D_corrected;
    pair_right_3D = pair_right_3D_corrected;

%     disp('HAS TO BE ZEROS, ENFORCING THE EQUATION 18');
%     dot(sym_normal_unit,pair_right_3D+pair_left_3D)+2*ds
% 
%     disp('HAS TO BE ZEROS, ENFORCING THE EQUATION 19');
%     vector_lr = pair_right_3D-pair_left_3D;
%     dot(sym_normal_unit,vector_lr)-norm(vector_lr)
%     disp('HAS TO BE ZEROS, ANGLE BETWEEN VECTOR LEFT RIGHT AND SYM NORMAL');
%     rad2deg(acos(dot(vector_lr/norm(vector_lr),sym_normal_unit)/norm(sym_normal_unit)))
% 
%     disp('HAS TO BE ZEROS AS THEY HAVE TO BE ON LEFT AND RIGHT PLANE');
%     dot([pair_left_3D; 1], pair_left_plane)
%     dot([pair_right_3D; 1], pair_right_plane)


end