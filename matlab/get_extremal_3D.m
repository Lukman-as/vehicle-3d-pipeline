function [w_unit, near_extremal_3D, far_extremal_3D] = get_extremal_3D(k_matrix, R, near_extremal, far_extremal, near_plane_normalized)
    sym_normal_unit = near_plane_normalized(1:3);
    near_extremal_proj_dir = inv(k_matrix*R)*near_extremal;
    near_lambda = -near_plane_normalized(4)/dot(near_extremal_proj_dir,near_plane_normalized(1:3));
    near_extremal_3D = near_extremal_proj_dir*near_lambda;
    
    %Based on the formulation that near_extremal dot Mnear = 0 
    %Formulation direction of far point
    far_extremal_proj_dir = inv(k_matrix*R)*far_extremal;
    far_extremal_normal = cross(far_extremal_proj_dir, cross(sym_normal_unit,...
        far_extremal_proj_dir));
    
    %Find the w
    far_extremal_w = -dot(near_extremal_3D,far_extremal_normal)...
        / dot(far_extremal_normal, sym_normal_unit);
    
    %Find the extremal point
    far_extremal_3D = near_extremal_3D + far_extremal_w*sym_normal_unit;

%     w_unit = far_extremal_w*norm(sym_normal_unit) %Fix this thing
%     assert(w_unit > 0)
    %Calculating using the norm instead
    w_unit = norm(far_extremal_3D-near_extremal_3D);
    assert(w_unit > 0);
end


