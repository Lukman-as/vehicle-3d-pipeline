function fErr = sym_3D_fun(k_matrix,R,sym_nvis_raw_aug,sym_vis_raw_aug, x0)
    %Get the param
    sym_vis = [x0(1) x0(2)]';
    sym_nvis = [x0(3) x0(4)]';
    lambda_vis = x0(5);
    lambda_nvis = x0(6);
   
    sym_vis_rec_aug = [sym_vis; 1]; %Augment it
    sym_nvis_rec_aug = [sym_nvis; 1];
    p_nvis = inv(k_matrix*R)*sym_nvis_rec_aug;
    p_vis = inv(k_matrix*R)*sym_vis_rec_aug;
    sym_vis_3D = lambda_vis*p_vis;
    sym_nvis_3D = lambda_nvis*p_nvis;
    sym_vis_2D = k_matrix*R*sym_vis_3D;
    sym_vis_2D = sym_vis_2D./sym_vis_2D(3,:);
    sym_nvis_2D = k_matrix*R*sym_nvis_3D;
    sym_nvis_2D = sym_nvis_2D./sym_nvis_2D(3,:);
    
    all_refined_points = [sym_vis_2D sym_nvis_2D];
    sample2D_gt = [sym_vis_raw_aug sym_nvis_raw_aug];

    fErr = ((sum((all_refined_points-sample2D_gt).^2,'all'))...
    /size(sample2D_gt,2))...
    .^(0.5);
end