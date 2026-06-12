function [sym_vis_3D, sym_nvis_3D] = get_3D_from_x0_coop(k_matrix,R, x0)
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
%     sym_vis_3D = round(sym_vis_3D,3);
%     sym_nvis_3D = round(sym_nvis_3D,3);
end