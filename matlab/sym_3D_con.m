function [c,ceq] = sym_3D_con(k_matrix,R, sym_normal_unit,ds,x0)
    %Get the param
    sym_vis = [x0(1) x0(2)]';
    sym_nvis = [x0(3) x0(4)]';
    lambda_vis = x0(5);
    lambda_nvis = x0(6);
    sym_vis_rec_aug = [sym_vis; 1]; %Augment it
    sym_nvis_rec_aug = [sym_nvis; 1];
    p_nvis = inv(k_matrix*R)*sym_nvis_rec_aug;
    p_vis = inv(k_matrix*R)*sym_vis_rec_aug;
    ceq = [];
    %Enforcing two constraint
    ceq = [ceq abs(lambda_nvis*dot(sym_normal_unit,p_nvis)-(-ds + 1/2*norm(lambda_nvis*p_nvis - lambda_vis*p_vis)))];
    ceq = [ceq abs(lambda_vis*dot(sym_normal_unit,p_vis)-(-ds - 1/2*norm(lambda_nvis*p_nvis - lambda_vis*p_vis)))];
    c = 0;
end