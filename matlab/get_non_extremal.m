function [non_ex1_l_3D, non_ex1_r_3D] = get_non_extremal(k_matrix,R,non_ex1_r_rec, non_ex1_l_rec,sym_normal_unit,ds)
    syms lambda_l lambda_r;
    eqn1 = lambda_r*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_r_rec)...
        == -ds + 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec - lambda_l*inv(k_matrix*R)*non_ex1_l_rec);
    eqn2 = lambda_l*dot(sym_normal_unit,inv(k_matrix*R)*non_ex1_l_rec)...
        == -ds - 1/2*norm(lambda_r*inv(k_matrix*R)*non_ex1_r_rec - lambda_l*inv(k_matrix*R)*non_ex1_l_rec);
    sol = vpasolve([eqn1, eqn2], [lambda_l, lambda_r]);
    lambda_l = sol.lambda_l;
    lambda_r = sol.lambda_r;
    non_ex1_l_3D = lambda_l*inv(k_matrix*R)*non_ex1_l_rec;
    non_ex1_r_3D = lambda_r*inv(k_matrix*R)*non_ex1_r_rec;
end