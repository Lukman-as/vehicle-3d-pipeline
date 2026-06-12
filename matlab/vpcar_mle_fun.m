function fErr = vpcar_mle_fun(x1_cov_inv,x2_cov_inv,anno_sympoints, x0)
    %Get the param
    total = 0;
    for k = 1: size(anno_sympoints,2)/2
        sym_vis = anno_sympoints(1:2,2*k-1);
        sym_nvis = anno_sympoints(1:2,2*k);
        sym_vis_rec = x0(1:2,2*k-1);
        sym_nvis_rec = x0(1:2,2*k);
        x1 = sym_vis;
        x1_mle = sym_vis_rec;
        x1_dist = (x1-x1_mle)'*x1_cov_inv*(x1-x1_mle);
        x2 = sym_nvis;
        x2_mle = sym_nvis_rec;
        x2_dist = (x2-x2_mle)'*x2_cov_inv*(x2-x2_mle);
        total = total + x1_dist + x2_dist;
    end   
    fErr = total;
end