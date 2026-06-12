function dLambda = dfitgamma(X,k_matrix,R,non_ex1_l_rec,non_ex1_r_rec,sym_normal_unit,ds,mounting_height)
   dLambda = nan(size(X));
   h = 1e-2; % this is the step size used in the finite difference.
   for i=1:numel(X)
       dX=zeros(size(X));
       dX(i) = h;
       dLambda(i) = (fitgamma(X+dX,k_matrix,R,non_ex1_l_rec,non_ex1_r_rec,sym_normal_unit,ds,mounting_height)...
           -fitgamma(X-dX,k_matrix,R,non_ex1_l_rec,non_ex1_r_rec,sym_normal_unit,ds,mounting_height))/(2*h);
   end
end