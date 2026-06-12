function [k_matrix, R, t] = decompose_projection(projection_matrix)
    [U,S,V] = svd(projection_matrix);
    C = V(1:3,end) / V(end, end) %Normalized
    [K,R] = rq(projection_matrix(1:3,1:3));
    D = diag(sign(diag(K)));
    K = K*D;
    R = D*R;
    k_matrix = K/K(end,end);
    t = -R*C;
end