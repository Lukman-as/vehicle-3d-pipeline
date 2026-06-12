%Get the actual data points from experiment or from anotation
function fErr = lsqFun(theta,k_matrix,R,sample2D_gt, estimated_3d_points,...
    non_extremal_indice,poleheight,backpoint)
    all_refined_points = get_new_estimate(theta,k_matrix,R,sample2D_gt, estimated_3d_points,...
    non_extremal_indice,poleheight,backpoint);
    %Calculate the prediction
    all_refined_points = k_matrix*R*all_refined_points;
    all_refined_points = all_refined_points./all_refined_points(3,:);
    %Calculate error
    fErr = sum(([sample2D_gt(:,:)]-[all_refined_points(:,:)]).^2,'all');
end