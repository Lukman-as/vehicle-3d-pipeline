function all_annos = center_points_3D(k_matrix,R,annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    if isfield(all_annos.(fieldname),'center')
        a = inv(k_matrix*R)*all_annos.(fieldname).center;
        center_3D = zeros(3,size(all_annos.(fieldname).center,2));
        for i = 1:size(a,2)
            lambda = -all_annos.(fieldname).ds/dot(a(:,i),all_annos.(fieldname).sym_normal_unit);
            point_3D = lambda*inv(k_matrix*R)*all_annos.(fieldname).center(:,i);
            center_3D(:,i) = point_3D;
        end
        all_annos.(fieldname).center_3D = center_3D;
    end
end