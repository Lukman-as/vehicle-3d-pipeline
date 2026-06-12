function all_annos = get_reproj_error_with_3D_points(k_matrix,R,annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    all_3D_points_to_2D = k_matrix*R*all_annos.(fieldname).all_3D_points;
    all_3D_points_to_2D = all_3D_points_to_2D./all_3D_points_to_2D(3,:); 
    all_annos.(fieldname).all_3D_points_to_2D = all_3D_points_to_2D;    
    reproj_error = ((sum((all_3D_points_to_2D-all_annos.(fieldname).all_annotated_points).^2,'all'))...
        /size(all_3D_points_to_2D,2))...
        .^(0.5);
    all_annos.(fieldname).reproj_error = reproj_error;
end
