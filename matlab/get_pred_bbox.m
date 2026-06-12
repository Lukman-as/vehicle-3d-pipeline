function all_annos = get_pred_bbox(k_matrix,R,annotated_car_id,all_annos,id_to_tire,car_model_param,image,mirror_one_side)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    origin_sym_coor = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin));
    cMw = get_cMw_using_world_points(k_matrix,R,all_annos.(fieldname).vp_car, origin_sym_coor,...
        all_annos,annotated_car_id, all_annos.(fieldname).all_3D_points,image);
    all_3D_points_car_coor = cMw*[all_annos.(fieldname).all_3D_points;...
        ones(1,size(all_annos.(fieldname).all_3D_points,2))];

    %Points only
    predicted_corners_car_coor = get_bbox_from_points(all_3D_points_car_coor,true, all_annos.(fieldname).has_mirror, mirror_one_side,all_annos.(fieldname).w_unit);
    predicted_corners = inv(cMw)*predicted_corners_car_coor;
    all_annos.(fieldname).pred_bbox_points_only = predicted_corners(1:3,:);

    %Combine with gaussian model
    [get_length_gaussian, pred_bbox_gaussian] = get_bbox_from_gaussian_model(all_3D_points_car_coor,all_annos, fieldname,car_model_param,id_to_tire, mirror_one_side); 
    pred_bbox = inv(cMw)*pred_bbox_gaussian;
    all_annos.(fieldname).pred_bbox = pred_bbox(1:3,:);
end