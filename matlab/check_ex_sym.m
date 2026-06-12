function valid = check_ex_sym(all_annos,annotated_car_id)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    start_pos = size(all_annos.(fieldname).tire_ids,2);
    ex_3D_1 = all_annos.(fieldname).all_3D_points(:,start_pos+1);
    ex_3D_2 = all_annos.(fieldname).all_3D_points(:,start_pos+2);
    ex_vec = ex_3D_2 - ex_3D_1;
    ex_vec = ex_vec/norm(ex_vec);
    ex_sym_angle_1 = abs(rad2deg(acos(dot(ex_vec, all_annos.(fieldname).sym_normal_unit))));
    ex_sym_angle_2 = abs(rad2deg(acos(dot(ex_vec, -all_annos.(fieldname).sym_normal_unit))));
    if ex_sym_angle_1 > 10 && ex_sym_angle_2 > 10
        valid = false;
    else
        valid = true;
    end