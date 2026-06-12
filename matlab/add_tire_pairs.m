function all_annos = add_tire_pairs(annotated_car_id, all_annos,id_to_tire,k_matrix,R,mounting_height)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        tground = all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
        temp =  inv(k_matrix*R)*tground;
        lambda = all_annos.(fieldname).mounting_height/temp(2);
        tground_3D = [temp(1)*lambda all_annos.(fieldname).mounting_height temp(3) * lambda]';
        all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i))) = tground_3D;
    end

    flag = 0;
    if ismember(1,all_annos.(fieldname).tire_ids) && ismember(4,all_annos.(fieldname).tire_ids)
       p1 = all_annos.(fieldname).tires_3D.(id_to_tire(1));
       p2 = all_annos.(fieldname).tires_3D.(id_to_tire(4));
       flag = 1;

    elseif ismember(2,all_annos.(fieldname).tire_ids) && ismember(3,all_annos.(fieldname).tire_ids)
       p1 = all_annos.(fieldname).tires_3D.(id_to_tire(2));
       p2 = all_annos.(fieldname).tires_3D.(id_to_tire(3));
       flag = 1;
    end

    if flag
        slope = -1/((p1(3)-p2(3))/(p1(1)-p2(1)));
        y_inter = p1(3) - slope*p1(1);
        p_new_x = p1(1)+1;
        p_new_y = slope*p_new_x+y_inter;
        p_new = [p_new_x p1(2) p_new_y]';
        p_new_2D = k_matrix*R*p_new;
        p_new_2D = p_new_2D/p_new_2D(3);
        p1_2D = k_matrix*R*p1;
        p1_2D = p1_2D/p1_2D(3);
        all_annos.(fieldname).tire_pairs = [p1_2D p_new_2D];
    end
end