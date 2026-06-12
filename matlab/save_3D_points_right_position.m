function all_annos = save_3D_points_right_position(all_annos,fieldname,id_to_tire,all_3D_points,origin)
    %INITIALLY
    %Points saved
    all_annos.(fieldname).all_3D_points = all_3D_points;
    %Save car origin
    all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin)) = origin;
    %Saving all points
    start_pos = 0;
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i))) = all_3D_points(:,start_pos+i);
    end

    %Save wheelbase:
    if ismember(1,all_annos.(fieldname).tire_ids) && ismember(4,all_annos.(fieldname).tire_ids)
       all_annos.(fieldname).wheelbase = norm(all_annos.(fieldname).tires_3D.(id_to_tire(1))...
           - all_annos.(fieldname).tires_3D.(id_to_tire(4)));
    elseif ismember(2,all_annos.(fieldname).tire_ids) && ismember(3,all_annos.(fieldname).tire_ids)
       all_annos.(fieldname).wheelbase = norm(all_annos.(fieldname).tires_3D.(id_to_tire(2))...
           - all_annos.(fieldname).tires_3D.(id_to_tire(3)));
    end

    start_pos = start_pos+size(all_annos.(fieldname).tire_ids,2);
    for i = 1: size(all_annos.(fieldname).ex_3D,2)
        all_annos.(fieldname).ex_3D(:,i) = all_3D_points(:,start_pos+i);
    end
    start_pos = start_pos+size(all_annos.(fieldname).ex_3D,2);
    
    for i = 1: size(all_annos.(fieldname).nonex_3D,2)
        all_annos.(fieldname).nonex_3D(:,i) = all_3D_points(:,start_pos+i);
    end
end