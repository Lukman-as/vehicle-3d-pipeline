function all_annos = add_tire_pairs_horizon(annotated_car_id, all_annos,id_to_tire)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));    
    flag = 0;
    if ismember(1,all_annos.(fieldname).tire_ids) && ismember(2,all_annos.(fieldname).tire_ids)
       p1 = all_annos.(fieldname).tires.(id_to_tire(1));
       p2 = all_annos.(fieldname).tires.(id_to_tire(2));
       flag = 1;
    elseif ismember(4,all_annos.(fieldname).tire_ids) && ismember(3,all_annos.(fieldname).tire_ids)
       p1 = all_annos.(fieldname).tires.(id_to_tire(4));
       p2 = all_annos.(fieldname).tires.(id_to_tire(3));
       flag = 1;
    end
    if flag
        all_annos.(fieldname).tire_pairs_horizon = [p1 p2];
    end
end