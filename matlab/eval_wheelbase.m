function all_annos = eval_wheelbase(annotated_car_id,all_annos,id_to_tire)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    if ismember(1,all_annos.(fieldname).tire_ids) && ismember(4,all_annos.(fieldname).tire_ids)
       all_annos.(fieldname).wheelbase = norm(all_annos.(fieldname).tires_3D.(id_to_tire(1))...
           - all_annos.(fieldname).tires_3D.(id_to_tire(4)));
    end
    if ismember(2,all_annos.(fieldname).tire_ids) && ismember(3,all_annos.(fieldname).tire_ids)
       all_annos.(fieldname).wheelbase = norm(all_annos.(fieldname).tires_3D.(id_to_tire(2))...
           - all_annos.(fieldname).tires_3D.(id_to_tire(3)));
    end
    if ~isfield(all_annos.(fieldname),'wheelbase')
        all_annos.(fieldname).wheelbase = -1; %If do not exist then it is -1
    end
end