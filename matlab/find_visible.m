function all_annos = find_visible(annotated_car_id, all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    %Find the more visible side
    count_points = struct;
    count_points.D = 0;
    count_points.P = 0;
    for k = 1:size(all_annos.(fieldname).tire_ids,2)
        if all_annos.(fieldname).tire_ids(k) == 1 || all_annos.(fieldname).tire_ids(k) == 4
            count_points.D = count_points.D +1;
        else
            count_points.P = count_points.P +1;
        end
    end
    
    if count_points.D >= count_points.P
        all_annos.(fieldname).visible = "D";
    else
        all_annos.(fieldname).visible = "P";
    end

    if count_points.D > 1 && all_annos.(fieldname).visible == "D"
        all_annos.(fieldname).car_origin = 1;  %Prioritize when have 2 points
    elseif count_points.D == 1 && all_annos.(fieldname).visible == "D"
        if isfield(all_annos.(fieldname).tires,"DF")
            all_annos.(fieldname).car_origin = 1;
        else
            all_annos.(fieldname).car_origin = 4;
        end
    end
    
    if count_points.P > 1 && all_annos.(fieldname).visible == "P"
        all_annos.(fieldname).car_origin = 2;
    elseif count_points.P == 1 && all_annos.(fieldname).visible == "P"
        if isfield(all_annos.(fieldname).tires,"PF")
            all_annos.(fieldname).car_origin = 2;
        else
            all_annos.(fieldname).car_origin = 3;
        end
    end
end