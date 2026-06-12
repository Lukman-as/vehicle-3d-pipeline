function all_annos = get_reproj_error(k_matrix,R,annotated_car_id,all_annos,id_to_tire)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    order = struct;
    num_points = 0;
    num_points = num_points + size(all_annos.(fieldname).tire_ids,2);
    order.('tire') = num_points;
    num_points = num_points + size(all_annos.(fieldname).ex_3D,2);
    order.('ex_3D') = num_points;
%     num_points = num_points + size(all_annos.(fieldname).center_3D,2);
%     order.('center_3D') = num_points;
    num_points = num_points + size(all_annos.(fieldname).nonex_3D,2);
    order.('nonex_3D') = num_points;
    all_annos.(fieldname).num_points = num_points;
    all_annos.(fieldname).order = order;

    %Start assembling
    all_3D_points = zeros(3,num_points);
    all_annotated_points = zeros(3,num_points);

    %Start adding in
    start_pos = 0;
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        a_tire = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
        all_3D_points(:,start_pos+i) = a_tire;
        a_tire = all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
        all_annotated_points(:,start_pos+i) = a_tire;
    end
    start_pos = start_pos+size(all_annos.(fieldname).tire_ids,2);
    if isfield(all_annos.(fieldname),'ex_3D')
%         all_3D_points(:,start_pos+1) = all_annos.(fieldname).ex_3D(:,1);
%         all_3D_points(:,start_pos+2) = all_annos.(fieldname).ex_3D(:,2);
%         all_annotated_points(:,start_pos+1) = all_annos.(fieldname).ex(:,1);
%         all_annotated_points(:,start_pos+2) = all_annos.(fieldname).ex(:,2);
%         start_pos = start_pos+size(all_annos.(fieldname).ex_3D,2);
        for i = 1: size(all_annos.(fieldname).ex_3D,2)
            a_point = all_annos.(fieldname).ex_3D(:,i); 
            all_3D_points(:,start_pos+i) = a_point;
            a_point = all_annos.(fieldname).ex(:,i);
            all_annotated_points(:,start_pos+i) = a_point;
        end
        start_pos = start_pos+size(all_annos.(fieldname).ex_3D,2);
    end
    if isfield(all_annos.(fieldname),'center_3D')
        for i = 1: size(all_annos.(fieldname).center_3D,2)
            a_point = all_annos.(fieldname).center_3D(:,i); 
            all_3D_points(:,start_pos+i) = a_point;
            a_point = all_annos.(fieldname).center(:,i);
            all_annotated_points(:,start_pos+i) = a_point;
        end
        start_pos = start_pos+size(all_annos.(fieldname).center_3D,2);
    end
    for i = 1: size(all_annos.(fieldname).nonex_3D,2)
        a_point = all_annos.(fieldname).nonex_3D(:,i); %point driver
        all_3D_points(:,start_pos+i) = a_point;
        a_point = all_annos.(fieldname).nonex(:,i); %point driver
        all_annotated_points(:,start_pos+i) = a_point;
    end

    all_3D_points_to_2D = k_matrix*R*all_3D_points;
    all_3D_points_to_2D = all_3D_points_to_2D./all_3D_points_to_2D(3,:); 
    all_annos.(fieldname).all_3D_points_to_2D = all_3D_points_to_2D;
    all_annos.(fieldname).all_3D_points = all_3D_points;
    all_annos.(fieldname).all_annotated_points = all_annotated_points;
    reproj_error = ((sum((all_3D_points_to_2D-all_annotated_points).^2,'all'))...
        /size(all_3D_points_to_2D,2))...
        .^(0.5);
    all_annos.(fieldname).reproj_error = reproj_error;
end
