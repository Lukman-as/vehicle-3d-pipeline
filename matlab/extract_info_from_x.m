function [vp_car, origin, extracted_3D_points] = extract_info_from_x(all_annos,...
    annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    mounting_height = all_annos.(fieldname).mounting_height;
    
    %Extracting process
    w_unit = abs(x0(1:x0_order.w_unit)); %Add absolute separation

%     if w_unit < 0
%         w_unit
%     end
%     x0_order
%     'w_unit'
%     w_unit
    assert(w_unit > 0, 'W_unit always larger than 0')
 
    %Extracting vp_car from fitted line
    azi_ele = [x0(x0_order.w_unit+1:x0_order.vp_car)];
    azi = azi_ele(1);
%     ele = azi_ele(2);
    ele = 0;
    vp_car = vp_car_from_azi_ele(azi,ele,R_horizon);


    origin = [x0(x0_order.vp_car+1:x0_order.origin-1) mounting_height x0(x0_order.origin:x0_order.origin)]';
    num_tires = x0(x0_order.origin+1:x0_order.num_tires);
    
    %Extracting 3D points
    extracted_3D_points = zeros(4,all_annos.(fieldname).num_points);
    start_pos = 1;
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        if id_to_tire(all_annos.(fieldname).tire_ids(i)) ~= id_to_tire(all_annos.(fieldname).car_origin)
            if (all_annos.(fieldname).car_origin + all_annos.(fieldname).tire_ids(i)) ~=5 %Not on same side
                if all_annos.(fieldname).car_origin == 2 || all_annos.(fieldname).car_origin == 3
                    extracted_3D_points(:,i) = [num_tires(start_pos) 0 -w_unit 1]';
                else
                    extracted_3D_points(:,i) = [num_tires(start_pos) 0 w_unit 1]';
                end
            else
                extracted_3D_points(:,i) = [num_tires(start_pos) 0 0 1]';
            end
            start_pos = start_pos+1;
        else
            extracted_3D_points(:,i) = [0 0 0 1]';
        end
    end

    
    %Old way with separation 
%     ex_points = x0(x0_order.num_tires+1:x0_order.ex_points);
%     if size(ex_points,2) > 0
%         start_pos = 1;
%         k = size(ex_points,2)/3;
%         for i=all_annos.(fieldname).order.tire+1:2:all_annos.(fieldname).order.ex_3D
%             if all_annos.(fieldname).car_origin == 2 || all_annos.(fieldname).car_origin == 3
%                  extracted_3D_points(:,i) =   [ex_points(start_pos) ...
%                      ex_points(start_pos+k) -w_unit/2-ex_points(start_pos+2*k)/2 1]';
%                  extracted_3D_points(:,i+1) =   [ex_points(start_pos) ...
%                      ex_points(start_pos+k) -w_unit/2+ex_points(start_pos+2*k)/2 1];
%             else
%                  extracted_3D_points(:,i) =   [ex_points(start_pos) ...
%                      ex_points(start_pos+k) w_unit/2-ex_points(start_pos+2*k)/2 1]';
%                  extracted_3D_points(:,i+1) =   [ex_points(start_pos) ...
%                      ex_points(start_pos+k) w_unit/2+ex_points(start_pos+2*k)/2 1];
%             end
%             start_pos = start_pos+1;
%         end
%     end

    %New way with no speration
    ex_points = x0(x0_order.num_tires+1:x0_order.ex_points);
    if all_annos.(fieldname).tire_both_sides
        if size(ex_points,2) > 0
            start_pos = 1;
            k = size(ex_points,2)/3;
            for i=all_annos.(fieldname).order.tire+1:2:all_annos.(fieldname).order.ex_3D
                if all_annos.(fieldname).car_origin == 2 || all_annos.(fieldname).car_origin == 3
                     extracted_3D_points(:,i) =   [ex_points(start_pos) ...
                         ex_points(start_pos+k) -w_unit/2-abs(ex_points(start_pos+2*k)/2) 1]';
                     extracted_3D_points(:,i+1) =   [ex_points(start_pos) ...
                         ex_points(start_pos+k) -w_unit/2+abs(ex_points(start_pos+2*k)/2) 1];
                else
                     extracted_3D_points(:,i) =   [ex_points(start_pos) ...
                         ex_points(start_pos+k) w_unit/2-abs(ex_points(start_pos+2*k)/2) 1]';
                     extracted_3D_points(:,i+1) =   [ex_points(start_pos) ...
                         ex_points(start_pos+k) w_unit/2+abs(ex_points(start_pos+2*k)/2) 1];
                end
                start_pos = start_pos+1;
            end
        end
    else
        if size(ex_points,2) > 0
            start_pos = 1;
            k = size(ex_points,2)/2;
            for i=all_annos.(fieldname).order.tire+1:2:all_annos.(fieldname).order.ex_3D
                if all_annos.(fieldname).car_origin == 2 || all_annos.(fieldname).car_origin == 3
                     extracted_3D_points(:,i) =   [ex_points(start_pos) ex_points(start_pos+k) -w_unit 1]';
                     extracted_3D_points(:,i+1) =   [ex_points(start_pos) ex_points(start_pos+k) 0 1]';
                else
                     extracted_3D_points(:,i) =   [ex_points(start_pos) ex_points(start_pos+k) 0 1]';
                     extracted_3D_points(:,i+1) =   [ex_points(start_pos) ex_points(start_pos+k) w_unit 1]';
                end
                start_pos = start_pos+1;
            end
        end
    end

%     extracted_3D_points(:,4:5)
%     all_3D_points_car_coor(:,4:5)
%     norm(extracted_3D_points(:,4:5)-all_3D_points_car_coor(:,4:5))

%     center_points = x0(x0_order.ex_points+1:x0_order.center_points);
%     if size(center_points,2) > 0
%         start_pos = 1;
%         for i=all_annos.(fieldname).order.ex_3D+1:1:all_annos.(fieldname).order.center_3D
%             if all_annos.(fieldname).car_origin == 2 || all_annos.(fieldname).car_origin == 3
%                   extracted_3D_points(:,i) =   [center_points(start_pos) ...
%                 center_points(start_pos+size(center_points,2)/2) -w_unit/2 1]';
%             else
%                   extracted_3D_points(:,i) =   [center_points(start_pos) ...
%                 center_points(start_pos+size(center_points,2)/2) w_unit/2 1]';
%             end
%             start_pos = start_pos+1;
%         end
%     end
%     non_ex_points = x0(x0_order.center_points+1:end);

    non_ex_points = x0(x0_order.ex_points+1:end);

    if size(non_ex_points,2) > 0
        start_pos =  1;
        k = size(non_ex_points,2)/3;
        for i=all_annos.(fieldname).order.ex_3D+1:2:all_annos.(fieldname).order.nonex_3D
%             'useful info'
%             [non_ex_points(start_pos) non_ex_points(start_pos+k) abs(non_ex_points(start_pos+2*k)/2)]

             if all_annos.(fieldname).car_origin == 2 || all_annos.(fieldname).car_origin == 3
                 extracted_3D_points(:,i) =   [non_ex_points(start_pos) ...
                     non_ex_points(start_pos+k) -w_unit/2-abs(non_ex_points(start_pos+2*k)/2) 1]';
                 extracted_3D_points(:,i+1) =  [non_ex_points(start_pos) ...
                     non_ex_points(start_pos+k) -w_unit/2+abs(non_ex_points(start_pos+2*k)/2) 1]';
             else
                 extracted_3D_points(:,i) =   [non_ex_points(start_pos) ...
                     non_ex_points(start_pos+k) w_unit/2-abs(non_ex_points(start_pos+2*k)/2) 1]';
                 extracted_3D_points(:,i+1) =  [non_ex_points(start_pos) ...
                     non_ex_points(start_pos+k) w_unit/2+abs(non_ex_points(start_pos+2*k)/2) 1]';
             end

             start_pos = start_pos+1;
        end
    end
 
% %     extracted_3D_points(:,9:end)
% %     all_3D_points_car_coor(:,9:end)
% %     norm(extracted_3D_points(:,9:end)-all_3D_points_car_coor(:,9:end))
% % 
% %     norm(extracted_3D_points-all_3D_points_car_coor)
end