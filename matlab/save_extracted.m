
function all_annos = save_extracted(annotated_car_id,all_annos,id_to_tire, ...
    cMw, car_model_param,extracted_3D_points_car_coor,mirror_one_side, origin,image,k_matrix,R, estimated_dist_to_move)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    estimated_3d_points = inv(cMw)*extracted_3D_points_car_coor;
    estimated_3d_points = estimated_3d_points(1:3,:);
    all_3D_points = estimated_3d_points;
    assert(size(all_3D_points,2) == all_annos.(fieldname).num_points);

    %Initially
    all_annos = save_3D_points_right_position(all_annos,fieldname,id_to_tire,all_3D_points,origin);


    %SOLUTION 1 Use side view mirrors to veto extremal
    % Gotta resave all the ones that have been saved above
    adjusted_ex = false;

    if true %Adjust all
%     if false  %Whether adjusted or not
        dist_using_ex_before = norm(all_annos.(fieldname).ex_3D(:,1)-all_annos.(fieldname).ex_3D(:,2));
        
        %Get via mirror
        dist_using_mirror =  norm(all_annos.(fieldname).nonex_3D(:,end-1)-all_annos.(fieldname).nonex_3D(:,end))-2*mirror_one_side;
    
        %Get Wheelbase-Gaussian
        scenario = 'OW__WB';
        dist_using_wb = gaussian_pred(car_model_param, scenario, all_annos.(fieldname).wheelbase);

        %Get via estimated Bias
        dist_using_bias = dist_using_ex_before + 2*estimated_dist_to_move;

        %Get Trackwidth
%         scenario = 'OW__TW';
%         tw_est = dist_using_ex_before - 0.225; %Subtrack one tire width to get tw
%         dist_using_tw = gaussian_pred(car_model_param, scenario, tw_est);

        if all_annos.(fieldname).tire_both_sides
            if  all_annos.(fieldname).has_mirror 
                dist_before = dist_using_mirror;
%             if all_annos.(fieldname).wheelbase > 0
%                 dist_before = dist_using_wb;
            else
                dist_before = dist_using_ex_before; %No change
            end
%               dist_before = dist_using_tw;
        elseif all_annos.(fieldname).has_mirror
            %If has wheelbase use Wheelbase-Gaussian, elseif has mirror use 
            if all_annos.(fieldname).wheelbase > 0
                dist_before = dist_using_wb;
            else
                dist_before = dist_using_mirror; %No change
            end
        elseif all_annos.(fieldname).wheelbase > 0
            dist_before = dist_using_wb;
        elseif all_annos.(fieldname).wheelbase == -1
            dist_before = dist_using_bias;
        end

        dist_to_move = (dist_before-dist_using_ex_before)/2;
        if dist_to_move > 0 %Only expansion not smaller
            adjusted_ex = true;
            ds = -dot(all_annos.(fieldname).sym_normal_unit,mean(all_annos.(fieldname).ex_3D,2));

            %INITIALLY
            %Extend the car origin then save
%             'origin'
            all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin)) = ...
                extend_a_point(all_annos,fieldname,origin,dist_to_move,ds);

            %Extend the tires and then save
            start_pos = 0;
            for i = 1:size(all_annos.(fieldname).tire_ids,2)
%                 'tire'
                %Extending
                a_point = all_3D_points(:,start_pos+i);
                a_point_corrected  = extend_a_point(all_annos,fieldname,a_point,dist_to_move,ds);

                %Saving
                all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i))) = a_point_corrected;
                all_3D_points(:,start_pos+i) = a_point_corrected;
            end

            %Saving wheelbase agai
            if ismember(1,all_annos.(fieldname).tire_ids) && ismember(4,all_annos.(fieldname).tire_ids)
               all_annos.(fieldname).wheelbase = norm(all_annos.(fieldname).tires_3D.(id_to_tire(1))...
                   - all_annos.(fieldname).tires_3D.(id_to_tire(4)));
            elseif ismember(2,all_annos.(fieldname).tire_ids) && ismember(3,all_annos.(fieldname).tire_ids)
               all_annos.(fieldname).wheelbase = norm(all_annos.(fieldname).tires_3D.(id_to_tire(2))...
                   - all_annos.(fieldname).tires_3D.(id_to_tire(3)));
            end

            %Extend the extremal
            start_pos = start_pos+size(all_annos.(fieldname).tire_ids,2);
            for i = 1: size(all_annos.(fieldname).ex_3D,2)
%                 'extremal'
                %Extending
                a_point = all_3D_points(:,start_pos+i);
                a_point_corrected  = extend_a_point(all_annos,fieldname,a_point,dist_to_move,ds);

                %Saving
                all_annos.(fieldname).ex_3D(:,i) = a_point_corrected;
                all_3D_points(:,start_pos+i) = a_point_corrected;

            end
            %Save all_3D_points again
            all_annos.(fieldname).all_3D_points = all_3D_points;
            origin_sym_coor = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin));
            %Get the new transformation
            cMw = get_cMw_using_world_points(k_matrix,R,all_annos.(fieldname).vp_car,...
                origin_sym_coor,all_annos,annotated_car_id, all_annos.(fieldname).all_3D_points,image);
            extracted_3D_points_car_coor = cMw*[all_3D_points;ones(1,size(all_3D_points,2))];
      
            %Assert the width is now extended - %Not necessarily true
            %anymore
%             if all_annos.(fieldname).has_mirror
%                 dist_using_ex_after = norm(all_annos.(fieldname).ex_3D(:,1)-all_annos.(fieldname).ex_3D(:,2));
%                 dist_using_mirrors_after =  norm(all_annos.(fieldname).nonex_3D(:,end-1)-all_annos.(fieldname).nonex_3D(:,end))-2*mirror_one_side;
%                 assert(abs(dist_using_ex_after-dist_using_mirrors_after)<1e-6, "Gotta be equal plane now");
%             end
        end
    end

    if adjusted_ex
        all_annos.(fieldname).dist_to_move = dist_to_move;
    else
        all_annos.(fieldname).dist_to_move = 0;
    end

    % Localte all 3D points on each side
    if all_annos.(fieldname).tire_both_sides
        visside = [];
        nvisside = [];
        for i = 1:size(all_annos.(fieldname).tire_ids,2)
            if all_annos.(fieldname).visible == "D"
                if all_annos.(fieldname).tire_ids(i) == 1 || all_annos.(fieldname).tire_ids(i) == 4
                    visside = [visside all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)))];
                else
                    nvisside = [nvisside all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)))];
                end
            else
                if all_annos.(fieldname).tire_ids(i) == 1 || all_annos.(fieldname).tire_ids(i) == 4
                    nvisside = [nvisside all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)))];
                else
                    visside = [visside all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)))];
                end
            end
        end
        sym_point = mean([mean(visside,2) mean(nvisside,2)],2); %Mean of means
        ds = -dot(all_annos.(fieldname).sym_normal_unit,sym_point);
        sym_plane_norm = [all_annos.(fieldname).sym_normal_unit ;ds];
        all_annos.(fieldname).ds = ds;
        all_annos.(fieldname).sym_plane_norm = sym_plane_norm;
        
        dv = -dot(all_annos.(fieldname).sym_normal_unit,mean(visside,2));
        vis_plane_norm = [all_annos.(fieldname).sym_normal_unit ;dv];
        all_annos.(fieldname).dv = dv;
        all_annos.(fieldname).vis_plane_norm = vis_plane_norm;
        
        dnv = -dot(all_annos.(fieldname).sym_normal_unit,mean(nvisside,2));
        nvis_plane_norm = [all_annos.(fieldname).sym_normal_unit ;dnv];
        all_annos.(fieldname).dnv = dnv;
        all_annos.(fieldname).nvis_plane_norm = nvis_plane_norm;
        all_annos.(fieldname).w_unit = abs(dot(all_annos.(fieldname).sym_normal_unit,mean(nvisside,2))+ds)*2;
        
    end

    if ~all_annos.(fieldname).tire_both_sides
        if all_annos.(fieldname).visible == "D"
            vis_ex_3D = all_annos.(fieldname).ex_3D(:,1);
            nvis_ex_3D = all_annos.(fieldname).ex_3D(:,2);
        else
            nvis_ex_3D = all_annos.(fieldname).ex_3D(:,1);
            vis_ex_3D = all_annos.(fieldname).ex_3D(:,2);
        end
        all_annos.(fieldname).w_unit = norm(vis_ex_3D-nvis_ex_3D);
        

        %Symmetry plane
        ds = -dot(all_annos.(fieldname).sym_normal_unit,mean(all_annos.(fieldname).ex_3D,2));
        all_annos.(fieldname).ds = ds;
        sym_plane_norm = [all_annos.(fieldname).sym_normal_unit ;ds];
        all_annos.(fieldname).sym_plane_norm = sym_plane_norm;
     
        dv = -dot(all_annos.(fieldname).sym_normal_unit,vis_ex_3D);
        all_annos.(fieldname).dv = dv;
        vis_plane_norm = [all_annos.(fieldname).sym_normal_unit ;dv];
        all_annos.(fieldname).vis_plane_norm = vis_plane_norm;

        dnv = -dot(all_annos.(fieldname).sym_normal_unit,nvis_ex_3D);
        all_annos.(fieldname).dnv = dnv;
        nvis_plane_norm = [all_annos.(fieldname).sym_normal_unit ;dnv];
        all_annos.(fieldname).nvis_plane_norm = nvis_plane_norm;
    end

    %Remove not needed fields like separations
%     fields_to_clear = ["nonex_sep","ex_sep"];
%     for i = 1:size(fields_to_clear,2)
%         if isfield(all_annos.(fieldname),fields_to_clear(i))
%             all_annos.(fieldname) = rmfield(all_annos.(fieldname),fields_to_clear(i));
%         end
%     end

    

    %Save bbox points only
    predicted_corners_car_coor = get_bbox_from_points(extracted_3D_points_car_coor,true, all_annos.(fieldname).has_mirror, mirror_one_side,all_annos.(fieldname).w_unit);
    predicted_corners = inv(cMw)*predicted_corners_car_coor;
    all_annos.(fieldname).pred_bbox_points_only = predicted_corners(1:3,:);
    %Combine with gaussian model
    [get_length_gaussian, corners] = get_bbox_from_gaussian_model(extracted_3D_points_car_coor,all_annos, fieldname,car_model_param,id_to_tire, mirror_one_side); 
    pred_bbox = inv(cMw)*corners;
    all_annos.(fieldname).pred_bbox = pred_bbox(1:3,:);
    all_annos.(fieldname).pred_bbox_bev = [all_annos.(fieldname).pred_bbox(1,1:4); all_annos.(fieldname).pred_bbox(3,1:4)];
    all_annos.(fieldname).get_length_gaussian = get_length_gaussian; 
end
