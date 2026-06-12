function [valid_result, reason] = all_unit_tests(annotated_car_id,all_annos,id_to_tire,threshold, horizon_line,after_nonlin)
    valid_result = true;
    reason = '';
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    mounting_height = all_annos.(fieldname).mounting_height;

    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        atire = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
        assert(abs(atire(2) - mounting_height) < threshold, 'Tire contact must be on ground');
    end
    
    assert(abs(sum(all_annos.(fieldname).vis_plane_norm(1:3) == ...
        all_annos.(fieldname).sym_plane_norm(1:3)) - 3) < threshold,...
        'Visible plane has to be parallel to sym plane');
    assert(abs(sum(all_annos.(fieldname).nvis_plane_norm(1:3) == ...
        all_annos.(fieldname).sym_plane_norm(1:3)) - 3) < threshold,...
            'Not visible has to be parallel to sym plane');
    
    assert(abs((all_annos.(fieldname).vis_plane_norm(4) + all_annos.(fieldname).nvis_plane_norm(4))/2 - ...
        all_annos.(fieldname).sym_plane_norm(4)) < threshold, ...
        'Extremal planes has to be equally displaced from sym plane');
    
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        atire = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
        tire_id = all_annos.(fieldname).tire_ids(i);
        if ((tire_id == 1 || tire_id == 4) && (all_annos.(fieldname).visible == "D")) || ...
               ((tire_id == 2 || tire_id == 3) && (all_annos.(fieldname).visible == "P"))
            assert(abs(dot(atire, all_annos.(fieldname).vis_plane_norm(1:3))+ ...
                all_annos.(fieldname).vis_plane_norm(4))...
                     < threshold,'visible tire point on visible plane');
        else
            assert(abs(dot(atire, all_annos.(fieldname).nvis_plane_norm(1:3))+ ...
                all_annos.(fieldname).nvis_plane_norm(4))...
                     < threshold,'nvisible tire point on nvisible plane');
        end
    end
    
    %Check order of tire contact points ordering front and back
    if after_nonlin
        if ismember(1,all_annos.(fieldname).tire_ids) && ismember(4,all_annos.(fieldname).tire_ids)
            vec1 = all_annos.(fieldname).pred_bbox(:,1) - all_annos.(fieldname).pred_bbox(:,4);
            vec1 = vec1/norm(vec1);
            vec2 = all_annos.(fieldname).tires_3D.(id_to_tire(1)) - all_annos.(fieldname).tires_3D.(id_to_tire(4));
            vec2 = vec2/norm(vec2);
            if abs(acos(dot(vec1,vec2))) > threshold
                valid_result = false;
                reason = append(reason,'_order_DF_DR');
            end
%             assert(abs(acos(dot(vec1,vec2))) < threshold, 'The order of front and back tire has to be correct for DF and DR')
        end
    
        if ismember(2,all_annos.(fieldname).tire_ids) && ismember(3,all_annos.(fieldname).tire_ids)
            vec1 = all_annos.(fieldname).pred_bbox(:,2) - all_annos.(fieldname).pred_bbox(:,3);
            vec1 = vec1/norm(vec1);
            vec2 = all_annos.(fieldname).tires_3D.(id_to_tire(2)) - all_annos.(fieldname).tires_3D.(id_to_tire(3));
            vec2 = vec2/norm(vec2);
            if abs(acos(dot(vec1,vec2))) > threshold
                valid_result = false;
                reason = append(reason,'_order_PF_PR');
            end
%             assert(abs(acos(dot(vec1,vec2))) < threshold, 'The order of front and back tire has to be correct for PF and PR')
        end
    end

    %If tire not both sides then extremal points have to be on extremal plane
    if ~all_annos.(fieldname).tire_both_sides
        if all_annos.(fieldname).visible == "D"
            point = all_annos.(fieldname).ex_3D(:,1);
            assert(abs(dot(point, all_annos.(fieldname).vis_plane_norm(1:3))+ ...
                all_annos.(fieldname).vis_plane_norm(4))...
                     < threshold,'visible extremal on visible plane');
            point = all_annos.(fieldname).ex_3D(:,2);
            assert(abs(dot(point, all_annos.(fieldname).nvis_plane_norm(1:3))+ ...
                all_annos.(fieldname).nvis_plane_norm(4))...
                     < threshold,'nvisible extremal on nvisible plane');
        else
            point = all_annos.(fieldname).ex_3D(:,2);
            assert(abs(dot(point, all_annos.(fieldname).vis_plane_norm(1:3))+ ...
                all_annos.(fieldname).vis_plane_norm(4))...
                     < threshold,'visible extremal on visible plane');
            point = all_annos.(fieldname).ex_3D(:,1);
            assert(abs(dot(point, all_annos.(fieldname).nvis_plane_norm(1:3))+ ...
                all_annos.(fieldname).nvis_plane_norm(4))...
                     < threshold,'nvisible extremal on nvisible plane');
        end
    end
    
    %Check both case because you do not enforce going from Driver to
    %Passenger anymore
    if size(all_annos.(fieldname).nonex_3D,2) > 0
        for k = 1: size(all_annos.(fieldname).nonex_3D,2)/2
            pd = all_annos.(fieldname).nonex_3D(:,2*k-1); %point driver
            pp = all_annos.(fieldname).nonex_3D(:,2*k); %point passenger
            vec = pd-pp;
            check1 = abs(dot(all_annos.(fieldname).sym_normal_unit, vec)- norm(vec));
            check2 = abs(dot(all_annos.(fieldname).sym_normal_unit, -vec)- norm(vec));
            assert(check1 <threshold || check2<threshold , 'Non extremal points on sym line');
            assert(abs(dot(all_annos.(fieldname).sym_normal_unit, pp+pd)+2*...
                all_annos.(fieldname).sym_plane_norm(4))<threshold,...
                'symmetry point pairs has to be equally displaced from sym plane');
        end
    end

    for k = 1: size(all_annos.(fieldname).ex_3D,2)/2
        pd = all_annos.(fieldname).ex_3D(:,2*k-1); %point driver
        pp = all_annos.(fieldname).ex_3D(:,2*k); %point passenger
        vec = pd-pp;
        check1 = abs(dot(all_annos.(fieldname).sym_normal_unit, vec)- norm(vec));
        check2 = abs(dot(all_annos.(fieldname).sym_normal_unit, -vec)- norm(vec));
        assert(check1 <threshold || check2<threshold , 'Extremal points on sym line');
        assert(abs(dot(all_annos.(fieldname).sym_normal_unit, pp+pd)+2*...
            all_annos.(fieldname).sym_plane_norm(4))<threshold,...
            'symmetry point pairs has to be equally displaced from sym plane');
    end

    % Assert ground bounding box corners on the ground or mounting height
    % after nonlin
    if after_nonlin
        if abs(max(all_annos.(fieldname).pred_bbox(2,:))-all_annos.(fieldname).mounting_height) >= threshold
            all_annos.(fieldname).mounting_height
            max(all_annos.(fieldname).pred_bbox(2,:))
            all_annos.(fieldname).pred_bbox
        end
%         assert(abs(max(all_annos.(fieldname).pred_bbox(2,:))-all_annos.(fieldname).mounting_height) < threshold, ...
%             'All predicted 3D points have to be above the ground');
         if abs(max(all_annos.(fieldname).pred_bbox(2,:))-all_annos.(fieldname).mounting_height) >= threshold
            valid_result = false;
            reason = append(reason,'_below_ground');
         end
    end

    %Check vpcar to be on the horizonline
    assert(abs(dot(all_annos.(fieldname).vp_car ,horizon_line))<threshold, "vpcar has to be on the horizon line")
    assert(abs(all_annos.(fieldname).vp_car(3)-1)<threshold, "vpcar has to have z = 1")
    
    %Assert pred_width have to equal w_unit
    abs(all_annos.(fieldname).w_unit - ...
        (abs(all_annos.(fieldname).vis_plane_norm(4) - ...
        all_annos.(fieldname).nvis_plane_norm(4))))
    assert(abs(all_annos.(fieldname).w_unit - ...
        (abs(all_annos.(fieldname).vis_plane_norm(4) - ...
        all_annos.(fieldname).nvis_plane_norm(4))))<threshold, "Distance between plane = w_unit")
end