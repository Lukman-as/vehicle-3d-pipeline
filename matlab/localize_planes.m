function [bad_annotation_sym, all_annos] = localize_planes(k_matrix,R,annotated_car_id,all_annos,id_to_tire,image)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    fields_to_clear = ["tires_3D", "ds","sym_plane_norm","dv","vis_plane_norm","dnv","nvis_plane_norm","w_unit"];
    mounting_height = all_annos.(fieldname).mounting_height;
    for i = 1:size(fields_to_clear,2)
        if isfield(all_annos.(fieldname),fields_to_clear(i))
            all_annos.(fieldname) = rmfield(all_annos.(fieldname),fields_to_clear(i));
        end
    end
    
    %Section 2.1
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        tground = all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
        temp =  inv(k_matrix*R)*tground;
        lambda = mounting_height/temp(2);
        tground_3D = [temp(1)*lambda mounting_height temp(3) * lambda]';
        all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i))) = tground_3D;
    end
    
    % Localte all 3D points on each side
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
    
    
    %Get sympoint CASE 1
    if all_annos.(fieldname).tire_both_sides
        bad_annotation_sym = false;
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
        
        assert(abs(dot(all_annos.(fieldname).sym_normal_unit,mean(nvisside,2))+ds)== ...
        -abs(dot(all_annos.(fieldname).sym_normal_unit,mean(visside,2))+ds)<1e-12,...
        "Distance from sym to both planes gotta equal")
        all_annos.(fieldname).w_unit = abs(dot(all_annos.(fieldname).sym_normal_unit,mean(nvisside,2))+ds)*2;
    end
    
    
    %Section 2.2
    if ~all_annos.(fieldname).tire_both_sides
        dv = -dot(all_annos.(fieldname).sym_normal_unit,mean(visside,2));
        vis_plane_norm = [all_annos.(fieldname).sym_normal_unit ;dv];
        all_annos.(fieldname).dv = dv;
        all_annos.(fieldname).vis_plane_norm = vis_plane_norm;
        all_w_unit = zeros(1,size(all_annos.(fieldname).ex,2)/2);
        all_ex_3D = zeros(3,size(all_annos.(fieldname).ex,2));
        all_compares = [];
        for k = 1:size(all_annos.(fieldname).ex,2)/2
            if all_annos.(fieldname).visible == "D"
                vis_ex = all_annos.(fieldname).ex(:,2*k-1);
                nvis_ex = all_annos.(fieldname).ex(:,2*k);
            else
                vis_ex = all_annos.(fieldname).ex(:,2*k);
                nvis_ex = all_annos.(fieldname).ex(:,2*k-1);
            end
            plane = all_annos.(fieldname).vis_plane_norm;
            [w_unit, vis_ex_3D, nvis_ex_3D] = ...
                get_extremal_3D(k_matrix, R, vis_ex, nvis_ex, plane);
            all_w_unit(k) = w_unit;

            if dot(vis_ex_3D, all_annos.(fieldname).sym_normal_unit) < ...
                    dot(nvis_ex_3D, all_annos.(fieldname).sym_normal_unit)
                all_compares = [all_compares 1];
            else
                all_compares = [all_compares 0];
            end
            %Saving the ex_3D
            if all_annos.(fieldname).visible == "D"
                all_ex_3D(:,2*k-1:2*k) = [vis_ex_3D nvis_ex_3D];
            else
                all_ex_3D(:,2*k-1:2*k) = [nvis_ex_3D vis_ex_3D];
            end
        end
        all_annos.(fieldname).w_unit = mean(all_w_unit);

        %Finding the distance to the left and right and sym
%         dnv = -dot(all_annos.(fieldname).sym_normal_unit, nvis_ex_3D);
        if range(all_compares) == 0
            bad_annotation_sym = false;
            assert(range(all_compares) == 0, 'All have to be same side')
            if all_compares(1) == 1
                dnv = all_annos.(fieldname).dv - all_annos.(fieldname).w_unit;
            else
                dnv = all_annos.(fieldname).dv + all_annos.(fieldname).w_unit;
            end
            nvis_plane_norm = [all_annos.(fieldname).sym_normal_unit; dnv];
            all_annos.(fieldname).dnv = dnv;
            all_annos.(fieldname).nvis_plane_norm = nvis_plane_norm;
            
            %Finding the distance to the left and right and sym
    %         ds = all_annos.(fieldname).dv - w_unit/2;
            ds = 1/2*(dnv+all_annos.(fieldname).dv);
            sym_plane_norm = [all_annos.(fieldname).sym_normal_unit ;ds];
            all_annos.(fieldname).ds = ds;
            all_annos.(fieldname).sym_plane_norm = sym_plane_norm;
    
            assert(abs(all_annos.(fieldname).w_unit - ...
            (abs(all_annos.(fieldname).vis_plane_norm(4) - ...
            all_annos.(fieldname).nvis_plane_norm(4))))<1e-6, "Distance between plane = w_unit")
        else
            bad_annotation_sym = true;
        end
    end
        

    %Retifying every tires
    if ~bad_annotation_sym
        for i = 1:size(all_annos.(fieldname).tire_ids,2)
            pre = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
            %Different d_plane for that tire
            if all_annos.(fieldname).visible == "D"
                if all_annos.(fieldname).tire_ids(i) == 1 || all_annos.(fieldname).tire_ids(i) == 4
                    d_plane = all_annos.(fieldname).dv;
                else
                    d_plane = all_annos.(fieldname).dnv;
                end
            else
                if all_annos.(fieldname).tire_ids(i) == 1 || all_annos.(fieldname).tire_ids(i) == 4
                    d_plane = all_annos.(fieldname).dnv;
                else
                    d_plane = all_annos.(fieldname).dv;
                end
            end
            %Finding the projection onto this plane
            k = (-d_plane-dot(pre,all_annos.(fieldname).sym_normal_unit))/dot(all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).sym_normal_unit);
            rec = pre+k*all_annos.(fieldname).sym_normal_unit;
            assert(abs(dot(all_annos.(fieldname).sym_normal_unit,rec)+d_plane)<1e-6)
            all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).tire_ids(i))) = rec;
        end
    end
    
    %If there are more than one extremal then have to correct for extremal
    if ~all_annos.(fieldname).tire_both_sides && ~bad_annotation_sym
        all_annos.(fieldname).ex_3D = zeros(3,size(all_annos.(fieldname).ex,2));
        for k = 1:size(all_annos.(fieldname).ex,2)/2
             %Saving the ex_3D
            if all_annos.(fieldname).visible == "D"
                pre = all_ex_3D(:,2*k-1);
                d_plane = all_annos.(fieldname).dv;
                j = (-d_plane-dot(pre,all_annos.(fieldname).sym_normal_unit))/dot(all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).sym_normal_unit);
                rec = pre+j*all_annos.(fieldname).sym_normal_unit;
                assert(abs(dot(all_annos.(fieldname).sym_normal_unit,rec)+d_plane)<1e-6);
                all_annos.(fieldname).ex_3D(:,2*k-1) = rec;
    
                pre = all_ex_3D(:,2*k);
                d_plane = all_annos.(fieldname).dnv;
                j = (-d_plane-dot(pre,all_annos.(fieldname).sym_normal_unit))/dot(all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).sym_normal_unit);
                rec = pre+j*all_annos.(fieldname).sym_normal_unit;
                assert(abs(dot(all_annos.(fieldname).sym_normal_unit,rec)+d_plane)<1e-6)
                all_annos.(fieldname).ex_3D(:,2*k) = rec;
                assert(abs(dot(all_annos.(fieldname).ex_3D(:,2*k-1),all_annos.(fieldname).vis_plane_norm(1:3))+all_annos.(fieldname).dv)<1e-5);
                assert(abs(dot(all_annos.(fieldname).ex_3D(:,2*k),all_annos.(fieldname).nvis_plane_norm(1:3))+all_annos.(fieldname).dnv)<1e-5);
            else
                pre = all_ex_3D(:,2*k-1);
                d_plane = all_annos.(fieldname).dnv;
                j = (-d_plane-dot(pre,all_annos.(fieldname).sym_normal_unit))/dot(all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).sym_normal_unit);
                rec = pre+j*all_annos.(fieldname).sym_normal_unit;
                assert(abs(dot(all_annos.(fieldname).sym_normal_unit,rec)+d_plane)<1e-6);
                all_annos.(fieldname).ex_3D(:,2*k-1) = rec;
    
                pre = all_ex_3D(:,2*k);
                d_plane = all_annos.(fieldname).dv;
                j = (-d_plane-dot(pre,all_annos.(fieldname).sym_normal_unit))/dot(all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).sym_normal_unit);
                rec = pre+j*all_annos.(fieldname).sym_normal_unit;
                assert(abs(dot(all_annos.(fieldname).sym_normal_unit,rec)+d_plane)<1e-6)
                all_annos.(fieldname).ex_3D(:,2*k) = rec;
                assert(abs(dot(all_annos.(fieldname).ex_3D(:,2*k),all_annos.(fieldname).vis_plane_norm(1:3))+all_annos.(fieldname).dv)<1e-5);
                assert(abs(dot(all_annos.(fieldname).ex_3D(:,2*k-1),all_annos.(fieldname).nvis_plane_norm(1:3))+all_annos.(fieldname).dnv)<1e-5);
            end
        end
    end
end