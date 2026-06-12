function [c, ceq] = all_unit_tests_opt(horizon_line,annotated_car_id,all_annos,id_to_tire, c,ceq)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    mounting_height = all_annos.(fieldname).mounting_height;
    
    %Enforcing sym unit equal 0
%     vp_car = all_annos.(fieldname).vp_car_opt;
%     ceq = [ceq dot(vp_car,horizon_line)];

    %Enforcing on the ground
%     for i = 1:size(all_annos.(fieldname).tire_ids,2)
%         atire = all_annos.(fieldname).tires_3D_opt.(id_to_tire(all_annos.(fieldname).tire_ids(i)));
%         ceq = [ceq abs(atire(2) - mounting_height)];
%     end

    %Enforcing all sym pairs to be above the ground and less than certain
    %threshold
%     for k = 1: size(all_annos.(fieldname).ex_3D_opt,2)/2
%         pd = all_annos.(fieldname).ex_3D_opt(:,2*k-1); %point driver
%         c = [c pd(2)-mounting_height];
%     end
%     for k = 1: size(all_annos.(fieldname).nonex_3D_opt,2)/2
%         pd = all_annos.(fieldname).nonex_3D_opt(:,2*k-1); %point driver
%         c = [c pd(2)-mounting_height];
%     end
%     if isfield(all_annos.(fieldname),'center_3D')
%         for k = 1: size(all_annos.(fieldname).center_3D_opt,2)
%             pd = all_annos.(fieldname).center_3D_opt(:,k); %center point
%             c = [c pd(2)-mounting_height];
%         end
%     end

    %Enforcing 18 19 for both ex and non ex
%     for k = 1: size(all_annos.(fieldname).ex_3D_opt,2)/2
%         pd = all_annos.(fieldname).ex_3D_opt(:,2*k-1); %point driver
%         pp = all_annos.(fieldname).ex_3D_opt(:,2*k); %point passenger
%         if all_annos.(fieldname).visible == "D"
%             vec = pp-pd;
%         else
%             vec = pd-pp;
%         end
%         ceq = [ceq abs(dot(all_annos.(fieldname).sym_normal_unit_opt, vec)- ...
%             norm(vec))];
%         ceq = [ceq abs(dot(all_annos.(fieldname).sym_normal_unit_opt, pp+pd)+2*...
%             all_annos.(fieldname).sym_plane_norm(4))];
%     end
%     for k = 1: size(all_annos.(fieldname).nonex_3D_opt,2)/2
%         pd = all_annos.(fieldname).nonex_3D_opt(:,2*k-1); %point driver
%         pp = all_annos.(fieldname).nonex_3D_opt(:,2*k); %point passenger
%         if all_annos.(fieldname).visible == "D"
%             vec = pp-pd;
%         else
%             vec = pd-pp; 
%         end
%         ceq = [ceq abs(dot(all_annos.(fieldname).sym_normal_unit_opt, vec)- ...
%             norm(vec))];
%         ceq = [ceq abs(dot(all_annos.(fieldname).sym_normal_unit_opt, pp+pd)+2*...
%             all_annos.(fieldname).sym_plane_norm(4))];
%     end
end