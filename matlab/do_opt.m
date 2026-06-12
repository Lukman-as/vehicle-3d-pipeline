function [bad_annotation, all_annos] = do_opt(annotated_car_id,all_annos,...
    id_to_tire,horizon_line,k_matrix,R,image, car_model_param,use_enforce,...
    mirror_one_side, estimated_dist_to_move)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    origin_sym_coor = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin));
    cMw = get_cMw_using_world_points(k_matrix,R,all_annos.(fieldname).vp_car, origin_sym_coor,all_annos,annotated_car_id, all_annos.(fieldname).all_3D_points,image);
    all_3D_points_car_coor = cMw*[all_annos.(fieldname).all_3D_points; ones(1,size(all_annos.(fieldname).all_3D_points,2))];

    %Get azi and ele
    horizon_line = horizon_line/norm(horizon_line);
    R_horizon = get_rotation_from_two_vecs(horizon_line, [0 0 1]');
    vp_car_horizon = R_horizon*all_annos.(fieldname).vp_car;
    vp_car_hor = vp_car_horizon/norm(vp_car_horizon);
    azi = atan(vp_car_hor(2)/vp_car_hor(1));

     
    %Processing tire
    all_tires_except_origin_car_coor = zeros(4,all_annos.(fieldname).order.tire-1);
    start_pos = 1;
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        if id_to_tire(all_annos.(fieldname).tire_ids(i)) ~= id_to_tire(all_annos.(fieldname).car_origin)
            all_tires_except_origin_car_coor(:,start_pos) = all_3D_points_car_coor(:,i);
            start_pos = start_pos+1;
        end
    end
    
    ex_points = all_3D_points_car_coor(:,all_annos.(fieldname).order.tire+1:all_annos.(fieldname).order.ex_3D);
%     center_points = all_3D_points_car_coor(:,all_annos.(fieldname).order.ex_3D+1:all_annos.(fieldname).order.center_3D);
%     non_ex_points = all_3D_points_car_coor(:,all_annos.(fieldname).order.center_3D+1:end);
    non_ex_points = all_3D_points_car_coor(:,all_annos.(fieldname).order.ex_3D+1:end);


     %Separate exsep way
    x0_order =  struct;
    num_points = 1;
    x0_order.('w_unit')=num_points;
    num_points = num_points+1;
    x0_order.('vp_car')=num_points;
    num_points = num_points+2;
    x0_order.('origin')=num_points;
    num_points = num_points+size(all_tires_except_origin_car_coor,2);
    x0_order.('num_tires')=num_points;
    if all_annos.(fieldname).tire_both_sides
        num_points = num_points+size(ex_points,2)/2+size(ex_points,2); %With the separation part
    else
        num_points = num_points+size(ex_points,2);
    end
    x0_order.('ex_points')=num_points;
%     num_points = num_points+size(center_points,2)*2;
%     x0_order.('center_points')=num_points;
    num_points = num_points+size(non_ex_points,2)/2+size(non_ex_points,2);
    x0_order.('non_ex_points')=num_points;    

   if all_annos.(fieldname).tire_both_sides
        x0=...
        [all_annos.(fieldname).w_unit,...
        azi,...%VP car
        origin_sym_coor(1), origin_sym_coor(3),... %Origin tire
        all_tires_except_origin_car_coor(1,:),...
        ex_points(1,1:2:end), ex_points(2,1:2:end), all_annos.(fieldname).ex_sep,...
        non_ex_points(1,1:2:end), non_ex_points(2,1:2:end), all_annos.(fieldname).nonex_sep,...
        ];
   else
        x0=...
        [all_annos.(fieldname).w_unit,...
        azi,...%VP car
        origin_sym_coor(1), origin_sym_coor(3),... %Origin tire
        all_tires_except_origin_car_coor(1,:),...
        ex_points(1,1:2:end), ex_points(2,1:2:end),...
        non_ex_points(1,1:2:end), non_ex_points(2,1:2:end), all_annos.(fieldname).nonex_sep,...
        ];
   end

%    if all_annos.(fieldname).tire_both_sides
%         x0=...
%         [all_annos.(fieldname).w_unit,...
%         azi,...%VP car
%         origin_sym_coor(1), origin_sym_coor(3),... %Origin tire
%         all_tires_except_origin_car_coor(1,:),...
%         ex_points(1,1:2:end), ex_points(2,1:2:end), all_annos.(fieldname).ex_sep,...
%         center_points(1,:),center_points(2,:),...     
%         non_ex_points(1,1:2:end), non_ex_points(2,1:2:end), all_annos.(fieldname).nonex_sep,...
%         ];
%    else
%         x0=...
%         [all_annos.(fieldname).w_unit,...
%         azi,...%VP car
%         origin_sym_coor(1), origin_sym_coor(3),... %Origin tire
%         all_tires_except_origin_car_coor(1,:),...
%         ex_points(1,1:2:end), ex_points(2,1:2:end),...
%         center_points(1,:),center_points(2,:),...     
%         non_ex_points(1,1:2:end), non_ex_points(2,1:2:end), all_annos.(fieldname).nonex_sep,...
%         ];
%    end

   assert(size(x0,2)==x0_order.non_ex_points);
   [vp_car, ~, extracted_3D_points_car_coor] = extract_info_from_x(...
    all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon);
%    'norm'
%    x0
%    'car id'
%    annotated_car_id
%    
%    norm(all_3D_points_car_coor-extracted_3D_points_car_coor)
%    all_3D_points_car_coor
%    extracted_3D_points_car_coor


   if norm(all_3D_points_car_coor-extracted_3D_points_car_coor) >1e-3
       norm(all_3D_points_car_coor-extracted_3D_points_car_coor)
       all_annos.(fieldname).order
       format short
       all_3D_points_car_coor;
       extracted_3D_points_car_coor
       all_annos.(fieldname).all_3D_points
   end
   bad_annotation = false;
   try
      assert(norm(all_3D_points_car_coor-extracted_3D_points_car_coor)<1e-3);
   catch
      bad_annotation = true;
   end
   
   %Return if bad annotation
   if bad_annotation
       return
   end
   assert(norm(vp_car-all_annos.(fieldname).vp_car)<3e-1);


   %LM optimization
    options = optimoptions("lsqnonlin","StepTolerance",1e-15,"MaxIterations",3e3,...
        "MaxFunctionEvaluations",5e4, 'Algorithm','levenberg-marquardt',"FunctionTolerance",1e-20,...
        'Display','off');
    use_bounds = true;
    assert(use_bounds);
    if use_bounds
        %Add bounds
        x0_lb = -Inf(size(x0));
        x0_ub = Inf(size(x0));
        %Setup
        if strcmp(use_enforce, 'bounds')
           enforce_all = true;
        elseif strcmp(use_enforce, 'nobounds')
           enforce_all = false;
        end
        if enforce_all
            enforcing_min_height = true;
            enforcing_max_height = true;
            enforcing_width = true;
            enforcing_length = true;
        else
            enforcing_min_height = false;
            enforcing_max_height = false;
            enforcing_width = false;
            enforcing_length = false;
        end

        %Dimension to enforce
        max_width = 2.93+0.6; %From the database + mirror
%         min_wwom =  1.56; %From the database
        max_height = 3.05; %From the database
        max_FH = 1.1; %From the database
        max_WB = 4.47; %From the database
        max_RH = 2.02; %From the database
    
        if all_annos.(fieldname).tire_both_sides
            num_ex = (x0_order.ex_points-x0_order.num_tires)/3;
        else
            num_ex = (x0_order.ex_points-x0_order.num_tires)/2;
        end
        num_non_ex = (x0_order.non_ex_points-x0_order.ex_points)/3;
    
        %Height dimension
        if enforcing_min_height
            %Lower points for ex points first but it has to be ub because ypoint down
            x0_ub(x0_order.num_tires+1+num_ex:x0_order.num_tires+num_ex*2) = 0; %Index 2 is the wy index
            %Bound for non ex
            x0_ub(x0_order.ex_points+1+num_non_ex:x0_order.ex_points+num_non_ex*2) = 0;
        end
        if enforcing_max_height
            %Bound for ex
            x0_lb(x0_order.num_tires+1+num_ex:x0_order.num_tires+num_ex*2) = -max_height; %Index 2 is the wy index
            %Bound for non ex
            x0_lb(x0_order.ex_points+1+num_non_ex:x0_order.ex_points+num_non_ex*2) = -max_height;
        end
      
        %Enforcing width
        if enforcing_width
            x0_ub(x0_order.w_unit) = max_width;
            x0_lb(x0_order.w_unit) = -max_width;
%             x0_lb(x0_order.w_unit) = min_wwom;  %Lowerbound for w_unit only

            if all_annos.(fieldname).tire_both_sides
                x0_ub(x0_order.num_tires+num_ex*2+1:x0_order.ex_points) = max_width; %All the ex separation
                x0_lb(x0_order.num_tires+num_ex*2+1:x0_order.ex_points) = -max_width; 
            end
            x0_ub(x0_order.ex_points+num_non_ex*2+1:end) = max_width; %All the non ex separation
            x0_lb(x0_order.ex_points+num_non_ex*2+1:end) = -max_width; %All the non ex separation
        end
        
        %Enforcing length dimension
        if enforcing_length
            if all_annos.(fieldname).car_origin == 1 || all_annos.(fieldname).car_origin == 2
                min_length = -max_FH;
                max_length = max_WB + max_RH;
                %Tirepoints enforce across length
                x0_lb(x0_order.origin+1:x0_order.num_tires) = min_length;
%                 x0_lb(x0_order.origin+1:x0_order.num_tires) = 0; %Because
%                 cannot cross to the front - REMOVING NO ENFORCE TIRES

                x0_ub(x0_order.origin+1:x0_order.num_tires) = max_length;
                %Ex points
                x0_lb(x0_order.num_tires+1:x0_order.num_tires+num_ex) = min_length;
                x0_ub(x0_order.num_tires+1:x0_order.num_tires+num_ex) = max_length;
                %Non ex points
                x0_lb(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = min_length;
                x0_ub(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = max_length;
            else
                min_length = -max_WB - max_FH;
                max_length = max_RH;
                x0_lb(x0_order.origin+1:x0_order.num_tires) = min_length;
                x0_ub(x0_order.origin+1:x0_order.num_tires) = max_length;
%                 x0_ub(x0_order.origin+1:x0_order.num_tires) = 0; %Because
%                 cannot cross to the back - REMOVING NO ENFORCE TIRES

                %Ex points
                x0_lb(x0_order.num_tires+1:x0_order.num_tires+num_ex) = min_length;
                x0_ub(x0_order.num_tires+1:x0_order.num_tires+num_ex) = max_length;
                %Non ex points
                x0_lb(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = min_length;
                x0_ub(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = max_length;
            end
        end


        try
          x = lsqnonlin(@(x0) coop_fun_lsq(k_matrix,R,all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon),x0,...
            x0_lb,x0_ub, options);
        catch
          bad_annotation = true;
        end
        %Return if bad annotation
        if bad_annotation
           return
        end
    else
        x = lsqnonlin(@(x0) coop_fun_lsq(k_matrix,R,all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon),x0,...
        [],[], options);
    end
    
    %% Convert back to world
%     %% Setting x = x0 for no optimization reprojection - To be removed
%     x = x0;
    [vp_car, origin, extracted_3D_points_car_coor] = extract_info_from_x(...
        all_annos,annotated_car_id,x0_order,id_to_tire, x,horizon_line,R_horizon);
    assert(abs(vp_car(3)-1)<1e-6);
    assert(abs(dot(vp_car,horizon_line))<1e6);

    debug = false;
    if ~debug
        % Save all the record in for new prediction
        all_annos.(fieldname).vp_car = vp_car; 
        sym_normal = inv(k_matrix*R)*all_annos.(fieldname).vp_car;
        sym_normal_unit = sym_normal/norm(sym_normal);
        all_annos.(fieldname).sym_normal_unit = sym_normal_unit;
        cMw = get_cMw_using_car_points(k_matrix,R,vp_car, origin,all_annos,...
            annotated_car_id, extracted_3D_points_car_coor);

        %%Gotta save more, check everything in all_unit tests, save more
        all_annos = save_extracted(annotated_car_id,all_annos,id_to_tire, cMw, car_model_param,...
            extracted_3D_points_car_coor,mirror_one_side, origin,image,k_matrix,R,estimated_dist_to_move);
    end
    
    %% Visualizing in 1D model
    

    %% Visualize to check
    if false
        edges = [
              1 2;
             2 3;
             3 4;
              4 1;
              5 6;
              6 7;
              7 8;
              8 5;
             1 5;
             2 6;
            3 7;
             4 8;
        ];
        figure(annotated_car_id+1)
        imshow(image)
        hold on
        fieldname = sprintf("obj_%s",int2str(annotated_car_id));
%         corners_bbox_world = all_annos.(fieldname).corners_bbox_world(1:3,:);
%         corners_bbox_2D = k_matrix*R*corners_bbox_world;
%         corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:);
%         predicted_corners = all_annos.(fieldname).predicted_corners(1:3,:);
%         predicted_corners_bbox_2D = k_matrix*R*predicted_corners;
%         predicted_corners_bbox_2D = predicted_corners_bbox_2D./predicted_corners_bbox_2D(3,:);
        scatter(all_annos.(fieldname).all_annotated_points(1,:),all_annos.(fieldname).all_annotated_points(2,:),20,'red','filled');
%         scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),all_annos.(fieldname).all_3D_points_to_2D(2,:),10,'green','filled');
%         scatter(all_3D_points_to_2D(1,:),all_3D_points_to_2D(2,:),10,'yellow','filled');
%         predicted_corners_opt = all_annos.(fieldname).predicted_corners_opt(1:3,:);
%         predicted_corners_opt_bbox_2D = k_matrix*R*predicted_corners_opt;
%         predicted_corners_opt_bbox_2D = predicted_corners_opt_bbox_2D./predicted_corners_opt_bbox_2D(3,:);

%         for i = 1:size(edges,1)
%             x_coor = [corners_bbox_2D(1, edges(i,1)) corners_bbox_2D(1, edges(i,2))];
%             y_coor = [corners_bbox_2D(2, edges(i,1)) corners_bbox_2D(2, edges(i,2))];
%             line(x_coor, y_coor,'Color','red','LineWidth',1)
%         end
%         for i = 1:size(edges,1)
%             x_coor = [predicted_corners_bbox_2D(1, edges(i,1)) predicted_corners_bbox_2D(1, edges(i,2))];
%             y_coor = [predicted_corners_bbox_2D(2, edges(i,1)) predicted_corners_bbox_2D(2, edges(i,2))];
%             line(x_coor, y_coor,'Color','green','LineWidth',1)
%         end
%         for i = 1:size(edges,1)
%             x_coor = [predicted_corners_opt_bbox_2D(1, edges(i,1)) predicted_corners_opt_bbox_2D(1, edges(i,2))];
%             y_coor = [predicted_corners_opt_bbox_2D(2, edges(i,1)) predicted_corners_opt_bbox_2D(2, edges(i,2))];
%             line(x_coor, y_coor,'Color','yellow','LineWidth',1)
%         end
%         scatter(all_car_points_world_2D(1,:),all_car_points_world_2D(2,:),4,'blue','filled');

    end
    %%
    if false
%         ptCloud_world_sample = randsample(size(ptCloud_world,2),2000);
%         y_pred = fobj.evaluate([ptCloud_world(1,ptCloud_world_sample)' ptCloud_world(3,ptCloud_world_sample)']);

        figure;
        hold on
%         pcshow([all_annos.(fieldname).predicted_corners_opt(1,:)',all_annos.(fieldname).predicted_corners_opt(2,:)',all_annos.(fieldname).predicted_corners_opt(3,:)'],'yellow','MarkerSize',700);
        pcshow([all_annos.(fieldname).pred_bbox_points_only(1,:)',all_annos.(fieldname).pred_bbox_points_only(2,:)',all_annos.(fieldname).pred_bbox_points_only(3,:)'],'yellow','MarkerSize',500);
        pcshow([all_annos.(fieldname).all_3D_points(1,:)',all_annos.(fieldname).all_3D_points(2,:)',all_annos.(fieldname).all_3D_points(3,:)'],'yellow','MarkerSize',50);
        pcshow([all_annos.(fieldname).pred_bbox(1,:)',all_annos.(fieldname).pred_bbox(2,:)',all_annos.(fieldname).pred_bbox(3,:)'],'green','MarkerSize',500);
        pcshow([all_annos.(fieldname).corners_bbox_world(1,:)',all_annos.(fieldname).corners_bbox_world(2,:)',all_annos.(fieldname).corners_bbox_world(3,:)'],'red','MarkerSize',500);
%         pcshow([all_annos.(fieldname).all_3D_points_opt(1,:)',all_annos.(fieldname).all_3D_points_opt(2,:)',all_annos.(fieldname).all_3D_points_opt(3,:)'],'yellow','MarkerSize',100);
        pcshow(ptCloud_world','blue','MarkerSize',3);
%         pcshow(ptCloud_above','red','MarkerSize',5);
%         pcshow([ptCloud_world(1,ptCloud_world_sample)' y_pred ptCloud_world(3,ptCloud_world_sample)'], 'yellow','MarkerSize',10);

        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        hold off
    end
end

