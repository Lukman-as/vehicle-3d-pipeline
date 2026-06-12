function [bad_annotation, all_annos] = do_opt_hw7(annotated_car_id,all_annos,id_to_tire,horizon_line,k_matrix,R,image, car_model_param)
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
   [vp_car, origin, extracted_3D_points_car_coor] = extract_info_from_x(...
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
       all_3D_points_car_coor
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
            enforce_all = true;
            if enforce_all
                enforcing_min_height = true;
                enforcing_max_height = true;
                max_height = 3.05; %From the database
                enforcing_max_width = true;
                max_width = 2.93+0.4;%From the database + mirror
                enforcing_length = true;
            else
                enforcing_min_height = false;
                enforcing_max_height = false;
                enforcing_width = false;
                enforcing_length = false;
            end

            %Dimension to enforce
            max_width = 2.93+0.6; %From the database + mirror
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
                if all_annos.(fieldname).tire_both_sides
                    x0_ub(x0_order.ex_points) = max_width; %Add one for the separation when tire NOT on both sides
                end
                x0_ub(x0_order.ex_points+num_non_ex*2+1:end) = max_width; %All the non ex separation
            end

            if enforcing_length
                if all_annos.(fieldname).car_origin == 1 || all_annos.(fieldname).car_origin == 2
                    min_length = -max_FH;
                    max_length = max_WB + max_RH;
                    %Tirepoints
%                     x0_lb(x0_order.origin+1:x0_order.num_tires) = min_length;
                    x0_ub(x0_order.origin+1:x0_order.num_tires) = max_length;
                    x0_lb(x0_order.origin+1:x0_order.num_tires) = 0; %Because cannot cross to the front

                    %Ex points
                    x0_lb(x0_order.num_tires+1) = min_length;
                    x0_ub(x0_order.num_tires+1) = max_length;
                    %Non ex points
                    x0_lb(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = min_length;
                    x0_ub(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = max_length;
                else
                    min_length = -max_WB - max_FH;
                    max_length = max_RH;
                    x0_lb(x0_order.origin+1:x0_order.num_tires) = min_length;
%                     x0_ub(x0_order.origin+1:x0_order.num_tires) = max_length;
                    x0_ub(x0_order.origin+1:x0_order.num_tires) = 0; %Because cannot cross to the back
                    %Ex points
                    x0_lb(x0_order.num_tires+1) = min_length;
                    x0_ub(x0_order.num_tires+1) = max_length;
                    %Non ex points
                    x0_lb(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = min_length;
                    x0_ub(x0_order.ex_points+1:x0_order.ex_points+num_non_ex) = max_length;
                end
            end
            x = lsqnonlin(@(x0) coop_fun_lsq(k_matrix,R,all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon),x0,...
                x0_lb,x0_ub, options);
        else
            x = lsqnonlin(@(x0) coop_fun_lsq(k_matrix,R,all_annos,annotated_car_id,x0_order,id_to_tire, x0,horizon_line,R_horizon),x0,...
            [],[], options);
   end
    
    %% Convert back to world
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

        cMw = get_cMw_using_car_points(k_matrix,R,vp_car, origin,all_annos,annotated_car_id, extracted_3D_points_car_coor); 
        estimated_3d_points = inv(cMw)*extracted_3D_points_car_coor;
        estimated_3d_points = estimated_3d_points(1:3,:);
        all_annos.(fieldname).all_3D_points = estimated_3d_points;

        %Save car origin
        all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin)) = origin;
        
% This is in get_reproj_error_with_3D_points already
%         all_3D_points_to_2D = k_matrix*R*estimated_3d_points;
%         all_3D_points_to_2D = all_3D_points_to_2D./all_3D_points_to_2D(3,:); 
%         all_annos.(fieldname).all_3D_points_to_2D = all_3D_points_to_2D;
        
        %%Gotta save more, check everything in all_unit tests, save more
        all_annos = save_extracted(annotated_car_id,all_annos,id_to_tire,estimated_3d_points, cMw, car_model_param, extracted_3D_points_car_coor);
    end
    
    %% Visualizing in 1D model
    if false
        corners_velo = cMw*[all_annos.(fieldname).corners_bbox_world;ones(1,size(all_annos.(fieldname).corners_bbox_world,2))];
        corners_velo = corners_velo(1:3,:);
        
        corners_pred = cMw*[all_annos.(fieldname).pred_bbox;ones(1,size(all_annos.(fieldname).pred_bbox,2))];
        corners_pred = corners_pred(1:3,:);

        buff = 1;
        min_x = min([min(corners_pred(1,:)),min(corners_velo(1,:))])-buff;
        max_x = max([max(corners_pred(1,:)),max(corners_velo(1,:))])+buff;
        min_y = min([min(corners_pred(2,:)),min(corners_velo(2,:))])-buff/2;
        max_y = max([max(corners_pred(2,:)),max(corners_velo(2,:))])+buff/2;
        min_z = min([min(corners_pred(3,:)),min(corners_velo(3,:))])-buff;
        max_z = max([max(corners_pred(3,:)),max(corners_velo(3,:))])+buff;


        %TODO for viasualization for now
        corners_points_pred = cMw*[all_annos.(fieldname).pred_bbox_points_only;ones(1,size(all_annos.(fieldname).pred_bbox_points_only,2))];
        corners_points_pred = corners_points_pred(1:3,:);

        corners_pred = cMw*[all_annos.(fieldname).pred_bbox;ones(1,size(all_annos.(fieldname).pred_bbox,2))];
        corners_pred = corners_pred(1:3,:);

        %Get LiDAR points surrounding
        velo = cMw*[ptCloud_world;ones(1,size(ptCloud_world,2))];
        velo = velo(1:3,:);
        velo = velo';
        good_rows = find(velo(:,1) >= min_x & velo(:,1)  <= max_x &...
            velo(:,2) >= min_y & velo(:,2)  <= max_y &...
            velo(:,3) >= min_z & velo(:,3)  <= max_z);
        all_car_points = velo(good_rows,1:3)';

        %Get LiDAR points terrain only
        velo = cMw*[ptCloud_above;ones(1,size(ptCloud_above,2))];
        velo = velo(1:3,:);
        velo = velo';
        good_rows = find(velo(:,1) >= min_x & velo(:,1)  <= max_x &...
            velo(:,2) >= min_y & velo(:,2)  <= max_y &...
            velo(:,3) >= min_z & velo(:,3)  <= max_z);
        all_car_points_above = velo(good_rows,1:3)';

%         all_annos.(fieldname).num_lidar_returns = size(good_rows,1); %Not good, not including buffer

        
        colors_bbox_1 = 'red';
        colors_bbox_2 = 'green';
        colors_bbox_3 = 'magenta';

   
        %% Visualize on the 1D plane XY THESISVIS
        if true
            figure
            hold on
            set(gca, 'YDir','reverse')
            set(gca, 'XDir','reverse')
            %Plot GT box
    %         scatter(corners_velo(1,:),corners_velo(2,:),20,'red','filled')
            line([corners_velo(1,5), corners_velo(1,8)],[corners_velo(2,5),...
                 corners_velo(2,8)],'Color',colors_bbox_1,'LineWidth',4,'LineStyle','-')
            line([corners_velo(1,4), corners_velo(1,8)],[corners_velo(2,4),...
             corners_velo(2,8)],'Color',colors_bbox_1,'LineWidth',4,'LineStyle','-')
            line([corners_velo(1,1), corners_velo(1,5)],[corners_velo(2,1),...
            corners_velo(2,5)],'Color',colors_bbox_1,'LineWidth',4,'LineStyle','-')
             line([corners_velo(1,1), corners_velo(1,4)],[corners_velo(2,1),...
             corners_velo(2,4)],'Color',colors_bbox_1,'LineWidth',4,'LineStyle','-')
    
            %Plot predicted box
    %         scatter(corners_pred(1,:),corners_pred(2,:),20,colors_bbox_2,'filled')
%             line([corners_pred(1,5), corners_pred(1,8)],[corners_pred(2,5),...
%                  corners_pred(2,8)],'Color',colors_bbox_2,'LineWidth',4,'LineStyle','-')
%             line([corners_pred(1,4), corners_pred(1,8)],[corners_pred(2,4),...
%              corners_pred(2,8)],'Color',colors_bbox_2,'LineWidth',4,'LineStyle','-')
%             line([corners_pred(1,1), corners_pred(1,5)],[corners_pred(2,1),...
%             corners_pred(2,5)],'Color',colors_bbox_2,'LineWidth',4,'LineStyle','-')
%              line([corners_pred(1,1), corners_pred(1,4)],[corners_pred(2,1),...
%              corners_pred(2,4)],'Color',colors_bbox_2,'LineWidth',4,'LineStyle','-')
    
            %Points only
    %         scatter(corners_points_pred(1,:),corners_points_pred(2,:),20,'black','filled')
%             line([corners_points_pred(1,5), corners_points_pred(1,8)],[corners_points_pred(2,5),...
%                  corners_points_pred(2,8)],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
%             line([corners_points_pred(1,4), corners_points_pred(1,8)],[corners_points_pred(2,4),...
%              corners_points_pred(2,8)],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
%             line([corners_points_pred(1,1), corners_points_pred(1,5)],[corners_points_pred(2,1),...
%             corners_points_pred(2,5)],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
%              line([corners_points_pred(1,1), corners_points_pred(1,4)],[corners_points_pred(2,1),...
%              corners_points_pred(2,4)],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
    
            scatter(all_car_points(1,:),all_car_points(2,:),15,'blue','filled')
%             scatter(extracted_3D_points_car_coor(1,:),extracted_3D_points_car_coor(2,:),80,"magenta",'filled')
%             start_pos = 0;
%             for i = 1:size(all_annos.(fieldname).tire_ids,2)
%                 atire = extracted_3D_points_car_coor(:,start_pos+i);
%                 scatter(atire(1),atire(2),80,"black",'filled')
%             end

            %Custom legend
%             qw{1} = plot(nan, 'o','Color','magenta','MarkerSize',8,'MarkerFaceColor','magenta');
%             qw{2} = plot(nan, 'o','Color','black','MarkerSize',8,'MarkerFaceColor','black');
%             qw{3} = plot(nan, '-','Color','green','LineWidth',4);
%             qw{4} = plot(nan, '-','Color','red','LineWidth',4);
%             qw{5} = plot(nan, '.','Color','blue','MarkerSize',15);
%             legend([qw{:}], {'Predicted 3D locations of symmetry point pairs',...
%                 'Predicted 3D locations of tire-ground contact points ',...
%                 'Predicted 3D bounding box',...
%                 'LiDAR-based bounding box',...
%                 'LiDAR points'}, 'location', 'best', 'FontSize', 20)

            qw{1} = plot(nan, '-','Color','red','LineWidth',4);
            qw{2} = plot(nan, '.','Color','blue','MarkerSize',15);
            legend([qw{:}], {
                'LiDAR-based bounding box',...
                'LiDAR points'}, 'location', 'northeast', 'FontSize', 15)

            set(gca,'fontsize',15)
            multiple = 0.5;
            ylimits = [-2.7 1];
            xlimits = [-1.1 3.8];
            ylim(ylimits);
            xlim(xlimits);
            xt = round(xlimits(1)/multiple)*multiple:multiple:round(xlimits(2)/multiple)*multiple;
            xticks(xt);
            yt = round(ylimits(1)/multiple)*multiple:multiple:round(ylimits(2)/multiple)*multiple;
            yticks(yt);
            ytickformat('%.2f');
            xtickformat('%.2f')
            xlabel('X')
            ylabel('Y')
            axis on
            axis equal
        end
        

        %% Visualize on the 1D plane XZ
        if false
            figure
            hold on
            set(gca, 'YDir','reverse')
            set(gca, 'XDir','reverse')
            x_axis = 1;
            y_axis = 3;
            links =  [1 2; 2 3; 3 4; 4 1];
                  

            %Plot GT box
            line([corners_velo(x_axis,links(1,1)), corners_velo(x_axis,links(1,2))],[corners_velo(y_axis,links(1,1)),...
                 corners_velo(y_axis,links(1,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
            line([corners_velo(x_axis,links(2,1)), corners_velo(x_axis,links(2,2))],[corners_velo(y_axis,links(2,1)),...
             corners_velo(y_axis,links(2,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
            line([corners_velo(x_axis,links(3,1)), corners_velo(x_axis,links(3,2))],[corners_velo(y_axis,links(3,1)),...
            corners_velo(y_axis,links(3,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
             line([corners_velo(x_axis,links(4,1)), corners_velo(x_axis,links(4,2))],[corners_velo(y_axis,links(4,1)),...
             corners_velo(y_axis,links(4,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
    
            %Plot predicted box
            line([corners_pred(x_axis,links(1,1)), corners_pred(x_axis,links(1,2))],[corners_pred(y_axis,links(1,1)),...
                 corners_pred(y_axis,links(1,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
            line([corners_pred(x_axis,links(2,1)), corners_pred(x_axis,links(2,2))],[corners_pred(y_axis,links(2,1)),...
             corners_pred(y_axis,links(2,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
            line([corners_pred(x_axis,links(3,1)), corners_pred(x_axis,links(3,2))],[corners_pred(y_axis,links(3,1)),...
            corners_pred(y_axis,links(3,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
             line([corners_pred(x_axis,links(4,1)), corners_pred(x_axis,links(4,2))],[corners_pred(y_axis,links(4,1)),...
             corners_pred(y_axis,links(4,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
    
            %Points only
%              line([corners_points_pred(x_axis,links(1,1)), corners_points_pred(x_axis,links(1,2))],[corners_points_pred(y_axis,links(1,1)),...
%                  corners_points_pred(y_axis,links(1,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
%             line([corners_points_pred(x_axis,links(2,1)), corners_points_pred(x_axis,links(2,2))],[corners_points_pred(y_axis,links(2,1)),...
%              corners_points_pred(y_axis,links(2,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
%             line([corners_points_pred(x_axis,links(3,1)), corners_points_pred(x_axis,links(3,2))],[corners_points_pred(y_axis,links(3,1)),...
%             corners_points_pred(y_axis,links(3,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
%              line([corners_points_pred(x_axis,links(4,1)), corners_points_pred(x_axis,links(4,2))],[corners_points_pred(y_axis,links(4,1)),...
%              corners_points_pred(y_axis,links(4,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')

            scatter(all_car_points_above(x_axis,:),all_car_points_above(y_axis,:),5,'blue','filled')
%             scatter(extracted_3D_points_car_coor(x_axis,:),extracted_3D_points_car_coor(y_axis,:),40,"magenta",'filled')
%     
%             start_pos = 0;
%             for i = 1:size(all_annos.(fieldname).tire_ids,2)
%                 atire = extracted_3D_points_car_coor(:,start_pos+i);
%                 scatter(atire(x_axis),atire(y_axis),40,"green",'filled')
%             end
            axis equal
            axis on
            xlabel('X')
            ylabel('Z')
        end

         %% Visualize on the 1D plane XZ
        if false
            figure
            hold on
            set(gca, 'YDir','reverse')
            set(gca, 'XDir','reverse')
            x_axis = 3;
            y_axis = 2;
            links =  [1 2; 2 6; 6 5; 5 1];
                  
            %Plot GT box
            line([corners_velo(x_axis,links(1,1)), corners_velo(x_axis,links(1,2))],[corners_velo(y_axis,links(1,1)),...
                 corners_velo(y_axis,links(1,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
            line([corners_velo(x_axis,links(2,1)), corners_velo(x_axis,links(2,2))],[corners_velo(y_axis,links(2,1)),...
             corners_velo(y_axis,links(2,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
            line([corners_velo(x_axis,links(3,1)), corners_velo(x_axis,links(3,2))],[corners_velo(y_axis,links(3,1)),...
            corners_velo(y_axis,links(3,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
             line([corners_velo(x_axis,links(4,1)), corners_velo(x_axis,links(4,2))],[corners_velo(y_axis,links(4,1)),...
             corners_velo(y_axis,links(4,2))],'Color',colors_bbox_1,'LineWidth',2,'LineStyle','-')
    
            %Plot predicted box
            line([corners_pred(x_axis,links(1,1)), corners_pred(x_axis,links(1,2))],[corners_pred(y_axis,links(1,1)),...
                 corners_pred(y_axis,links(1,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
            line([corners_pred(x_axis,links(2,1)), corners_pred(x_axis,links(2,2))],[corners_pred(y_axis,links(2,1)),...
             corners_pred(y_axis,links(2,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
            line([corners_pred(x_axis,links(3,1)), corners_pred(x_axis,links(3,2))],[corners_pred(y_axis,links(3,1)),...
            corners_pred(y_axis,links(3,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
             line([corners_pred(x_axis,links(4,1)), corners_pred(x_axis,links(4,2))],[corners_pred(y_axis,links(4,1)),...
             corners_pred(y_axis,links(4,2))],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
    
            %Points only
             line([corners_points_pred(x_axis,links(1,1)), corners_points_pred(x_axis,links(1,2))],[corners_points_pred(y_axis,links(1,1)),...
                 corners_points_pred(y_axis,links(1,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
            line([corners_points_pred(x_axis,links(2,1)), corners_points_pred(x_axis,links(2,2))],[corners_points_pred(y_axis,links(2,1)),...
             corners_points_pred(y_axis,links(2,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
            line([corners_points_pred(x_axis,links(3,1)), corners_points_pred(x_axis,links(3,2))],[corners_points_pred(y_axis,links(3,1)),...
            corners_points_pred(y_axis,links(3,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')
             line([corners_points_pred(x_axis,links(4,1)), corners_points_pred(x_axis,links(4,2))],[corners_points_pred(y_axis,links(4,1)),...
             corners_points_pred(y_axis,links(4,2))],'Color',colors_bbox_3,'LineWidth',2,'LineStyle','-')

            scatter(all_car_points(x_axis,:),all_car_points(y_axis,:),5,'blue','filled')
            scatter(extracted_3D_points_car_coor(x_axis,:),extracted_3D_points_car_coor(y_axis,:),40,"magenta",'filled')
    
            start_pos = 0;
            for i = 1:size(all_annos.(fieldname).tire_ids,2)
                atire = extracted_3D_points_car_coor(:,start_pos+i);
                scatter(atire(x_axis),atire(y_axis),40,"green",'filled')
            end
            axis equal
            axis on
            xlabel('Z')
            ylabel('Y')
        end
    end
    

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