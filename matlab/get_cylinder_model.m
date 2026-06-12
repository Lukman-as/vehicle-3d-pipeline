function all_annos = get_cylinder_model(k_matrix,R,annotated_car_id,all_annos,id_to_tire, image)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    %Get the rotation into car system
    origin_sym_coor = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin));
    cMw = get_cMw_using_world_points(k_matrix,R,all_annos.(fieldname).vp_car, origin_sym_coor,all_annos,annotated_car_id,...
        all_annos.(fieldname).all_3D_points,image);
    ex_3D_car_coors = cMw*[all_annos.(fieldname).ex_3D; ones(1,size(all_annos.(fieldname).ex_3D,2))];
    nonex_3D_car_coors  = cMw*[all_annos.(fieldname).nonex_3D; ones(1,size(all_annos.(fieldname).nonex_3D,2))];
    all_3D_points_car_coor = cMw*[all_annos.(fieldname).all_3D_points; ones(1,size(all_annos.(fieldname).all_3D_points,2))];
    corners_pred = cMw*[all_annos.(fieldname).pred_bbox;ones(1,size(all_annos.(fieldname).pred_bbox,2))];

    ex_3D_car_coors = ex_3D_car_coors(1:3,:);    
    nonex_3D_car_coors = nonex_3D_car_coors(1:3,:);  
    sym_points = [ex_3D_car_coors, nonex_3D_car_coors]

    if all_annos.(fieldname).has_mirror
        mirror_points = nonex_3D_car_coors(:,end-1:end)
    end

    %Find the most forward point and the project onto bounding box
    [extreme_length,I] = min(corners_pred(1, :)) %Min or max depending on what you are looking at
    if all_annos.(fieldname).has_mirror
        mirror_points_extreme = mirror_points(:,1);
        mirror_points_extreme(1) = extreme_length;
    end



     %% Visualize on the 1D plane XY THESISVIS
    if true
        colors_bbox_2 = 'green';
        figure
        hold on
        set(gca, 'YDir','reverse')
        set(gca, 'XDir','reverse')

        %Plot predicted box
        line([corners_pred(1,5), corners_pred(1,8)],[corners_pred(2,5),...
             corners_pred(2,8)],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
        line([corners_pred(1,4), corners_pred(1,8)],[corners_pred(2,4),...
         corners_pred(2,8)],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
        line([corners_pred(1,1), corners_pred(1,5)],[corners_pred(2,1),...
        corners_pred(2,5)],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')
         line([corners_pred(1,1), corners_pred(1,4)],[corners_pred(2,1),...
         corners_pred(2,4)],'Color',colors_bbox_2,'LineWidth',2,'LineStyle','-')

        scatter(sym_points(1,:),sym_points(2,:),40,"magenta",'filled')

        if all_annos.(fieldname).has_mirror
             scatter(mirror_points(1,:),mirror_points(2,:),40,"blue",'filled')
             scatter(mirror_points_extreme(1,:),mirror_points_extreme(2,:),70,"black",'filled')
        end
        
        if all_annos.(fieldname).has_mirror
            all_points = [sym_points, mirror_points_extreme, all_3D_points_car_coor(1:3,1:size(all_annos.(fieldname).tire_ids,2))];
        else
            all_points = [sym_points, all_3D_points_car_coor(1:3,1:size(all_annos.(fieldname).tire_ids,2))];
        end


        K = convhull(all_points(1,:), all_points(2,:));
        convexHullX = all_points(1,K);
        convexHullY = all_points(2,K);
        % Plot the convex hull
        plot(convexHullX, convexHullY, 'r-', 'LineWidth', 2);

            
        %Visualize the tire
        start_pos = 0;
        for i = 1:size(all_annos.(fieldname).tire_ids,2)
            atire = all_3D_points_car_coor(:,start_pos+i);
            scatter(atire(1),atire(2),50,"green",'filled')
        end
        axis equal
        axis on
        xlabel('X')
        ylabel('Y')
    end


end
