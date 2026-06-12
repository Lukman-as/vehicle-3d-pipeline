function visualize_side_view(annotated_car_id,all_annos, k_matrix,R,image,id_to_tire,ptCloud_world,reverse_x)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    origin_sym_coor = all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin));
    cMw = get_cMw_using_world_points(k_matrix,R, ...
        all_annos.(fieldname).vp_car, ...
        origin_sym_coor, ...
        all_annos,annotated_car_id, ...
        all_annos.(fieldname).all_3D_points, ...
        image);

    corners_velo = cMw*[all_annos.(fieldname).corners_bbox_world;ones(1,size(all_annos.(fieldname).corners_bbox_world,2))];
    corners_velo = corners_velo(1:3,:);
    corners_pred = cMw*[all_annos.(fieldname).pred_bbox;ones(1,size(all_annos.(fieldname).pred_bbox,2))];
    corners_pred = corners_pred(1:3,:);

    %Setup the buffer around these box toinclude morepoints
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

   [in,on] = inpolygon(velo(:,1),velo(:,3),corners_velo(1,:)',corners_velo(3,:)');
   velo_sel = velo(in,1:3);
   good_rows = find(velo_sel(:,2) >= min_y & velo_sel(:,2)  <= max_y);
   all_car_points = velo_sel(good_rows,1:3)';


    %Get LiDAR points terrain only
%     velo = cMw*[ptCloud_above;ones(1,size(ptCloud_above,2))];
%     velo = velo(1:3,:);
%     velo = velo';
%     good_rows = find(velo(:,1) >= min_x & velo(:,1)  <= max_x &...
%         velo(:,2) >= min_y & velo(:,2)  <= max_y &...
%         velo(:,3) >= min_z & velo(:,3)  <= max_z);
%     all_car_points_above = velo(good_rows,1:3)';


    colors_bbox_1 = 'red';
    colors_bbox_2 = 'green';
    colors_bbox_3 = 'magenta';


   %% Setup
   visualize_xy = true;
   visualize_xz = true;
   visualize_zy = false;
   line_size = 4;
   visualize_lidar_box = false;
   visualize_complete_box = true;
   visualize_points_box = true;
   visualize_predicted_points = true;
%    buff = 1;
    
    %% Visualize on the 1D plane XY THESISVIS
    if visualize_xy
        figure
        hold on
        set(gca, 'YDir','reverse')
        if ~reverse_x
            set(gca, 'XDir','reverse')
        end
        x_axis = 1;
        y_axis = 2;
        %Plot GT box
        if visualize_lidar_box
    %         scatter(corners_velo(1,:),corners_velo(2,:),20,'red','filled')
            line([corners_velo(1,5), corners_velo(1,8)],[corners_velo(2,5),...
                 corners_velo(2,8)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
            line([corners_velo(1,4), corners_velo(1,8)],[corners_velo(2,4),...
             corners_velo(2,8)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
            line([corners_velo(1,1), corners_velo(1,5)],[corners_velo(2,1),...
            corners_velo(2,5)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
             line([corners_velo(1,1), corners_velo(1,4)],[corners_velo(2,1),...
             corners_velo(2,4)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
        end

        %Plot predicted box
        if visualize_complete_box
            scatter(corners_pred(1,:),corners_pred(2,:),20,colors_bbox_2,'filled')
                line([corners_pred(1,5), corners_pred(1,8)],[corners_pred(2,5),...
                     corners_pred(2,8)],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
                line([corners_pred(1,4), corners_pred(1,8)],[corners_pred(2,4),...
                 corners_pred(2,8)],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
                line([corners_pred(1,1), corners_pred(1,5)],[corners_pred(2,1),...
                corners_pred(2,5)],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
                 line([corners_pred(1,1), corners_pred(1,4)],[corners_pred(2,1),...
                 corners_pred(2,4)],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
        end

        %Points only
        if visualize_points_box
            scatter(corners_points_pred(1,:),corners_points_pred(2,:),20,'black','filled')
                line([corners_points_pred(1,5), corners_points_pred(1,8)],[corners_points_pred(2,5),...
                     corners_points_pred(2,8)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                line([corners_points_pred(1,4), corners_points_pred(1,8)],[corners_points_pred(2,4),...
                 corners_points_pred(2,8)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                line([corners_points_pred(1,1), corners_points_pred(1,5)],[corners_points_pred(2,1),...
                corners_points_pred(2,5)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                 line([corners_points_pred(1,1), corners_points_pred(1,4)],[corners_points_pred(2,1),...
                 corners_points_pred(2,4)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
        end
        
        %Plotting all cars points
        scatter(all_car_points(x_axis,:),all_car_points(y_axis,:),30,'blue','filled')

        %Plotting special points
        if visualize_predicted_points
            all_3D_points_car_coor = cMw*[all_annos.(fieldname).all_3D_points; ones(1,size(all_annos.(fieldname).all_3D_points,2))];
            scatter(all_3D_points_car_coor(1,:),all_3D_points_car_coor(2,:),80,"green",'filled')
            start_pos = 0;
            for i = 1:size(all_annos.(fieldname).tire_ids,2)
%                 'a tireeee'
                atire = all_3D_points_car_coor(:,start_pos+i);
%                 scatter(atire(1),atire(2),80,"black",'filled')
                scatter(atire(x_axis), atire(y_axis), 80, 'green', 'filled', 'MarkerEdgeColor', 'black', 'LineWidth', 2)

            end
        end

        %Custom legend
        %             qw{1} = plot(nan, 'o','Color','magenta','MarkerSize',8,'MarkerFaceColor','magenta');
        %             qw{2} = plot(nan, 'o','Color','black','MarkerSize',8,'MarkerFaceColor','black');
        %             qw{3} = plot(nan, '-','Color','green','LineWidth',4);
        %             qw{4} = plot(nan, '-','Color','red','LineWidth',4);
        %             qw{5} = plot(nan, '.','Color','blue','MarkerSize',15);
        %             legend([qw{:}], {'Predicted 3D locations of symmetry point pairs',...
        %                 'Predicted 3D locations of tire-ground contact points ',...
        %                 'Predicted 3D bounding box',...
        %                 'LiDAR-derived bounding box',...
        %                 'LiDAR points'}, 'location', 'best', 'FontSize', 20)
        
        %METHOD Figure of Bounding Points only and Bbox Gaussian
        qw{1} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green');
        qw{2} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green','MarkerEdgeColor', 'black', 'LineWidth', 3);
        qw{3} = plot(nan, '-','Color',colors_bbox_3,'LineWidth',line_size);
        qw{4} = plot(nan, '-','Color',colors_bbox_2,'LineWidth',line_size);
        qw{5} = plot(nan, '.','Color','blue','MarkerSize',30);
        legend([qw{:}], {
            'Predicted 3D locations of symmetry point pairs',...
            'Predicted 3D locations of tire-ground contact points ',...
            'Predicted 3D bounding box enclosing 3D predicted points only',...
            'Predicted 3D bounding box after Multivariate Gaussian model',...
            'LiDAR points'},'location', 'best', 'FontSize', 20)
        


        %METHOD  A visualization of a produced bounding box before and after 
%         adopting constraints with the most improve- ment in IOU using Pandas
%         qw{1} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green');
%         qw{2} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green','MarkerEdgeColor', 'black', 'LineWidth', 3);
%         qw{3} = plot(nan, '-','Color','green','LineWidth',4);
%         qw{4} = plot(nan, '-','Color','red','LineWidth',4);
%         qw{5} = plot(nan, '.','Color','blue','MarkerSize',15);
%         legend([qw{:}], {'Predicted 3D locations of symmetry point pairs',...
%                     'Predicted 3D locations of tire-ground contact points ',...
%                     'Predicted 3D bounding box',...
%                     'LiDAR-derived bounding box',...
%                     'LiDAR points'}, 'location', 'best', 'FontSize', 20)




        ylimits = [min_y  max_y]; 
        xlimits = [min_x  max_x];

        set(gca,'fontsize',30)
        multiple = 0.5;
        ylim(ylimits);
        xlim(xlimits);
        % Calculate and set x and y ticks
        xt = round(xlimits(1)/multiple)*multiple:multiple:round(xlimits(2)/multiple)*multiple;
        xticks(xt);
        yt = round(ylimits(1)/multiple)*multiple:multiple:round(ylimits(2)/multiple)*multiple;
        yticks(yt);

        ytickformat('%.1f');
        xtickformat('%.1f')
        xlabel('X')
        ylabel('Y')
        axis on
        axis equal

        % Set tick label font size
        tickLabelFontSize = 25; % Adjust the font size as needed
        ax = gca; % Get the current axes
        ax.FontSize = tickLabelFontSize; % Set the font size for both x and y tick labels


        % Add text labels at the leftmost and rightmost of the X axis
        % Adjust the vertical position as needed
        verticalPosition = ylimits(2)  + 0.5;
        if reverse_x
            text(xlimits(1)+1.5, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
            text(xlimits(2)-1.5, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
        else
            text(xlimits(1)+0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
            text(xlimits(2)-0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
        end

%         text(xlimits(1)+0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
%         text(xlimits(2)-0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
%     
    
    end
   
    %% Visualize on the 1D plane XZ
    if visualize_xz
        figure
        hold on
        if ~reverse_x
            set(gca, 'YDir','reverse')
            set(gca, 'XDir','reverse')
        end
        x_axis = 1;
        y_axis = 3;
        links =  [1 2; 2 3; 3 4; 4 1];
        %Plot GT box
        if visualize_lidar_box
            line([corners_velo(x_axis,links(1,1)), corners_velo(x_axis,links(1,2))],[corners_velo(y_axis,links(1,1)),...
                 corners_velo(y_axis,links(1,2))],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
            line([corners_velo(x_axis,links(2,1)), corners_velo(x_axis,links(2,2))],[corners_velo(y_axis,links(2,1)),...
             corners_velo(y_axis,links(2,2))],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
            line([corners_velo(x_axis,links(3,1)), corners_velo(x_axis,links(3,2))],[corners_velo(y_axis,links(3,1)),...
            corners_velo(y_axis,links(3,2))],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
             line([corners_velo(x_axis,links(4,1)), corners_velo(x_axis,links(4,2))],[corners_velo(y_axis,links(4,1)),...
             corners_velo(y_axis,links(4,2))],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
        end 

        %Plot predicted box
        if visualize_complete_box
            line([corners_pred(x_axis,links(1,1)), corners_pred(x_axis,links(1,2))],[corners_pred(y_axis,links(1,1)),...
                 corners_pred(y_axis,links(1,2))],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
            line([corners_pred(x_axis,links(2,1)), corners_pred(x_axis,links(2,2))],[corners_pred(y_axis,links(2,1)),...
             corners_pred(y_axis,links(2,2))],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
            line([corners_pred(x_axis,links(3,1)), corners_pred(x_axis,links(3,2))],[corners_pred(y_axis,links(3,1)),...
            corners_pred(y_axis,links(3,2))],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
             line([corners_pred(x_axis,links(4,1)), corners_pred(x_axis,links(4,2))],[corners_pred(y_axis,links(4,1)),...
             corners_pred(y_axis,links(4,2))],'Color',colors_bbox_2,'LineWidth',line_size,'LineStyle','-')
        end

        %Points only
        if visualize_points_box
             line([corners_points_pred(x_axis,links(1,1)), corners_points_pred(x_axis,links(1,2))],[corners_points_pred(y_axis,links(1,1)),...
                 corners_points_pred(y_axis,links(1,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
            line([corners_points_pred(x_axis,links(2,1)), corners_points_pred(x_axis,links(2,2))],[corners_points_pred(y_axis,links(2,1)),...
             corners_points_pred(y_axis,links(2,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
            line([corners_points_pred(x_axis,links(3,1)), corners_points_pred(x_axis,links(3,2))],[corners_points_pred(y_axis,links(3,1)),...
            corners_points_pred(y_axis,links(3,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
             line([corners_points_pred(x_axis,links(4,1)), corners_points_pred(x_axis,links(4,2))],[corners_points_pred(y_axis,links(4,1)),...
             corners_points_pred(y_axis,links(4,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
        end

        scatter(all_car_points(x_axis,:),all_car_points(y_axis,:),30,'blue','filled')
        if visualize_predicted_points
%             scatter(extracted_3D_points_car_coor(x_axis,:),extracted_3D_points_car_coor(y_axis,:),40,"magenta",'filled')
%             start_pos = 0;
%             for i = 1:size(all_annos.(fieldname).tire_ids,2)
%                 atire = extracted_3D_points_car_coor(:,start_pos+i);
%                 scatter(atire(x_axis),atire(y_axis),40,"green",'filled')
%             end

            all_3D_points_car_coor = cMw*[all_annos.(fieldname).all_3D_points; ones(1,size(all_annos.(fieldname).all_3D_points,2))];
            scatter(all_3D_points_car_coor(x_axis,:),all_3D_points_car_coor(y_axis,:),80,"green",'filled')
            start_pos = 0;
            for i = 1:size(all_annos.(fieldname).tire_ids,2)
%                 'a tireeee'
                atire = all_3D_points_car_coor(:,start_pos+i);
%                 scatter(atire(x_axis),atire(y_axis),80,"black",'filled')
                scatter(atire(x_axis), atire(y_axis), 80, 'green', 'filled', 'MarkerEdgeColor', 'black', 'LineWidth', 3)

            end
        end
%         qw{1} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green');
%         qw{2} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green','MarkerEdgeColor', 'black', 'LineWidth', 3);
%         qw{3} = plot(nan, '-','Color','green','LineWidth',4);
%         qw{4} = plot(nan, '-','Color','red','LineWidth',4);
%         qw{5} = plot(nan, '.','Color','blue','MarkerSize',15);
%         legend([qw{:}], {'Predicted 3D locations of symmetry point pairs',...
%                     'Predicted 3D locations of tire-ground contact points ',...
%                     'Predicted 3D bounding box',...
%                     'LiDAR-derived bounding box',...
%                     'LiDAR points'}, 'location', 'best', 'FontSize', 20)


        ylimits = [min_z  max_z]; 
        xlimits = [min_x  max_x];
        set(gca,'fontsize',30)
        multiple = 0.5;
        ylim(ylimits);
        xlim(xlimits);
        xt = round(xlimits(1)/multiple)*multiple:multiple:round(xlimits(2)/multiple)*multiple;
        xticks(xt);
        yt = round(ylimits(1)/multiple)*multiple:multiple:round(ylimits(2)/multiple)*multiple;
        yticks(yt);
        ytickformat('%.1f');
        xtickformat('%.1f')
        axis equal
        axis on
        xlabel('X')
        ylabel('Z')
        tickLabelFontSize = 25; % Adjust the font size as needed
        ax = gca; % Get the current axes
        ax.FontSize = tickLabelFontSize; % Set the font size for both x and y tick labels
        % Add text labels at the leftmost and rightmost of the X axis
        % Adjust the vertical position as needed
        if reverse_x
            verticalPosition = ylimits(1) - 0.5;
        else
            verticalPosition = ylimits(2) - 0.5;
        end
        if reverse_x
            text(xlimits(1)+1.3, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
            text(xlimits(2)-1.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
        else
            text(xlimits(1)+0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
            text(xlimits(2)-0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
        end

%         text(xlimits(1)+0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
%         text(xlimits(2)-0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
%         
    end

     %% Visualize on the 1D plane ZY
    if visualize_zy
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