function all_annos = get_lidar_box_non_propagate(annotated_car_id,all_annos, k_matrix,R,image,id_to_tire,ptCloud_world)
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
    max_x = max(corners_velo(1,:));
    min_x = min(corners_velo(1,:));
    max_y = max(corners_velo(2,:));
    min_y = min(corners_velo(2,:));
    max_z = max(corners_velo(3,:));
    min_z = min(corners_velo(3,:));

   %Get LiDAR points surrounding
   velo = cMw*[ptCloud_world;ones(1,size(ptCloud_world,2))];
   velo = velo(1:3,:);
   velo = velo';

    
   [in,on] = inpolygon(velo(:,1),velo(:,3),corners_velo(1,:)',corners_velo(3,:)');
   velo_sel = velo(in,1:3);
   good_rows = find(velo_sel(:,2) >= min_y & velo_sel(:,2)  <= max_y);
   all_car_points = velo_sel(good_rows,1:3)';


   %Get the box in 2D
   all_car_points_xz = [all_car_points(1,:); all_car_points(3,:)];
   noise_level = 0.001;
   if size(all_car_points_xz,2) > 0
       if size(all_car_points_xz,2) < 3 %Add random noise to have enough points
           for i = 1:3-size(all_car_points_xz,2)
               all_car_points_xz = [all_car_points_xz, ...
                   [all_car_points_xz(1) + noise_level*randn(); all_car_points_xz(2) + noise_level*randn()]]
           end
       end
       assert(size(all_car_points_xz,2) >= 3)
       bb = minBoundingBox(double(all_car_points_xz));
       %Arrange BB correctly
       new_min_y = min(all_car_points(2,:));
       new_max_y = max(all_car_points(2,:));
       gt_box_nonpropagate = [bb(1,:),bb(1,:); 
           new_max_y*ones(1,4), new_min_y*ones(1,4),;
           bb(2,:),bb(2,:)];
    
    
       colors_bbox_1 = 'red';
       colors_bbox_2 = 'green';
       colors_bbox_3 = 'blue';
    
       %% Setup
       visualize_xy = false;
       visualize_xz = false;
       visualize_zy = false;
       line_size = 3;
       visualize_lidar_box = true;
       visualize_complete_box = true;
       visualize_points_box = false;
       visualize_predicted_points = true; 
    
      %% Visualize on the 1D plane XY THESISVIS
      reverse_x = 1;
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
                line([corners_velo(1,5), corners_velo(1,8)],[corners_velo(2,5),...
                     corners_velo(2,8)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
                line([corners_velo(1,4), corners_velo(1,8)],[corners_velo(2,4),...
                 corners_velo(2,8)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
                line([corners_velo(1,1), corners_velo(1,5)],[corners_velo(2,1),...
                corners_velo(2,5)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
                 line([corners_velo(1,1), corners_velo(1,4)],[corners_velo(2,1),...
                 corners_velo(2,4)],'Color',colors_bbox_1,'LineWidth',line_size,'LineStyle','-')
            end
    
            if true
                line([gt_box_nonpropagate(1,6), gt_box_nonpropagate(1,8)],[gt_box_nonpropagate(2,6),...
                     gt_box_nonpropagate(2,8)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                line([gt_box_nonpropagate(1,2), gt_box_nonpropagate(1,4)],[gt_box_nonpropagate(2,2),...
                     gt_box_nonpropagate(2,4)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                line([gt_box_nonpropagate(1,8), gt_box_nonpropagate(1,4)],[gt_box_nonpropagate(2,8),...
                     gt_box_nonpropagate(2,4)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')

                line([gt_box_nonpropagate(1,2), gt_box_nonpropagate(1,6)],[gt_box_nonpropagate(2,2),...
                 gt_box_nonpropagate(2,6)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
%                 line([gt_box_nonpropagate(1,1), gt_box_nonpropagate(1,5)],[gt_box_nonpropagate(2,1),...
%                 gt_box_nonpropagate(2,5)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                 line([gt_box_nonpropagate(1,1), gt_box_nonpropagate(1,2)],[gt_box_nonpropagate(2,2),...
                 gt_box_nonpropagate(2,2)],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
            end 
         
    
            %Plotting all cars points
            scatter(all_car_points(x_axis,:),all_car_points(y_axis,:),20,'blue','filled')
    
    %         %Plotting special points
    %         if visualize_predicted_points
    %             all_3D_points_car_coor = cMw*[all_annos.(fieldname).all_3D_points; ones(1,size(all_annos.(fieldname).all_3D_points,2))];
    %             scatter(all_3D_points_car_coor(1,:),all_3D_points_car_coor(2,:),80,"green",'filled')
    %             start_pos = 0;
    %             for i = 1:size(all_annos.(fieldname).tire_ids,2)
    %                 atire = all_3D_points_car_coor(:,start_pos+i);
    %                 scatter(atire(x_axis), atire(y_axis), 80, 'green', 'filled', 'MarkerEdgeColor', 'black', 'LineWidth', 2)
    % 
    %             end
    %         end

            %METHOD  A visualization of a produced bounding box before and after 
            % adopting constraints with the most improve- ment in IOU using Pandas
            qw{1} = plot(nan, '-','Color','blue','LineWidth',4);
            qw{2} = plot(nan, '-','Color','red','LineWidth',4);
            qw{3} = plot(nan, '.','Color','blue','MarkerSize',15);
            legend([qw{:}], {
                'LiDAR-derived bounding box',...
                'LiDAR-derived bounding box with inter-frame propagation',...
                'LiDAR points'}, 'location', 'best', 'FontSize', 20)
        
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
%             if ~reverse_x
%                 text(xlimits(1)+0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
%                 text(xlimits(2)-0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
%             else
%                 text(xlimits(1)+0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
%                 text(xlimits(2)-0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
%             end  
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
    
            if true
                line([gt_box_nonpropagate(x_axis,links(1,1)), gt_box_nonpropagate(x_axis,links(1,2))],[gt_box_nonpropagate(y_axis,links(1,1)),...
                     gt_box_nonpropagate(y_axis,links(1,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                line([gt_box_nonpropagate(x_axis,links(2,1)), gt_box_nonpropagate(x_axis,links(2,2))],[gt_box_nonpropagate(y_axis,links(2,1)),...
                 gt_box_nonpropagate(y_axis,links(2,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                line([gt_box_nonpropagate(x_axis,links(3,1)), gt_box_nonpropagate(x_axis,links(3,2))],[gt_box_nonpropagate(y_axis,links(3,1)),...
                gt_box_nonpropagate(y_axis,links(3,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
                 line([gt_box_nonpropagate(x_axis,links(4,1)), gt_box_nonpropagate(x_axis,links(4,2))],[gt_box_nonpropagate(y_axis,links(4,1)),...
                 gt_box_nonpropagate(y_axis,links(4,2))],'Color',colors_bbox_3,'LineWidth',line_size,'LineStyle','-')
            end 
    
            scatter(all_car_points(x_axis,:),all_car_points(y_axis,:),20,'blue','filled')
            ylimits = [min_z  max_z]; 
            xlimits = [min_x  max_x];

            %METHOD  A visualization of a produced bounding box before and after 
            % adopting constraints with the most improve- ment in IOU using Pandas
            qw{1} = plot(nan, '-','Color','blue','LineWidth',4);
            qw{2} = plot(nan, '-','Color','red','LineWidth',4);
            qw{3} = plot(nan, '.','Color','blue','MarkerSize',15);
            legend([qw{:}], {
                'LiDAR-derived bounding box',...
                'LiDAR-derived bounding box with inter-frame propagation',...
                'LiDAR points'}, 'location', 'best', 'FontSize', 20)
        



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
                verticalPosition = ylimits(1) + 0.5;
            else
                verticalPosition = ylimits(2) - 0.5;
            end
%             if reverse_x
%                 text(xlimits(1)+0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
%                 text(xlimits(2)-0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
%             else
%                 text(xlimits(1)+0.3, verticalPosition, 'Rear', 'HorizontalAlignment', 'left', 'FontSize', tickLabelFontSize);
%                 text(xlimits(2)-0.3, verticalPosition, 'Front', 'HorizontalAlignment', 'right', 'FontSize', tickLabelFontSize);
%             end
        end
    
       %Extend the box in 3D
       %Eval width length height non propagate
       all_dists = sort([norm(bb(:,1)- bb(:,2)),norm(bb(:,2)- bb(:,3)),norm(bb(:,1)- bb(:,3))]);
       width = all_dists(1);
       length = all_dists(2); %Hypotenus is alway the longest;

%        new_max_y = 0; %The case when extending to the ground
       height = new_max_y - new_min_y;
       all_annos.(fieldname).gt_width_nonpropagate = width;
       all_annos.(fieldname).gt_height_nonpropagate = height;
       all_annos.(fieldname).gt_length_nonpropagate = length;
   else
       all_annos.(fieldname).gt_width_nonpropagate = 0;
       all_annos.(fieldname).gt_height_nonpropagate = 0;
       all_annos.(fieldname).gt_length_nonpropagate = 0;
   end
end