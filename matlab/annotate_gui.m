function annotate_gui(all_cars,file_name, image)
    edges_2D = [
         1 3;
         1 2;
         2 4;
         3 4;
    ];
    all_cars = all_cars(1:1,:)
    global annotations 
    annotations = zeros(2, 14*size(all_cars,1))
    global this_figure
    this_figure = figure
    imshow(image)
    for k = 1:size(all_cars,1)
        acar = all_cars(k,:);
        bbox_2D = [acar(1) acar(1) acar(3) acar(3); 
            acar(2) acar(4) acar(2) acar(4);
                    ones(1,4)];
        hold on
        scatter(bbox_2D(1,:),bbox_2D(2,:),30,'green','filled'); %Plot the sample 2D
        for i = 1:size(edges_2D,1)
            x_coor = [bbox_2D(1, edges_2D(i,1)) bbox_2D(1, edges_2D(i,2))];
            y_coor = [bbox_2D(2, edges_2D(i,1)) bbox_2D(2, edges_2D(i,2))];
            line(x_coor, y_coor,'Color','red','LineWidth',1);
        end
        global still_moving
        still_moving = true
    
        for j = 1:14
            roi = drawpoint('Color','r');
            annotations(:,14*(k-1)+j) = [roi.Position(1),roi.Position(2)]';
            roi.Label = sprintf('%.0f',j);
            addlistener(roi,'ROIMoved',@(src,eventdata)move_points_callback(src,eventdata,j,k,this_figure));
        end
    
        while still_moving
            uicontrol('Position',[20 75 60 20],'String','Visual check','Callback','uiresume(this_figure)');
            uiwait(this_figure)
            delete(findobj(gca, 'type', 'scatter'));
            scatter(annotations(1,14*(k-1)+1:14*(k-1)+14),annotations(2,14*(k-1)+1:14*(k-1)+14),10,'green','filled');
            c = uicontrol('String','Finished?','Callback', @(src,eventdata)finish_moving_callback(src,eventdata,this_figure));
        end
    
        %Clean up the unknown points
        for h = 14*(k-1)+1:14*(k-1)+14
            tp_x = annotations(1,h);
            tp_y = annotations(2,h);
            if tp_x < min(bbox_2D(1,:)) || tp_x > max(bbox_2D(1,:)) || ...
                 tp_y < min(bbox_2D(2,:)) || tp_y > max(bbox_2D(2,:))
                annotations(:,h) = 0;
            end
        end
        disp('Continue to annotate the next car');
    end
    disp('Finished annotations for all cars');

%     save(sprintf("data\\annotations\\%s.mat",file_name), 'annotations');
end