function all_annos = get_bbox_minimum(all_annos, annotated_car_id,ptCloud_above)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    loc = mean(all_annos.(fieldname).corners_bbox_world,2);
    max_dim = 8/2;
    bounds_x = [loc(1) - max_dim, loc(1) + max_dim];
    bounds_y = [loc(2) - max_dim, loc(2) + max_dim];
    bounds_z = [loc(3) - max_dim, loc(3) + max_dim];

    ptCloud_nearby = ptCloud_above(1:3,ptCloud_above(1,:) > bounds_x(1) & ptCloud_above(1,:) < bounds_x(2)...
        & ptCloud_above(2,:) > bounds_y(1) & ptCloud_above(2,:) < bounds_y(2)...
        & ptCloud_above(3,:) > bounds_z(1) & ptCloud_above(3,:) < bounds_z(2));

    R_velo_world = [[0 -1 0]' [0 0 -1]' [1 0 0]'];
    
    ptCloud_nearby_velo = R_velo_world * ptCloud_nearby;
        
    ptCloud_nearby_pc = pointCloud(ptCloud_nearby_velo');
    model = pcfitcuboid(ptCloud_nearby_pc);
    corner_points_velo = getCornerPoints(model)';
    corner_points = inv(R_velo_world)*corner_points_velo;
    
    assert(abs(model.Orientation(1)) < 1e-5, 'Only rotation y');
    assert(abs(model.Orientation(2)) < 1e-5, 'Only rotation y');
    angle = model.Orientation(3) + 90; %Rotation z is rotation y

    %Gotta rearrange these corner_points to match the front and back
    all_annos.(fieldname).corners_bbox_minimum = corner_points;
    all_annos.(fieldname).heading_angle_min_box = angle;

    
    if false
        figure
        hold on
        pcshow(ptCloud_above(1:3,:)', 'cyan','MarkerSize',5);
        pcshow(ptCloud_nearby(1:3,:)', 'green','MarkerSize',5);
        pcshow(corner_points(1:3,:)', 'yellow','MarkerSize',10);
        pcshow(all_annos.(fieldname).corners_bbox_world(1:3,:)', 'red','MarkerSize',10);
    end

end