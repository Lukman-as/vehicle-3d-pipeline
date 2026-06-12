function predicted_corners = get_bbox_from_points(all_3d_points_tire_tire_coor, points_only, has_mirror, mirror_one_side, w_unit)
    max_x = max(all_3d_points_tire_tire_coor(1,:));
    min_x = min(all_3d_points_tire_tire_coor(1,:));
    max_y = max(all_3d_points_tire_tire_coor(2,:));
    min_y = min(all_3d_points_tire_tire_coor(2,:));
    max_z = max(all_3d_points_tire_tire_coor(3,:));
    min_z = min(all_3d_points_tire_tire_coor(3,:));

%   TODO:if no mirrors then add the average width here, this is only
%   affecting the boxes.
    tolerance = 1e-8;
    if ~has_mirror && ~points_only
        %IF all points are within the extremal planes
        assert(max_z-min_z-w_unit > -tolerance, 'Points at extremities, cannot be smaller than the w_unit')
        if abs(max_z - min_z - w_unit) < tolerance
            max_z = max_z + mirror_one_side;
            min_z = min_z - mirror_one_side;
        else %Else you have to find true edge, w_unit is just dimension
            midpoint = (max_z+min_z)/2;
            min_z = min(midpoint - w_unit/2 - mirror_one_side,min_z);
            max_z = max(midpoint + w_unit/2 + mirror_one_side,max_z); %Still gotta include all the points
        end
    end

    predicted_corners = [
    min_x,min_x,max_x,max_x,min_x,min_x,max_x,max_x;
    max_y,max_y,max_y,max_y, min_y, min_y,min_y,min_y;
    min_z,max_z,max_z,min_z,min_z,max_z,max_z,min_z;
    ];

    %IMPORTANT the bottom of the bounding box to be on the ground to be on ground and valid car
%     assert(abs(max_y)<1e-6);
    predicted_corners = [predicted_corners; ones(1,size(predicted_corners,2))];
    vec1 = predicted_corners(1:3,1) - predicted_corners(1:3,2);
    vec2 = predicted_corners(1:3,3) - predicted_corners(1:3,2);
    assert(abs(dot(vec1/norm(vec1),vec2/norm(vec2)))<1e-6);
end