function mounting_height_gt = get_height_grid(image,target, origin_2D, k_matrix, R,polygon_data)
    % Find intersection successfully
    poly_plane = polygon_data(:,9:12);
    lambda_poly = -poly_plane(:,4)./(poly_plane(:,1:3)*inv(k_matrix*R)*origin_2D);
    point3D = lambda_poly*(inv(k_matrix*R)*origin_2D)';

    %Find the point within the frame for the origin_2D
%     withins = point3D(:,1) >= polygon_data(:,1) & point3D(:,1) < polygon_data(:,3) & ... 
%     point3D(:,3) >= polygon_data(:,5) & point3D(:,3) < polygon_data(:,6);
    centers = [(polygon_data(:,1)+polygon_data(:,3))/2,(polygon_data(:,5)+polygon_data(:,6))/2];
    point3D_xz = [point3D(:,1), point3D(:,3)];
    dist_centers = vecnorm(centers-point3D_xz,2,2); %Calculate distance to center
    [M,withins] = min(dist_centers);
%     dot(point3D(withins,:),polygon_data(withins,9:11));
%     polygon_data(withins,12)
    assert(abs(dot(point3D(withins,:),polygon_data(withins,9:11))+polygon_data(withins,12)) < 1e-3);
    mounting_height_gt = double(point3D(withins,2));
    if false
        all_corners_3D = [polygon_data(withins,1) polygon_data(withins,1)  polygon_data(withins,3) polygon_data(withins,3); 
                            ones(1,4)*mounting_height_gt;
                            polygon_data(withins,5) polygon_data(withins,6)  polygon_data(withins,5) polygon_data(withins,6); ]
        all_corners_2D = k_matrix*R*all_corners_3D;
        all_corners_2D = all_corners_2D./all_corners_2D(3,:)
        figure
        hold on
        imshow(image);
        hold on
        scatter([origin_2D(1)],[origin_2D(2)],10,'red','filled')
        scatter(all_corners_2D(1,:),all_corners_2D(2,:),10,'green','filled');
        text(origin_2D(1)+20,origin_2D(2)-3,target,'Color','green','FontSize',8);
        text(origin_2D(1)+20,origin_2D(2)-30,sprintf("%.2f",M),'Color','green','FontSize',8);
    end
end