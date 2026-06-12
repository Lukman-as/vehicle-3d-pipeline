function  mounting_height_gt = get_height_poly(image,target,origin_2D, k_matrix, R,polygon_data)
    polygon_data_cells = num2cell(polygon_data(:,1:8),2);
    %Findout the origin_2D is located inside what polygon
    poly_results = cellfun(@(polygon) coop_polygonmatch(origin_2D(1),origin_2D(2),polygon),polygon_data_cells);
    withins = find(poly_results);

    %In case no within then find the closest
    find_closest = "No"
    if size(withins,1) == 0
         poly_results = cellfun(@(polygon) p_poly_dist(origin_2D(1), origin_2D(2), polygon),polygon_data_cells);
         [C,withins] = min(poly_results);
         find_closest = "Yes"
    end

    if true
        figure
        hold on
        imshow(image);
        hold on
        scatter([origin_2D(1)],[origin_2D(2)],10,'red','filled')
        matched = polygon_data(withins,1:8)
        scatter(matched(1:4),matched(5:8),10,'green','filled');
        text(origin_2D(1)+3,origin_2D(2)-3,target,'Color','yellow','FontSize',10);
        text(origin_2D(1)+6,origin_2D(2)-3,find_closest,'Color','yellow','FontSize',10);
     end
   

    %Use the equation of that polygon plane to find the heigh
    poly_plane = polygon_data(withins,9:12);
    lambda_poly = -poly_plane(4)/dot(inv(k_matrix*R)*origin_2D,poly_plane(1:3));
    point3D = lambda_poly*inv(k_matrix*R)*origin_2D;
    mounting_height_gt = point3D(2);
end