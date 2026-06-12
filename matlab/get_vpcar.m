function vp_car = get_vpcar(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image)
    all_intersections = zeros(3,size(anno_sympoints_homo,2)/2);
    all_angles = zeros(1,size(anno_sympoints_homo,2)/2);
    for i = 1:size(anno_sympoints_homo,2)/2
        formed_line = cross(anno_sympoints_homo(:,2*i-1), anno_sympoints_homo(:,2*i)); %Forms from a pair of point
        intersection = cross(formed_line, horizon_line);
        intersection = intersection/intersection(3); %converting each homo to euclidean
        all_intersections(:,i) = intersection;
        sym_norm = inv(k_matrix*R)*intersection;
        sym_norm = sym_norm/norm(sym_norm);
        all_angles(:,i) = acos(dot(sym_norm,[1,0,0]'));
    end

    vp_car = mean(all_intersections, 2); %Taking the mean of all points
    dot(vp_car,horizon_line)

    if false
        figure
        hold on
        imshow(image)
        hold on
        scatter(anno_sympoints_homo(1,:),anno_sympoints_homo(2,:),10,'r','filled');
        scatter(vp_car(1,:),vp_car(2,:),180,'yellow','filled');
        for i = 1:size(all_intersections,2)
            a_cross = all_intersections(:,i);
            a_cross = (1/a_cross(3))*a_cross;
            assert(abs(a_cross(3)-1)<1e-6);
            scatter(a_cross(1,:),a_cross(2,:),180,'green','filled');
        end
        scatter(vp_points_2D(1,1),vp_points_2D(2,1),100,'red','filled');
        scatter(vp_points_2D(1,3),vp_points_2D(2,3),50,'blue','filled');
        line([vp_points_2D(1,1), vp_points_2D(1,3)],[vp_points_2D(2,1),...
            vp_points_2D(2,3)],'Color','g','LineWidth',2,'LineStyle','--')

        for i = 1:size(anno_sympoints_homo,2)/2
            line([anno_sympoints_homo(1,2*i), vp_car(1,:)], [anno_sympoints_homo(2,2*i), vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
        end
        axis equal
        axis on
        xlabel('X')
        ylabel('Y') 
    end


    assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line')
end