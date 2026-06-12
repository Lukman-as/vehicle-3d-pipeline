function vp_car = get_vpcar_avg_lines(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image)
    anno_sympoints_homo
    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);
    for k = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
%         if line(3) < 0
%             line = -line;
%         end
        formed_lines(:,k) = line/norm(line);
    end
    
    formed_lines

    if false
        figure;
        hold on
%         pcshow([0,0,0],'red','MarkerSize',500)
        pcshow([formed_lines(1,:)',formed_lines(2,:)',formed_lines(3,:)'],'yellow','MarkerSize',50)
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        hold off;
    end

    horizon_line = horizon_line/norm(horizon_line);
    fitted_line = mean(formed_lines,2);
    vp_car = cross(fitted_line,horizon_line);
%     if vp_car(3) < 0
%         vp_car = -vp_car;
%     end
    vp_car = vp_car/norm(vp_car);
    vp_car = vp_car/vp_car(3)
    abs(dot(vp_car,horizon_line))
    assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line');
end