function vp_car = get_vpcar_pizlo(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image)
    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);
    all_intersections = zeros(3,size(anno_sympoints_homo,2)/2);
    for k = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
        if line(3) < 0
            line = -line;
        end
        formed_lines(:,k) = line/norm(line);
        all_intersections(:,k) = cross(formed_lines(:,k), horizon_line);
    end

    B = formed_lines';
    %Different scenarios
    v0 = [-horizon_line(2) horizon_line(1) 0]';
    v1 = [0 -horizon_line(3) horizon_line(2)]';
    if horizon_line(2) == 0
        v1 = [-horizon_line(3) 0 horizon_line(1)]';
    end
    if horizon_line(1) == 0 && horizon_line(2) == 0 
        v0 = [1 0 0]';
        v1 = [0 1 0]';
    end

    assert (abs(dot(v0, horizon_line)) < 1e-6);
    assert (abs(dot(v1, horizon_line)) < 1e-6);

    lambda = inv((B*v1)'*(B*v1))*(B*v1)'*(-B*v0);
    vp_car = v0 + lambda * v1;
    vp_car = vp_car/vp_car(3);
    assert(abs(dot(vp_car,horizon_line)) < 1e-5, 'VP car gotta be on horizon line');


    %VIsualize intersection
    if false
        figure
        hold on
        imshow(image)
        hold on
        scatter(anno_sympoints_homo(1,:),anno_sympoints_homo(2,:),10,'r','filled');
%         for i = 1:size(all_intersections,2)
%             a_cross = all_intersections(:,i);
%             a_cross = (1/a_cross(3))*a_cross;
%             assert(abs(a_cross(3)-1)<1e-6);
%             scatter(a_cross(1,:),a_cross(2,:),180,'green','filled');
%         end

        for k = 1:size(anno_sympoints_homo,2)/2
            line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
            if line(3) < 0
                line = -line;
            end
            formed_lines(:,k) = line/norm(line);
            all_intersections(:,k) = cross(formed_lines(:,k), horizon_line);
            a_cross = all_intersections(:,k);
            a_cross = (1/a_cross(3))*a_cross;
            assert(abs(a_cross(3)-1)<1e-6);
            scatter(a_cross(1,:),a_cross(2,:),180,'green','filled');
            apoint = anno_sympoints_homo(:,2*k);

            line([apoint(1), a_cross(1)], [apoint(2), a_cross(2)],'Color','yellow','LineWidth',2,'LineStyle','--'); 
        end

        scatter(vp_car(1,:),vp_car(2,:),200,'blue','filled');
        %Vanishing line plotting
        vp_points_2D
        horizon_line_2D_vec = vp_points_2D(:,3)-vp_points_2D(:,1)
        vp_point_vis_1 = vp_points_2D(:,3) - 5*horizon_line_2D_vec
        vp_point_vis_2 = vp_points_2D(:,3) + 5*horizon_line_2D_vec
        line([vp_point_vis_1(1), vp_point_vis_2(1)],[vp_point_vis_1(2),...
                vp_point_vis_2(2)],'Color','red','LineWidth',20)

        axis equal
%         axis on
%         xlabel('X')
%         ylabel('Y') 
    end







%     if true
%         figure;
%         hold on
% %         pcshow([0,0,0],'red','MarkerSize',500)
%         pcshow([formed_lines(1,:)',formed_lines(2,:)',formed_lines(3,:)'],'yellow','MarkerSize',50)
%         xlabel('X');
%         ylabel('Y');
%         zlabel('Z');
%         hold off;
%     end
% 
%     horizon_line = horizon_line/norm(horizon_line);
%     fitted_line = mean(formed_lines,2);
%     vp_car = cross(fitted_line,horizon_line);
%     if vp_car(3) < 0
%         vp_car = -vp_car;
%     end
%     vp_car = vp_car/norm(vp_car);
%     vp_car = 1/(vp_car(3))*vp_car
%     abs(dot(vp_car,horizon_line))
%     assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line');
end