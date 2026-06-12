function vp_car = get_vpcar_avg_intersects(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image)
    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);
    for k = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
        if line(3) < 0
            line = -line;
        end
        formed_lines(:,k) = line/norm(line);
    end
    
    horizon_line = horizon_line/norm(horizon_line);
    if horizon_line(3) < 0
        horizon_line = -horizon_line;
    end

    all_intersects = zeros(3,size(formed_lines,2));
    for k = 1:size(formed_lines,2)
        line = cross(formed_lines(:,k), horizon_line);
        if line(3) < 0
            line = -line;
        end
        all_intersects(:,k) = line/norm(line);
    end

    all_intersects
    R_horizon = get_rotation_from_two_vecs(horizon_line, [0 0 1]');
    'all_intersects_on_horizon';;
    all_intersects_on_horizon = R_horizon*all_intersects;

    %Correct based on the first vector
    for k = 1:size(all_intersects_on_horizon,2)
%         if any(diff(sign([all_intersects_on_horizon(1,k),all_intersects_on_horizon(1,1)]))) ...
%             || any(diff(sign([all_intersects_on_horizon(2,k),all_intersects_on_horizon(2,1)]))) 
%             all_intersects_on_horizon(:,k) = -all_intersects_on_horizon(:,k);
%         end
        
        angle1 = rad2deg(acos(dot(all_intersects_on_horizon(:,k),all_intersects_on_horizon(:,1))));
        angle2 = rad2deg(acos(dot(-all_intersects_on_horizon(:,k),all_intersects_on_horizon(:,1))));
        if angle1 > angle2
            all_intersects_on_horizon(:,k) = -all_intersects_on_horizon(:,k); %Use the one with smaller angle or closer
        end   
    end

    if false
        figure
        hold on
        target = 1;
        scatter(all_intersects_on_horizon(1,target),all_intersects_on_horizon(2,target),'blue',"filled");
        scatter(-all_intersects_on_horizon(1,target),-all_intersects_on_horizon(2,target),'green',"filled");
        plot([all_intersects_on_horizon(1,target), -all_intersects_on_horizon(1,target)],...
            [all_intersects_on_horizon(2,target),-all_intersects_on_horizon(2,target)],'-o')
        target = 2;
        scatter(all_intersects_on_horizon(1,target),all_intersects_on_horizon(2,target),'blue',"filled");
        scatter(-all_intersects_on_horizon(1,target),-all_intersects_on_horizon(2,target),'green',"filled");
        plot([all_intersects_on_horizon(1,target), -all_intersects_on_horizon(1,target)],...
            [all_intersects_on_horizon(2,target),-all_intersects_on_horizon(2,target)],'-o') 
        target = 3;
        scatter(all_intersects_on_horizon(1,target),all_intersects_on_horizon(2,target),'blue',"filled");
        scatter(-all_intersects_on_horizon(1,target),-all_intersects_on_horizon(2,target),'green',"filled");
        plot([all_intersects_on_horizon(1,target), -all_intersects_on_horizon(1,target)],...
            [all_intersects_on_horizon(2,target),-all_intersects_on_horizon(2,target)],'-o') 
        scatter(0,0,100,'red',"filled");
        axis equal;
        axis on;
    end

    all_intersects = inv(R_horizon)*all_intersects_on_horizon; %Inverse back
    vp_car =  mean(all_intersects,2);
%     vp_car =  all_intersects(:,3);
    vp_car = vp_car./vp_car(3);
    assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line');
end