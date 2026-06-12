function vp_car = get_vpcar_avg_intersects_v2(anno_sympoints_homo, horizon_line)
    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);

    %Forming the symmetry line
    for k = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
        if line(3) < 0
            line = -line;
        end
        formed_lines(:,k) = line/norm(line);
    end
    
    %Normalize the horizon
    horizon_line = horizon_line/norm(horizon_line);
    if horizon_line(3) < 0
        horizon_line = -horizon_line;
    end

    %Forming the unit vectors, enforcing z > 0
    all_intersects = zeros(3,size(formed_lines,2));
    for k = 1:size(formed_lines,2)
        line = cross(formed_lines(:,k), horizon_line);
        if line(3) < 0
            line = -line;
        end
        all_intersects(:,k) = line/norm(line);
    end

%     'mean inter before'
    vp_car = mean(all_intersects,2);

    %Correct based on the angle
    all_valid = false;
    while ~all_valid
        for k = 1:size(all_intersects,2)
            angle = rad2deg(acos(dot(all_intersects(:,k),vp_car)));
            if angle > 90
                all_intersects(:,k) = -all_intersects(:,k); %Flip if more than 90
            end
            %Calculate new mean
            vp_car = mean(all_intersects,2);
        end
        %Recheck 
        all_valid = true;
        for k = 1:size(all_intersects,2)
            angle = rad2deg(acos(dot(all_intersects(:,k),vp_car)));
            if angle > 90
                all_valid = false;
                break
            end
        end
    end
    
%     'mean inter after'
%     vp_car
    
    %Assertion everything less than 90
    for k = 1:size(all_intersects,2)
        angle = rad2deg(acos(dot(all_intersects(:,k),vp_car)));
        assert(angle <= 90, 'everything has to be less than 90');
    end

    vp_car = vp_car./vp_car(3);
    assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line');
end