function vp_car = get_vpcar_collins_weiss(anno_sympoints_homo, horizon_line,k_matrix,R, image)
%     translation = [-size(image,2)/2, -size(image,1)/2, 0]';
%     w_selected = mean([size(image,2), size(image,1)])/2;
%     anno_sympoints_homo = anno_sympoints_homo + translation;
%     anno_sympoints_homo = anno_sympoints_homo * w_selected;
    if horizon_line(3) < 0
        horizon_line = -horizon_line;
    end
    horizon_line = horizon_line/norm(horizon_line);

    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);
    for k = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
        if line(3) < 0
            line = -line;
        end
        formed_lines(:,k) = line/norm(line);
%         formed_lines(:,k) = line;
    end

    B = formed_lines';
    all_moments = zeros(3,3);
    for k = 1:size(B,1)
        all_moments = all_moments + B(k,:)'*B(k,:);
    end

    C = (1/size(B,1))*all_moments;
    [V,D] = eig(C);
    u_direction = V(:,1);

%     u_direction = cross(formed_lines(:,2), horizon_line);

    if u_direction(3) < 0
        u_direction = -u_direction;
    end

    'angle between'
%     rad2deg(acos(dot(u_direction,formed_lines(:,1))))
    rad2deg(acos(dot(u_direction,horizon_line)))
%     rad2deg(acos(dot(formed_lines(:,1),horizon_line)))
%     rad2deg(acos(dot(formed_lines(:,2),horizon_line)))
%     rad2deg(acos(dot(formed_lines(:,2),formed_lines(:,1))))

    %Find the projection on to this plane of the horizon - so it is like
    %nearest point to the vanishing point but on the horizon
    assert(abs(norm(horizon_line)- 1)<1e6)
    assert(abs(norm(u_direction)- 1)<1e6)
    proj_u = dot(u_direction,horizon_line)*horizon_line;
    vp_car = u_direction - proj_u;
    vp_car = vp_car/norm(vp_car);
    'angle between vpcar and u_direction'
    norm(vp_car)
    
    u_direction
    vp_car

    rad2deg(acos(dot(u_direction,vp_car)))

       
%     vp_car = cross(cross(u_direction,horizon_line),horizon_line);
%     vp_car = u_direction;
    vp_car = vp_car./vp_car(3)
    

    
%     %What if
%     'd_direction'
%     u_direction = cross(formed_lines(:,1), horizon_line)
%     d_direction = inv(k_matrix)*u_direction
% 
%     'horizon'
%     horizon = cross(R(:,1),R(:,3))
% 
%     'intersect'
%     intersect = cross(d_direction, horizon)
%     vp_car = k_matrix*intersect;



    
%     u_direction = formed_lines(:,1)
%     vp_car = cross(u_direction,horizon_line);
%     vp_car = vp_car./vp_car(3)


%     'V(:,1)'
%     V(:,1)
% 
%     'u_direction'
%     u_direction
% 
%     'angle between'
%     angle = dot(V(:,1),u_direction)


%     vp_car_2= cross(V(:,1),horizon_line);
%     vp_car_2 = vp_car_2./vp_car_2(3)
    assert(vp_car(3)==1)
end