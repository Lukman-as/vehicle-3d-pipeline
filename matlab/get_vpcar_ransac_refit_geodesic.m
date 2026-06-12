function vp_car = get_vpcar_ransac_refit_geodesic(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image)
    all_intersections = zeros(3,size(anno_sympoints_homo,2)/2);
    all_intersections_2D = zeros(3,size(anno_sympoints_homo,2)/2);
    pos = 1;
    for i = 1:size(anno_sympoints_homo,2)/2
        formed_line = cross(anno_sympoints_homo(:,2*i-1), anno_sympoints_homo(:,2*i)); %Forms from a pair of point
        intersection = cross(formed_line, horizon_line);
        if intersection(3) < 0
            intersection = -intersection;
        end
        assert(intersection(3)>0)
        intersection = intersection/norm(intersection);
        all_intersections(:,pos) = intersection;
        all_intersections_2D(:,pos) = (1/intersection(3))*intersection;
        pos = pos+1;
    end

    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);
    for i = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*i-1), anno_sympoints_homo(:,2*i));
        if line(3) < 0
            line = -line;
        end
        formed_lines(:,i) = line/norm(line);
    end

    inliers_count = zeros(1,size(anno_sympoints_homo,2)/2);
    after_refitting_inliers_count = zeros(1,size(anno_sympoints_homo,2)/2);
    before_refitting_error = zeros(1,size(anno_sympoints_homo,2)/2);
    fitting_error = [];
    refit_points = [];
    inliers_threshold = 0.1;
    refit_threshold = 2;
    %Loop through all candidate points
    for i = 1:size(all_intersections, 2)
        cand = all_intersections(:,i);
        %Figure out the inliders
        inliers_indice = [];
        for k = 1:size(formed_lines,2)
            geo_dist = acos(dot(all_intersections(:,i),all_intersections(:,k)))
            if geo_dist < inliers_threshold
                inliers_count(i) = inliers_count(i) + 1;
                inliers_indice = [inliers_indice k];
            end
            before_refitting_error(i) = before_refitting_error(i) + geo_dist;
        end

        %REfit if satisfied threshold
        format short
        all_intersections(:,i)
        inliers_indice
        all_intersections(:,inliers_indice)
        mean(all_intersections(:,inliers_indice),2)
        if inliers_count(i) >= refit_threshold
            refit_cand = mean(all_intersections(:,inliers_indice),2);
            refit_cand_2D = (1/refit_cand(3))*refit_cand;
            %Calculate new error
            sum = 0
            for z = 1:size(formed_lines,2)
                geo_dist = acos(dot(refit_cand,all_intersections(:,z)));
                sum = sum+geo_dist;
                if geo_dist < inliers_threshold
                    after_refitting_inliers_count(i) = after_refitting_inliers_count(i) + 1;
                end
            end
            fitting_error = [fitting_error sum];
            refit_points = [refit_points refit_cand_2D];
        end
    end
    

    before_refitting_error
    fitting_error
    inliers_count
    after_refitting_inliers_count
    [M,I] = min(fitting_error);
    vp_car = refit_points(:,I)

    assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line')
end