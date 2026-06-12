function vp_car = get_vpcar_ransac_refit(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image)
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


    %Forming the lines
    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);
    for k = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
        if line(3) < 0
            line = -line;
        end
        formed_lines(:,k) = line/norm(line);
    end

    all_centroids = zeros(2,size(anno_sympoints_homo,2)/2);
    for i = 1:size(anno_sympoints_homo,2)/2
        all_centroids(:,i) = mean([anno_sympoints_homo(1:2,2*i-1), anno_sympoints_homo(1:2,2*i)],2);
    end


    inliers_count = zeros(1,size(anno_sympoints_homo,2)/2);
    after_refitting_inliers_count = zeros(1,size(anno_sympoints_homo,2)/2);
    before_refitting_error = zeros(1,size(anno_sympoints_homo,2)/2);
    fitting_error = zeros(1,size(anno_sympoints_homo,2)/2);
    refit_points = [];
    inliers_threshold = 2;
    refit_threshold = 2;
    horizon_line = horizon_line/norm(horizon_line);

    %Loop through all candidate points
    for i = 1:size(all_intersections_2D, 2)
        cand = all_intersections_2D(:,i);
        %Figure out the inliders
        inliers_indice = [];
        for k = 1:size(all_centroids,2)
            sym_vector = cand(1:2) -all_centroids(:,k);
            sym_vector = sym_vector/norm(sym_vector);
            pa = anno_sympoints_homo(1:2,2*k-1);
            wa = pa-all_centroids(:,k);
            da = norm(wa-(dot(wa,sym_vector)*sym_vector));
            pb = anno_sympoints_homo(1:2,2*k);
            wb = pb-all_centroids(:,k);
            db = norm(wb-(dot(wb,sym_vector)*sym_vector));
            total_residual = da+db;
            if total_residual < inliers_threshold
                inliers_count(i) = inliers_count(i) + 1;
                inliers_indice = [inliers_indice k];
                before_refitting_error(i) = before_refitting_error(i) + total_residual; %Considering inliers only
            end
        end
        before_refitting_error(i) = before_refitting_error(i)/inliers_count(i) %Average out

        %REfit if satisfied threshold
        format short
        i
        inliers_indice
        formed_lines(:,inliers_indice)
        all_intersections_2D(:,inliers_indice)

        if inliers_count(i) >= refit_threshold
            assert(abs(norm(mean(formed_lines(:,inliers_indice),2)) - 1)<1e-6)
            refit_cand = cross(mean(formed_lines(:,inliers_indice),2),horizon_line);
            if refit_cand(3) < 0
                refit_cand = -refit_cand;
            end
            refit_cand = refit_cand/norm(refit_cand);
            refit_cand_2D = (1/refit_cand(3))*refit_cand
            %Calculate new error
            for m = 1:size(inliers_indice,2)
                z = inliers_indice(m); %ONly calculate refit for the refit points
                sym_vector = refit_cand_2D(1:2) - all_centroids(:,z);
                sym_vector = sym_vector/norm(sym_vector);
                pa = anno_sympoints_homo(1:2,2*z-1);
                wa = pa-all_centroids(:,z);
                da = norm(wa-(dot(wa,sym_vector)*sym_vector));
                pb = anno_sympoints_homo(1:2,2*z);
                wb = pb-all_centroids(:,z);
                db = norm(wb-(dot(wb,sym_vector)*sym_vector));
                total_residual = da+db;
                fitting_error(i) = fitting_error(i)+total_residual;
                after_refitting_inliers_count(i) = after_refitting_inliers_count(i) + 1;
            end
            fitting_error(i) = fitting_error(i)/after_refitting_inliers_count(i);
            refit_points = [refit_points refit_cand_2D];
        else
            fitting_error(i) = 99;
            refit_points = [refit_points [0,0,1]'];
        end
    end

    before_refitting_error
    fitting_error
    inliers_count
    after_refitting_inliers_count
    [M,I] = min(fitting_error);
    [M,I]
    refit_points
    vp_car = refit_points(:,I);

    %VIsualize intersection
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