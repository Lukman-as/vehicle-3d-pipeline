function vp_car = get_vp_ransac_combo(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image)
    %Forming the lines
    formed_lines = zeros(3,size(anno_sympoints_homo,2)/2);
    for k = 1:size(anno_sympoints_homo,2)/2
        line = cross(anno_sympoints_homo(:,2*k-1), anno_sympoints_homo(:,2*k));
        if line(3) < 0
            line = -line;
        end
        formed_lines(:,k) = line/norm(line);
    end

    all_combos = nchoosek(1:size(formed_lines,2),2)';

    all_centroids = zeros(2,size(anno_sympoints_homo,2)/2);
    for i = 1:size(anno_sympoints_homo,2)/2
        all_centroids(:,i) = mean([anno_sympoints_homo(1:2,2*i-1), anno_sympoints_homo(1:2,2*i)],2);
    end

    inliers_count = zeros(1,size(all_combos,2));
    after_refitting_inliers_count = zeros(1,size(all_combos,2));
    before_refitting_error = zeros(1,size(all_combos,2));
    fitting_error = zeros(1,size(all_combos,2));
    refit_points = [];
    inliers_threshold = 2;
    refit_threshold = 3;
    horizon_line = horizon_line/norm(horizon_line);

    %Loop through all combos
    for i = 1:size(all_combos, 2)
        l1 = formed_lines(:,all_combos(1,i));
        l2 = formed_lines(:,all_combos(2,i));
        cand = cross(mean([l1,l2],2),horizon_line);
        if cand(3) < 0
            cand = -cand;
        end
        cand = cand/norm(cand);
        cand = (1/cand(3))*cand;
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

        %REfit if satisfied threshold
        inliers_indice
        formed_lines(:,inliers_indice)

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
    assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line')
end