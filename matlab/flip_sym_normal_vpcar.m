function sym_normal_unit = flip_sym_normal_vpcar(k_matrix,R,all_annos,annotated_car_id,vp_car,image)
    % Flip if need flip
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    one_anno = all_annos.(fieldname);
    %Get the centroid of it
    centroid = mean([one_anno.ex(1:2,1), one_anno.ex(1:2,2)], 2);

    %Find rectified symmetry points
    sym_line = vp_car(1:2) - centroid;%Symmetry line
    centroid_ex1_vec = one_anno.ex(1:2,1) - centroid;
    centroid_ex2_vec = one_anno.ex(1:2,2) - centroid;
    ex1_rec = centroid + dot(centroid_ex1_vec, sym_line)/norm(sym_line)...
        *(sym_line/norm(sym_line));
    ex2_rec = centroid + dot(centroid_ex2_vec, sym_line)/norm(sym_line)...
        *(sym_line/norm(sym_line));

    d1 = norm(vp_car(1:2) - ex1_rec); %Use the latest vp_car, not the one in the anno
    d2 = norm(ex2_rec - ex1_rec);
    d3 = norm(ex2_rec - vp_car(1:2));
%     if true
%         figure
%         %hold on
%         imshow(image)
%         hold on
%         scatter(one_anno.ex(1,1),one_anno.ex(2,1),40,'r','filled');
%         scatter(one_anno.ex(1,2),one_anno.ex(2,2),40,'g','filled');
%         scatter(ex1_rec(1),ex1_rec(2),20,'r','filled'); 
%         scatter(ex2_rec(1),ex2_rec(2),20,'g','filled'); 
%         scatter(centroid(1,:),centroid(2,:),20,'blue','filled'); 
%         line([centroid(1,:) vp_car(1)], [centroid(2,:) vp_car(2)],'Color','red','LineWidth',1);
%         axis equal
%         axis on
%         xlabel('X')
%         ylabel('Y')
%     end

    if abs(d1+d2-d3) < 1e-3
        need_flip = 1;
    else
        need_flip = 0;
    end

%     'need flip'
%     need_flip
%     abs(d1+d2-d3)

    if need_flip
        sym_normal = -inv(k_matrix*R)*vp_car;
    else
        sym_normal = inv(k_matrix*R)*vp_car;
    end
    sym_normal_unit = sym_normal/norm(sym_normal);
end