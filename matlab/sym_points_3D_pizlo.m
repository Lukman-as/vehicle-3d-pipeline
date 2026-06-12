function [points_behind, all_annos] = sym_points_3D_pizlo(k_matrix,R,annotated_car_id,all_annos,image)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    %Clear field to develop repetitively
    if all_annos.(fieldname).tire_both_sides
        fields_to_clear = ["nonex_sep", "nonex_3D","ex_sep","ex_3D"];
    else
        fields_to_clear = ["nonex_sep", "nonex_3D"];
    end

    for i = 1:size(fields_to_clear,2)
        if isfield(all_annos.(fieldname),fields_to_clear(i))
            all_annos.(fieldname) = rmfield(all_annos.(fieldname),fields_to_clear(i));
        end
    end

    anno_sympoints = [];
    %Adding points to the anno_sympoints
    if all_annos.(fieldname).tire_both_sides
        for i = 1:size(all_annos.(fieldname).ex,2)/2
            anno_sympoints = [anno_sympoints all_annos.(fieldname).ex(:,2*i-1) all_annos.(fieldname).ex(:,2*i)];
        end
    end
    for i = 1:size(all_annos.(fieldname).nonex,2)/2
        anno_sympoints = [anno_sympoints all_annos.(fieldname).nonex(:,2*i-1) all_annos.(fieldname).nonex(:,2*i)];
    end
    
    all_annos.(fieldname).nonex_3D = [];
    all_annos.(fieldname).nonex_sep = [];

    if all_annos.(fieldname).tire_both_sides
        count_ex_pairs = size(all_annos.(fieldname).ex,2)/2;
    else
        count_ex_pairs = 0;
    end

    for k = 1: size(anno_sympoints,2)/2
        sym_vis = anno_sympoints(1:2,2*k-1);
        sym_nvis = anno_sympoints(1:2,2*k);
        %Get the centroid of it
         centroid = mean([sym_vis, sym_nvis], 2);
        %Find rectified symmetry points
         sym_line = all_annos.(fieldname).vp_car(1:2) - centroid;%Symmetry line
        centroid_vis_vec = sym_vis - centroid;
        centroid_nvis_vec = sym_nvis - centroid;
        sym_vis_rec = centroid + dot(centroid_vis_vec, sym_line)/norm(sym_line)...
            *(sym_line/norm(sym_line));
        sym_nvis_rec = centroid + dot(centroid_nvis_vec, sym_line)/norm(sym_line)...
            *(sym_line/norm(sym_line));

        %Visualization of rectifying
        if false
            figure
            %hold on
            imshow(image)
            hold on
            scatter(sym_vis(1,:),sym_vis(2,:),40,'r','filled');
            scatter(sym_nvis(1,:),sym_nvis(2,:),40,'r','filled'); 
            scatter(sym_nvis_rec(1,:),sym_nvis_rec(2,:),20,'green','filled'); 
            scatter(sym_vis_rec(1,:),sym_vis_rec(2,:),20,'green','filled'); 
            scatter(centroid(1,:),centroid(2,:),20,'blue','filled'); 
%             line([sym_nvis_rec(1,:), all_annos.(fieldname).vp_car(1,:)], ...
%                 [sym_nvis_rec(2,:), all_annos.(fieldname).vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
            axis equal
            axis on
            xlabel('X')
            ylabel('Y')
        end
    
        vec= sym_vis_rec-sym_nvis_rec;
        assert(abs(vec(2)/vec(1)-sym_line(2)/sym_line(1))<1e-5);

        sym_vis_rec_aug = [sym_vis_rec; 1]; %Augment it
        sym_nvis_rec_aug = [sym_nvis_rec; 1];
        

        M = k_matrix*R;
        a = all_annos.(fieldname).sym_normal_unit'*inv(M)*sym_vis_rec_aug;
        b = all_annos.(fieldname).sym_normal_unit'*inv(M)*sym_nvis_rec_aug;
        c = norm(sym_vis_rec_aug(1:2)-all_annos.(fieldname).vp_car(1:2));
        d = -norm(sym_nvis_rec_aug(1:2)-all_annos.(fieldname).vp_car(1:2));
        'A,B,X';
        A = [a b; c d];
        B = [-2*all_annos.(fieldname).ds;0];
        X = linsolve(A,B);

        %Extracting solutions
        lambda_vis = double(X(1));
        lambda_nvis = double(X(2));
        sym_vis_3D = lambda_vis*inv(M)*sym_vis_rec_aug;
        sym_nvis_3D = lambda_nvis*inv(M)*sym_nvis_rec_aug;

        %Condition 1 mean of these two points has to be on the symmetry plane
        mid = (sym_vis_3D+sym_nvis_3D)/2;
%         abs(dot(mid, all_annos.(fieldname).sym_normal_unit)+all_annos.(fieldname).ds) 
        assert(abs(dot(mid, all_annos.(fieldname).sym_normal_unit)+all_annos.(fieldname).ds) < 1e-6);
        
        %Condition 2 sym line has to be normal to the sym plane
        assert(abs(dot(all_annos.(fieldname).sym_normal_unit, sym_nvis_3D-sym_vis_3D) ...
            - norm(sym_nvis_3D-sym_vis_3D)) < 1e-6 || abs(dot(all_annos.(fieldname).sym_normal_unit, sym_vis_3D-sym_nvis_3D) ...
            - norm(sym_nvis_3D-sym_vis_3D)) <1e-6);

        
        %Save ex_3D from this optimization 
        if ~isfield(all_annos.(fieldname),'ex_3D')
            all_annos.(fieldname).ex_3D = [];
            all_annos.(fieldname).ex_sep = [];
        end
        if count_ex_pairs > 0
            all_annos.(fieldname).ex_3D = [all_annos.(fieldname).ex_3D sym_vis_3D sym_nvis_3D];
            all_annos.(fieldname).ex_sep = [all_annos.(fieldname).ex_sep norm(sym_vis_3D-sym_nvis_3D)];
            count_ex_pairs = count_ex_pairs - 1;
        else
            %Save non ex
            all_annos.(fieldname).nonex_3D = [all_annos.(fieldname).nonex_3D sym_vis_3D sym_nvis_3D];
            all_annos.(fieldname).nonex_sep = [all_annos.(fieldname).nonex_sep norm(sym_vis_3D-sym_nvis_3D)];
        end
    end
    assert(size(all_annos.(fieldname).ex,2)==size(all_annos.(fieldname).ex_3D,2));
    assert(size(all_annos.(fieldname).nonex,2)==size(all_annos.(fieldname).nonex_3D,2));


    %% Check if any points behind camera
    points_behind = false;
    %Check the behind car cam and remove if necessary


    %% Just for HW7 only
%     for k=1:size(all_annos.(fieldname).ex_3D,2)/2
%        if all_annos.(fieldname).ex_3D(3,2*k-1) < 0
%           all_annos.(fieldname).ex_3D(:,2*k-1) = zeros(3,1); %Clean up those points
%           all_annos.(fieldname).ex(:,2*k-1) = zeros(3,1);
%        end
%        if all_annos.(fieldname).ex_3D(3,2*k) < 0
%           all_annos.(fieldname).ex_3D(:,2*k) = zeros(3,1); %Clean up those points
%           all_annos.(fieldname).ex(:,2*k) = zeros(3,1);
%        end
%     end
%     for k = 1:size(all_annos.(fieldname).nonex_3D,2)/2
%        if all_annos.(fieldname).nonex_3D(3,2*k-1) < 0
%           all_annos.(fieldname).nonex_3D(:,2*k-1) = zeros(3,1); %Clean up those points
%           all_annos.(fieldname).nonex(:,2*k-1) = zeros(3,1);
%        end
%        if all_annos.(fieldname).nonex_3D(3,2*k) < 0
%           all_annos.(fieldname).nonex_3D(:,2*k) = zeros(3,1); %Clean up those points
%           all_annos.(fieldname).nonex(:,2*k) = zeros(3,1);
%        end
%     end
    
    %Clear the separation
    search = all(~all_annos.(fieldname).nonex_3D,1);
    clear_index = unique(round(find(search == 1)/2));
    if size(clear_index,2) > 0
        all_annos.(fieldname).nonex_sep(:,clear_index) = [];
    end
    search = all(~all_annos.(fieldname).ex_3D,1);
    clear_index = unique(round(find(search == 1)/2));
    if size(clear_index,2) > 0 && isfield(all_annos.(fieldname),'ex_sep')
        all_annos.(fieldname).ex_sep(:,clear_index) = [];
    end

    %Clear up the file
   
    all_annos.(fieldname).nonex_3D(:,all(~all_annos.(fieldname).nonex_3D,1)) = [];
    all_annos.(fieldname).nonex(:,all(~all_annos.(fieldname).nonex,1)) = [];
    all_annos.(fieldname).ex_3D(:,all(~all_annos.(fieldname).ex_3D,1)) = [];
    all_annos.(fieldname).ex(:,all(~all_annos.(fieldname).ex,1)) = [];
    if isfield(all_annos.(fieldname),'ex_sep')
        assert(size(all_annos.(fieldname).ex,2)/2==size(all_annos.(fieldname).ex_sep,2));
    end
    assert(size(all_annos.(fieldname).nonex,2)/2==size(all_annos.(fieldname).nonex_sep,2));
    assert(size(all_annos.(fieldname).ex,2)==size(all_annos.(fieldname).ex_3D,2));
    assert(size(all_annos.(fieldname).nonex,2)==size(all_annos.(fieldname).nonex_3D,2));
    assert(sum(all(~all_annos.(fieldname).nonex_3D,1)) == 0, 'No zero columns anymore');
    assert(sum(all(~all_annos.(fieldname).ex_3D,1)) == 0, 'No zero columns anymore');
    
    %If no extremal then point behind to exclude, because it was in front
    %of camera before
    if size(all_annos.(fieldname).ex,2) == 0
        points_behind = true;
    end
end