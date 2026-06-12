function [no_solution all_annos] = sym_points_3D(k_matrix,R,annotated_car_id,all_annos,image)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));

    if all_annos.(fieldname).tire_both_sides
        fields_to_clear = ["nonex_sep", "nonex_3D","ex_sep","ex_3D"];
    else
        fields_to_clear = ["nonex_sep", "nonex_3D"];
    end

    mounting_height = all_annos.(fieldname).mounting_height;
    for i = 1:size(fields_to_clear,2)
        if isfield(all_annos.(fieldname),fields_to_clear(i))
            all_annos.(fieldname) = rmfield(all_annos.(fieldname),fields_to_clear(i));
        end
    end
    anno_sympoints = [];

    %Adding points to the anno_sympoints
%     if ~isfield(all_annos.(fieldname),'ex_3D') & size(all_annos.(fieldname).ex,2) ~= 0
    if all_annos.(fieldname).tire_both_sides
        if all_annos.(fieldname).visible == "D"
            anno_sympoints = [anno_sympoints all_annos.(fieldname).ex(:,1) all_annos.(fieldname).ex(:,2)];
        else
            anno_sympoints = [anno_sympoints all_annos.(fieldname).ex(:,2) all_annos.(fieldname).ex(:,1)];
        end
    end
    
    for i = 1:size(all_annos.(fieldname).nonex,2)/2
        if all_annos.(fieldname).visible == "D"
            anno_sympoints = [anno_sympoints all_annos.(fieldname).nonex(:,2*i-1) all_annos.(fieldname).nonex(:,2*i)];
        else
            anno_sympoints = [anno_sympoints all_annos.(fieldname).nonex(:,2*i) all_annos.(fieldname).nonex(:,2*i-1)];
        end
    end
    
    all_annos.(fieldname).nonex_3D = [];
    all_annos.(fieldname).nonex_sep = []; 
    no_solution = false;
    for k = 1: size(anno_sympoints,2)/2
        2*k-1;
        2*k;
        sym_vis = anno_sympoints(1:2,2*k-1);
        sym_nvis = anno_sympoints(1:2,2*k);
        %Get the centroid of it
         centroid = mean([sym_vis, sym_nvis], 2);
        %Find rectified symmetry points
         sym_line = all_annos.(fieldname).vp_car(1:2) - centroid; %Symmetry line
        centroid_vis_vec = sym_vis - centroid;
        centroid_nvis_vec = sym_nvis - centroid;
        sym_vis_rec = centroid + dot(centroid_vis_vec, sym_line)/norm(sym_line)...
            *(sym_line/norm(sym_line));
        sym_nvis_rec = centroid + dot(centroid_nvis_vec, sym_line)/norm(sym_line)...
            *(sym_line/norm(sym_line));
    
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
            line([sym_nvis_rec(1,:), all_annos.(fieldname).vp_car(1,:)], ...
                [sym_nvis_rec(2,:), all_annos.(fieldname).vp_car(2,:)],'Color','green','LineWidth',1,'LineStyle','--'); 
            axis equal
            axis on
            xlabel('X')
            ylabel('Y')
        end
    
%         sym_vis_rec = all_annos.(fieldname).anno_sympoints_mle(1:2,2*k-1);
%         sym_nvis_rec = all_annos.(fieldname).anno_sympoints_mle(1:2,2*k);

        vec= sym_vis_rec-sym_nvis_rec;
        assert(abs(vec(2)/vec(1)-sym_line(2)/sym_line(1))<1e-8);
        sym_vis_rec_aug = [sym_vis_rec; 1]; %Augment it
        sym_nvis_rec_aug = [sym_nvis_rec; 1];

        
   
        %Optimization process
%         p_nvis = inv(k_matrix*R)*sym_nvis_rec_aug;
%         p_vis = inv(k_matrix*R)*sym_vis_rec_aug;
        p_nvis = R'*inv(k_matrix)*sym_nvis_rec_aug;
        p_vis = R'*inv(k_matrix)*sym_vis_rec_aug;
    
        w_hat = all_annos.(fieldname).w_unit;
        lambda_vis = (-all_annos.(fieldname).ds - w_hat/2)/...
            dot(all_annos.(fieldname).sym_normal_unit,p_vis);
        lambda_nvis = (-all_annos.(fieldname).ds + w_hat/2)/...
            dot(all_annos.(fieldname).sym_normal_unit,p_nvis);

        init_guess = [lambda_vis; lambda_nvis];
        syms lambda_vis lambda_nvis;
        digits(8)
        eqn1 = lambda_nvis*dot(all_annos.(fieldname).sym_normal_unit,p_nvis)...
            == -all_annos.(fieldname).ds + 1/2*norm(lambda_nvis*p_nvis - lambda_vis*p_vis);
        eqn2 = lambda_vis*dot(all_annos.(fieldname).sym_normal_unit,p_vis)...
            == -all_annos.(fieldname).ds - 1/2*norm(lambda_nvis*p_nvis - lambda_vis*p_vis);

        sol = vpasolve([eqn1 eqn2], [lambda_vis, lambda_nvis],...
            [(mounting_height-5)/p_vis(2) mounting_height/p_vis(2);...
            (mounting_height-5)/p_nvis(2) mounting_height/p_nvis(2)]);
%         sol = vpasolve([eqn1 eqn2], [lambda_vis, lambda_nvis]);
        lambda_vis = double(sol.lambda_vis)
        lambda_nvis = double(sol.lambda_nvis)

    
    % in case vpa solve cannot solve
%         if size(lambda_nvis,1)== 0
%             lambda_vis = (-all_annos.(fieldname).ds - w_hat/2)/...
%             dot(all_annos.(fieldname).sym_normal_unit,p_vis);
%             lambda_nvis = (-all_annos.(fieldname).ds + w_hat/2)/...
%                 dot(all_annos.(fieldname).sym_normal_unit,p_nvis);
%             X_final = fsolve(@(x)dfitgamma(x,k_matrix,R,sym_vis_rec_aug,...
%                 sym_nvis_rec_aug,all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).ds,mounting_height),...
%                 [lambda_vis lambda_nvis 0 0],...
%                 optimset('display','off','TolFun',1e-20, 'Algorithm','trust-region-dogleg'));
%             lambda_vis = round(X_final(1),5)
%             lambda_nvis = round(X_final(2),5)
%         end
        
        if size(lambda_nvis,1) == 0 || size(lambda_vis,1) == 0
            no_solution = true;
            break
        end
        sym_vis_3D = lambda_vis*p_vis;
        sym_nvis_3D = lambda_nvis*p_nvis;

        if ~size(sol.lambda_vis,1)== 0
            'CHECKINGGGGG'
            abs(dot(all_annos.(fieldname).sym_normal_unit, lambda_nvis*p_nvis+lambda_vis*p_vis) ...
                +2*all_annos.(fieldname).ds)
            assert(abs(dot(all_annos.(fieldname).sym_normal_unit, lambda_nvis*p_nvis+lambda_vis*p_vis) ...
                +2*all_annos.(fieldname).ds)<1e-2);
            assert(abs(dot(all_annos.(fieldname).sym_normal_unit, lambda_nvis*p_nvis-lambda_vis*p_vis) ...
                - norm(lambda_nvis*p_nvis-lambda_vis*p_vis))<1e-2);
        end
        
        %Save ex
%         if ~isfield(all_annos.(fieldname),'ex_3D') & size(all_annos.(fieldname).ex,2) ~= 0
        if all_annos.(fieldname).tire_both_sides && ~isfield(all_annos.(fieldname),'ex_3D')
            if all_annos.(fieldname).visible == "D"
                all_annos.(fieldname).ex_3D = [sym_vis_3D sym_nvis_3D];
            else
                all_annos.(fieldname).ex_3D = [sym_nvis_3D sym_vis_3D];
            end
            all_annos.(fieldname).ex_sep = norm(sym_vis_3D-sym_nvis_3D);
            continue;
        end

        %Save non ex
        if all_annos.(fieldname).visible == "D"
            all_annos.(fieldname).nonex_3D = [all_annos.(fieldname).nonex_3D sym_vis_3D sym_nvis_3D];
        else
            all_annos.(fieldname).nonex_3D = [all_annos.(fieldname).nonex_3D sym_nvis_3D sym_vis_3D];
        end
        all_annos.(fieldname).nonex_sep = [all_annos.(fieldname).nonex_sep ...
            norm(sym_vis_3D-sym_nvis_3D)];
    end
    
    if ~no_solution
        if size(all_annos.(fieldname).ex,2) ~= 0
            assert(size(all_annos.(fieldname).ex,2)==size(all_annos.(fieldname).ex_3D,2));
        end
        all_annos.(fieldname).nonex
        all_annos.(fieldname).nonex_3D
        assert(size(all_annos.(fieldname).nonex,2)==size(all_annos.(fieldname).nonex_3D,2));
    end
end