function all_annos = sym_points_3D_fmincon(k_matrix,R,annotated_car_id,all_annos,image)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    fields_to_clear = ["nonex_sep", "nonex_3D","ex_sep","ex_3D"];
    mounting_height = all_annos.(fieldname).mounting_height;
    for i = 1:size(fields_to_clear,2)
        if isfield(all_annos.(fieldname),fields_to_clear(i))
            all_annos.(fieldname) = rmfield(all_annos.(fieldname),fields_to_clear(i));
        end
    end

    anno_sympoints = [];

    %Adding points to the anno_sympoints
    if ~isfield(all_annos.(fieldname),'ex_3D') & size(all_annos.(fieldname).ex,2) ~= 0
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
    for k = 1: size(anno_sympoints,2)/2
        2*k-1
        2*k
        sym_vis = anno_sympoints(1:2,2*k-1)
        sym_nvis = anno_sympoints(1:2,2*k)
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
        vec= sym_vis_rec-sym_nvis_rec;
        assert(abs(vec(2)/vec(1)-sym_line(2)/sym_line(1))<1e-12);

        sym_vis_raw_aug = [sym_vis; 1]; %Augment it
        sym_nvis_raw_aug = [sym_nvis; 1];

        sym_vis_rec_aug = [sym_vis_rec; 1]; %Augment it
        sym_nvis_rec_aug = [sym_nvis_rec; 1];
    
        %Optimization process
        p_nvis = inv(k_matrix*R)*sym_nvis_rec_aug;
        p_vis = inv(k_matrix*R)*sym_vis_rec_aug;
    
        w_hat = all_annos.(fieldname).w_unit;
        lambda_vis = (-all_annos.(fieldname).ds - w_hat/2)/...
            dot(all_annos.(fieldname).sym_normal_unit,p_vis);
        lambda_nvis = (-all_annos.(fieldname).ds + w_hat/2)/...
            dot(all_annos.(fieldname).sym_normal_unit,p_nvis);


%         sol = vpasolve([eqn1 eqn2], [lambda_vis, lambda_nvis],...
%             [(mounting_height-5)/p_vis(2) mounting_height/p_vis(2);...
%             (mounting_height-5)/p_nvis(2) mounting_height/p_nvis(2)]);
        x0 = [sym_vis_rec(1); sym_vis_rec(2); sym_nvis_rec(1);  sym_nvis_rec(2); lambda_vis; lambda_nvis];
        options = optimoptions("fmincon",'MaxFunctionEvaluations',10e3, ...
            'Display','notify-detailed','Algorithm','interior-point',...
            "EnableFeasibilityMode",true,"SubproblemAlgorithm","cg",...
            "ConstraintTolerance",1e-6,"StepTolerance",1e-10,"MaxIterations",3e3);
%         sym_3D_fun(k_matrix,R,sym_nvis_raw_aug,sym_vis_raw_aug, x0);
%         sym_3D_con(k_matrix,R, all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).ds,x0);
        np_std = 3; %Allow freedom
        lb = [sym_vis(1)-np_std; sym_vis(2)-np_std; sym_nvis(1)-np_std;  sym_nvis(2)-np_std; ...
            (mounting_height-2)/p_vis(2); (mounting_height-2)/p_nvis(2)];
        ub = [sym_vis(1)+np_std; sym_vis(2)+np_std; sym_nvis(1)+np_std;  sym_nvis(2)+np_std; ...
            (mounting_height)/p_vis(2); (mounting_height)/p_nvis(2)];


        [x,fval] = fmincon(@(x0) sym_3D_fun(k_matrix,R,sym_nvis_raw_aug,sym_vis_raw_aug, x0),...
        x0,[],[],[],[],lb,ub,...
        @(x0) sym_3D_con(k_matrix,R, all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).ds,x0),options);
        
%         [c,ceq] = sym_3D_con(k_matrix,R, all_annos.(fieldname).sym_normal_unit,all_annos.(fieldname).ds,x)
        %Export 3D points
        [sym_vis_3D, sym_nvis_3D] = get_3D_from_x0_coop(k_matrix,R, x);

        %AFter optimization check
        x0 = x;
        sym_vis = [x0(1) x0(2)]';
        sym_nvis = [x0(3) x0(4)]';
        lambda_vis = x0(5);
        lambda_nvis = x0(6);
        sym_vis_rec_aug = [sym_vis; 1]; %Augment it
        sym_nvis_rec_aug = [sym_nvis; 1];
        p_nvis = inv(k_matrix*R)*sym_nvis_rec_aug;
        p_vis = inv(k_matrix*R)*sym_vis_rec_aug;


        abs(dot(all_annos.(fieldname).sym_normal_unit, lambda_nvis*p_nvis+lambda_vis*p_vis) ...
            +2*all_annos.(fieldname).ds)
        assert(abs(dot(all_annos.(fieldname).sym_normal_unit, lambda_nvis*p_nvis+lambda_vis*p_vis) ...
            +2*all_annos.(fieldname).ds)<1e-1);
        abs(dot(all_annos.(fieldname).sym_normal_unit, lambda_nvis*p_nvis-lambda_vis*p_vis) ...
            - norm(lambda_nvis*p_nvis-lambda_vis*p_vis))
        assert(abs(dot(all_annos.(fieldname).sym_normal_unit, lambda_nvis*p_nvis-lambda_vis*p_vis) ...
            - norm(lambda_nvis*p_nvis-lambda_vis*p_vis))<1e-1);
        
        %Save ex
        if ~isfield(all_annos.(fieldname),'ex_3D') & size(all_annos.(fieldname).ex,2) ~= 0
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
%     all_annos.(fieldname).ex_3D

    if size(all_annos.(fieldname).ex,2) ~= 0
        assert(size(all_annos.(fieldname).ex,2)==size(all_annos.(fieldname).ex_3D,2));
    end
    assert(size(all_annos.(fieldname).nonex,2)==size(all_annos.(fieldname).nonex_3D,2));
end