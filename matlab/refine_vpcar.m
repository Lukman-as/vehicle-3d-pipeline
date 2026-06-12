function all_annos = refine_vpcar(horizon_line,annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    anno_sympoints = [];
    %Adding points to the anno_sympoints
    if ~isfield(all_annos.(fieldname),'ex_3D') && size(all_annos.(fieldname).ex,2) ~= 0
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

    x0 = zeros(2,size(anno_sympoints,2));
    for k = 1: size(anno_sympoints,2)/2
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
        x0(1:2,2*k-1) = sym_vis_rec;
        x0(1:2,2*k) = sym_nvis_rec;
    end
    x0 = [x0 [all_annos.(fieldname).vp_car(1); all_annos.(fieldname).vp_car(2)]];

    options = optimoptions("fmincon",'MaxFunctionEvaluations',10e3, ...
        'Display','iter-detailed','Algorithm','interior-point',...
        "EnableFeasibilityMode",true,"SubproblemAlgorithm","cg",...
        "ConstraintTolerance",1e-10,"StepTolerance",1e-10,"MaxIterations",3e3);

    total = 0;
    var = 0.25;
    var_xy = 0;
    x1_cov_inv = inv([var var_xy;var_xy var]);
    x2_cov_inv = inv([var var_xy;var_xy var]);

    [x,fval] = fmincon(@(x0) vpcar_mle_fun(x1_cov_inv,x2_cov_inv,anno_sympoints, x0),...
        x0,[],[],[],[],[],[],...
        @(x0) vpcar_mle_con(horizon_line,x0),options);
    %Save new vpcar
    all_annos.(fieldname).vp_car = [x(1,end), x(2,end), 1]';
    all_annos.(fieldname).vpcar_changed = [x(1,end), x(2,end), 1]';
    %Save MLE of points if needed to avoid correct by vpasolve
    anno_sympoints_mle = zeros(3,size(anno_sympoints,2));
    for k = 1: size(x0,2)/2 - 1%Exclude last pair
        sym_vis_rec = x0(1:2,2*k-1);
        sym_nvis_rec = x0(1:2,2*k);
        anno_sympoints_mle(:,2*k-1) = [sym_vis_rec;1];
        anno_sympoints_mle(:,2*k) = [sym_nvis_rec;1];
    end
    all_annos.(fieldname).anno_sympoints_mle = anno_sympoints_mle;
end