function all_annos = eval_dist_sym_plane(annotated_car_id,all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    vec = all_annos.(fieldname).corners_bbox_world(:,1) - all_annos.(fieldname).corners_bbox_world(:,2);
    vec = vec/norm(vec);
    assert(vec(2)==0);
    mid_point = mean([all_annos.(fieldname).corners_bbox_world(:,1), ...
        all_annos.(fieldname).corners_bbox_world(:,2)],2)
    if dot(vec,mid_point) < 0
        vec = -vec
    end
    all_annos.(fieldname).dist_sym_gt = dot(vec,mid_point)
    vec = all_annos.(fieldname).predicted_corners(:,1) - all_annos.(fieldname).predicted_corners(:,2);
    vec = vec/norm(vec)
    assert(vec(2)==0);
    mid_point = mean([all_annos.(fieldname).predicted_corners(:,1), ...
        all_annos.(fieldname).predicted_corners(:,2)],2)
    if dot(vec,mid_point) < 0
        vec = -vec
    end
    all_annos.(fieldname).dist_sym_pred = dot(vec,mid_point)
    all_annos.(fieldname).dist_sym_diff = abs(all_annos.(fieldname).dist_sym_gt-all_annos.(fieldname).dist_sym_pred)
end