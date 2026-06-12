function vp_car = get_vpcar_gt(annotated_car_id, all_annos, horizon_line,k_matrix,R)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id))
    corners_bbox_world = all_annos.(fieldname).corners_bbox_world(1:3,:)
    corners_bbox_2D = k_matrix*R*corners_bbox_world
    line = cross(corners_bbox_2D(:,1),corners_bbox_2D(:,2))
    vp_car = cross(line,horizon_line)
    vp_car = vp_car./vp_car(3)
    assert(abs(dot(vp_car,horizon_line)) < 1e-8, 'VP car gotta be on horizon line')
end