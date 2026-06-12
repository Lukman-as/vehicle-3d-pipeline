function all_annos = heading_from_normal(annotated_car_id, all_annos)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    if all_annos.(fieldname).car_origin == 1 || all_annos.(fieldname).car_origin == 4
        Ry_angle = -pi/2;
    elseif all_annos.(fieldname).car_origin == 3 || all_annos.(fieldname).car_origin == 2
        Ry_angle = pi/2;
    end
    R_head = [cos(Ry_angle) 0 sin(Ry_angle);
        0 1 0;
        -sin(Ry_angle) 0 cos(Ry_angle)];
    all_annos.(fieldname).heading_dir = R_head*all_annos.(fieldname).sym_normal_unit;
    all_annos.(fieldname).heading_angle  = acos(dot(all_annos.(fieldname).heading_dir,[1,0,0]'))*...
        dot(cross(all_annos.(fieldname).heading_dir,[1,0,0]'),[0,1,0]);
end