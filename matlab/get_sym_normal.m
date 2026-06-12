function all_annos = get_sym_normal(annotated_car_id, all_annos,k_matrix,R)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    vp_car = all_annos.(fieldname).vp_car;
    sym_normal = inv(k_matrix*R)*vp_car;
    sym_normal_unit = sym_normal/norm(sym_normal);
    all_annos.(fieldname).sym_normal_unit = sym_normal_unit;
    assert(abs(sym_normal(2)) <= 1e-6);
    assert(abs(all_annos.(fieldname).sym_normal_unit(2)) <= 1e-6);
end