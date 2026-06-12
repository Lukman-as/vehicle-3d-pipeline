function all_annos = get_obj_height(all_annos, annotated_car_id,mounting_height)
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    all_annos.(fieldname).mounting_height = mounting_height;
end