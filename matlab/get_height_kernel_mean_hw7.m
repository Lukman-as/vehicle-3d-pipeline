function [cars_in_roi, mounting_height] = get_height_kernel_mean_hw7(mask, fobj, k_matrix, R,id_to_tire, annotated_car_id, all_annos)
    %Check all within the mask first:
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    cars_in_roi = true;
    mounting_height = -1;
    %Only continue if all tire points are within ROI
    if cars_in_roi
        all_mounting_heights = zeros(1,size(all_annos.(fieldname).tire_ids,2));
        for i = 1:size(all_annos.(fieldname).tire_ids,2)
            id = all_annos.(fieldname).tire_ids(i);
            atire = all_annos.(fieldname).tires.(id_to_tire(id));
            all_mounting_heights(i) = get_height_kernel(fobj, k_matrix, R,atire);
        end
        mounting_height = mean(all_mounting_heights);
    end
end