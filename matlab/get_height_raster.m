function [cars_in_roi, mounting_height] = get_height_raster(mask,id_to_tire, annotated_car_id, all_annos, final_raster)
    %Check all within the mask first:
    fieldname = sprintf("obj_%s",int2str(annotated_car_id));
    cars_in_roi = true;
    mounting_height = -1;
    
%Why need a mask when you can use the raster directly
%     for i = 1:size(all_annos.(fieldname).tire_ids,2)
%         id = all_annos.(fieldname).tire_ids(i);
%         atire = all_annos.(fieldname).tires.(id_to_tire(id));
%         inmask = inpolygon(atire(1),atire(2),mask(:,1),mask(:,2)); 
%         if ~inmask
%             cars_in_roi = false;
%             break %Only include when all points are good
%         end
%     end
    %Only continue if all tire points are within ROI
    if cars_in_roi
        final_raster = final_raster'; %Have to transpose the raster
        all_mounting_heights = [];
        for i = 1:size(all_annos.(fieldname).tire_ids,2)
            id = all_annos.(fieldname).tire_ids(i);
            atire = all_annos.(fieldname).tires.(id_to_tire(id));
            y_pixel = round(atire(2));
            x_pixel = round(atire(1));
            a_height = final_raster(y_pixel,x_pixel);
            if a_height == 0
                cars_in_roi = false;
                break %Only include when all points are good, in case the polygon above does not work
            end
            assert(a_height > 0, 'a height has to be larger than 0');
            all_mounting_heights = [all_mounting_heights a_height];
        end
        if cars_in_roi
            mounting_height = mean(all_mounting_heights);
        end
    end
end