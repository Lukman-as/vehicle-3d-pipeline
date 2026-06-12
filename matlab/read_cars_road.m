function labels = read_cars_road(file)
    fid = fopen(file);
    raw = fread(fid,inf);
    str = char(raw'); 
    fclose(fid); 
    raw_data = jsondecode(str);
    obj_id = 1;
    valid_ids = [];
    labels = struct;
    for i = 1: size(raw_data,1)
        obj = raw_data(i);
        fieldname = sprintf("obj_%s",int2str(obj_id));
        if str2double(obj.occluded_state) == 0 && ...
                str2double(obj.truncated_state) == 0 & (strcmp(obj.type,'Car') || strcmp(obj.type,'Van'))
            num = [str2double(obj.x2d_box.xmin) str2double(obj.x2d_box.ymin) str2double(obj.x2d_box.xmax) str2double(obj.x2d_box.ymax) ...
                str2double(obj.x3d_dimensions.h) str2double(obj.x3d_dimensions.w) str2double(obj.x3d_dimensions.l) ...
                str2double(obj.x3d_location.x) str2double(obj.x3d_location.y) str2double(obj.x3d_location.z)-str2double(obj.x3d_dimensions.h)/2 ...
                str2double(obj.rotation)];
            labels.(fieldname) = num;
            valid_ids = [valid_ids obj_id];
        else
            labels.(fieldname) = 'Skip';
        end
        obj_id = obj_id + 1;
    end
    labels.valid_ids = valid_ids;
end