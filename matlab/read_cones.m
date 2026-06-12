function cones = read_cones(file)
    fid = fopen(file);
    raw = fread(fid,inf);
    str = char(raw'); 
    fclose(fid); 
    raw_data = jsondecode(str);
    valid_ids = [];
    cones = struct;
    for i = 1: size(raw_data,1)
        obj = raw_data(i);
        fieldname = sprintf("obj_%s",int2str(i));
        if str2double(obj.occluded_state) == 0 && ...
            str2double(obj.truncated_state) == 0 && (strcmp(obj.type,'Trafficcone'))
            % Different from road convert base at source
            %If used camera label
            num = [obj.x2d_box.xmin, obj.x2d_box.ymin, obj.x2d_box.xmax, obj.x2d_box.ymax, ...
                obj.x3d_dimensions.h, obj.x3d_dimensions.w, obj.x3d_dimensions.l, ...
                 str2double(obj.x3d_location.x),str2double(obj.x3d_location.y),  ...
                 str2double(obj.x3d_location.z)-obj.x3d_dimensions.h/2, ...
                obj.rotation];
            cones.(fieldname) = num;
            valid_ids = [valid_ids i];
        end
    end
    cones.valid_ids = valid_ids;
end