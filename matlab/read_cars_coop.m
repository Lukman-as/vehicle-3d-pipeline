function labels = read_cars_coop(file, using_camera)
    fid = fopen(file);
    raw = fread(fid,inf);
    str = char(raw'); 
    fclose(fid); 
    raw_data = jsondecode(str);
    valid_ids = [];
    labels = struct;
    for i = 1: size(raw_data,1)
        obj = raw_data(i);
        fieldname = sprintf("obj_%s",int2str(i));
%         if str2double(obj.occluded_state) == 0 & str2double(obj.truncated_state) == 0 & (strcmp(obj.type,'Car') || strcmp(obj.type,'Van'))
        if true
            % Different from road convert base at source
            %If used camera label
            if using_camera
                num = [obj.x2d_box.xmin, obj.x2d_box.ymin, obj.x2d_box.xmax, obj.x2d_box.ymax, ...
                    obj.x3d_dimensions.h, obj.x3d_dimensions.w, obj.x3d_dimensions.l, ...
                     str2double(obj.x3d_location.x),str2double(obj.x3d_location.y),  ...
                     str2double(obj.x3d_location.z)-obj.x3d_dimensions.h/2, ...
                    obj.rotation];
            else
%             If used virtual label
            num = [obj.x2d_box.xmin, obj.x2d_box.ymin, obj.x2d_box.xmax, obj.x2d_box.ymax, ...
                obj.x3d_dimensions.h, obj.x3d_dimensions.w, obj.x3d_dimensions.l, ...
                 obj.x3d_location.x,obj.x3d_location.y,  ...
                 obj.x3d_location.z-obj.x3d_dimensions.h/2, ...
                obj.rotation];
            end
            labels.(fieldname) = num;
            valid_ids = [valid_ids i];
        else
            labels.(fieldname) = 'Skip';
        end
    end
    labels.valid_ids = valid_ids;
end