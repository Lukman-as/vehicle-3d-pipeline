function labels = read_cars_hw7(file)
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
        % Different from road convert base at source
        %If used camera label
        if ~strcmp(obj.type,'Pedestrian')
            num = [obj.x2d_box.xmin, obj.x2d_box.ymin, obj.x2d_box.xmax, obj.x2d_box.ymax, ...
                obj.x3d_dimensions.x, obj.x3d_dimensions.y, obj.x3d_dimensions.z, ...
                obj.x3d_location.x,obj.x3d_location.y, obj.x3d_location.z-obj.x3d_dimensions.z/2, ...
                obj.rotation];
    %             If used virtual label
    %         num = [obj.x2d_box.xmin, obj.x2d_box.ymin, obj.x2d_box.xmax, obj.x2d_box.ymax, ...
    %             obj.x3d_dimensions.h, obj.x3d_dimensions.w, obj.x3d_dimensions.l, ...
    %              obj.x3d_location.x,obj.x3d_location.y,  ...
    %              obj.x3d_location.z-obj.x3d_dimensions.h/2, ...
    %             obj.rotation];
            labels.(fieldname) = num;
            valid_ids = [valid_ids i];
        end
    end
    labels.valid_ids = valid_ids;
end