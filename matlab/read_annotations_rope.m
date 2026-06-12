function [num_cars_anno, annotations] = read_annotations_rope(file)
    annotations = struct;
    %Get the 3D boxes in
    fid = fopen(file);
    obj_id = 1;
    while ~feof(fid)
        tline = fgetl(fid);
        fieldname = sprintf("obj_%s",int2str(obj_id));
        value = jsondecode(tline);
        annotations.(fieldname) = value;
        obj_id = obj_id + 1;
    end
    num_cars_anno = obj_id-1;
    fclose(fid);
end