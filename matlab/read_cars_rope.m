function labels = read_cars_rope(file)
    labels = struct;
    %Get the 3D boxes in
    fid = fopen(file);
    obj_id = 1
    valid_ids = [];
    while ~feof(fid)
        tline = fgetl(fid);
        data = split(tline);
        data = data'
        fieldname = sprintf("obj_%s",int2str(obj_id))
        if strcmp(data(1,1),'car') & str2double(data(1,2)) == 0 & str2double(data(1,3)) == 0
            num = data(1,5:end);
            num = str2double(num);
            labels.(fieldname) = num;
            valid_ids = [valid_ids obj_id];
        else
            labels.(fieldname) = 'Skip'
        end
        obj_id = obj_id + 1;
    end
    fclose(fid);
    labels.valid_ids = valid_ids;
end