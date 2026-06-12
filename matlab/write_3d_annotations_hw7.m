function jsonFile = write_3d_annotations_hw7_labeled(out_root, camera, target, all_annos, carIds)
    out_dir = fullfile(out_root, char(camera));    
    if ~isfolder(out_dir)
        mkdir(out_dir);
    end

    objects = {};
    for i = 1:numel(carIds)
        objId = carIds(i);
        fieldname = sprintf('obj_%d', objId);
        points = double(all_annos.(fieldname).all_3D_points);
        points = points';
        obj = struct();
        obj.obj_id = double(objId);
        obj.points = points;
        objects{end + 1} = obj;
    end
    jsonFile = fullfile(out_dir, sprintf('%s.json', char(target)));
    try
        json_text = jsonencode(objects, 'PrettyPrint', true);
    catch
        % Older MATLAB versions may not support PrettyPrint.
        json_text = jsonencode(objects);
    end

    fid = fopen(jsonFile, 'w');
    if fid < 0
        error('Could not open output file: %s', jsonFile);
    end

    cleanupObj = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', json_text);