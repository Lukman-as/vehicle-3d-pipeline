function jsonFile = write_3d_annotations_hw7_labeled(outRoot, camera, target, all_annos, carIds, id_to_tire, precision)
%WRITE_3D_ANNOTATIONS_HW7_LABELED Export labeled HW7 3D annotations to JSON.
%
%   jsonFile = write_3d_annotations_hw7_labeled(outRoot, camera, target, ...
%       all_annos, carIds, id_to_tire)
%
%   jsonFile = write_3d_annotations_hw7_labeled(outRoot, camera, target, ...
%       all_annos, carIds, id_to_tire, precision)
%
%   Creates:
%       <outRoot>/<camera>/<target>.json
%
%   Expected all_annos layout for each object:
%       all_annos.(obj).all_3D_points : 3xN optimized 3D points
%       all_annos.(obj).order.tire    : last tire column index
%       all_annos.(obj).order.ex_3D   : last extremal-point column index
%       all_annos.(obj).tire_ids      : 1-based IDs into id_to_tire
%
%   id_to_tire must be exactly ["DF", "PF", "PR", "DR"].
%   Missing tires are written as [-1 -1 -1].

    if nargin < 7
        precision = [];
    end

    precision = validatePrecision(precision);
    tireLabels = validateTireLabels(id_to_tire);

    outRoot = char(string(outRoot));
    camera = char(string(camera));
    target = char(string(target));
    carIds = double(carIds(:).');

    outDir = fullfile(outRoot, camera);
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    objects = emptyObjects(numel(carIds));
    for i = 1:numel(carIds)
        objId = carIds(i);
        fieldname = sprintf('obj_%d', objId);
        requireField(all_annos, fieldname, 'all_annos');

        objects(i).obj_id = objId;
        objects(i).annotations = annotationForObject(all_annos.(fieldname), tireLabels, precision, fieldname);
    end

    jsonFile = fullfile(outDir, sprintf('%s.json', target));
    writeJsonFile(jsonFile, objects);
end

function objects = emptyObjects(n)
    if n == 0
        objects = struct('obj_id', {}, 'annotations', {});
        return;
    end

    objects = repmat(struct('obj_id', NaN, 'annotations', struct()), 1, n);
end

function precision = validatePrecision(precision)
    if isempty(precision)
        return;
    end

    isWholeNumber = isnumeric(precision) && isscalar(precision) && ...
        isfinite(precision) && precision >= 0 && fix(precision) == precision;
    if ~isWholeNumber
        error('write_3d_annotations_hw7_labeled:BadPrecision', ...
            'precision must be [] or a nonnegative integer scalar.');
    end

    precision = double(precision);
end

function labels = validateTireLabels(id_to_tire)
    labels = reshape(string(id_to_tire), 1, []);
    expected = ["DF", "PF", "PR", "DR"];

    if ~isequal(labels, expected)
        error('write_3d_annotations_hw7_labeled:BadTireLabels', ...
            ['Expected id_to_tire to be exactly ["DF", "PF", "PR", "DR"], ', ...
             'because tire_ids are used as 1-based indices. Got [%s].'], ...
            char(strjoin(labels, ', ')));
    end
end

function annotations = annotationForObject(anno, tireLabels, precision, context)
    requireField(anno, 'all_3D_points', context);
    requireField(anno, 'order', context);
    requireField(anno, 'tire_ids', context);

    P = as3xN(anno.all_3D_points, sprintf('%s.all_3D_points', context));
    nCols = size(P, 2);

    order = anno.order;
    requireField(order, 'tire', sprintf('%s.order', context));
    requireField(order, 'ex_3D', sprintf('%s.order', context));

    tireEnd = requireIntegerScalar(order.tire, sprintf('%s.order.tire', context));
    exEnd = requireIntegerScalar(order.ex_3D, sprintf('%s.order.ex_3D', context));

    if tireEnd > exEnd || exEnd > nCols
        error('write_3d_annotations_hw7_labeled:BadOrder', ...
            '%s has invalid order boundaries: tire=%d, ex_3D=%d, total columns=%d.', ...
            context, tireEnd, exEnd, nCols);
    end

    if isfield(order, 'center_3D')
        centerEnd = requireIntegerScalar(order.center_3D, sprintf('%s.order.center_3D', context));
        if centerEnd < exEnd || centerEnd > nCols
            error('write_3d_annotations_hw7_labeled:BadOrder', ...
                '%s has invalid center_3D boundary: ex_3D=%d, center_3D=%d, total columns=%d.', ...
                context, exEnd, centerEnd, nCols);
        end
    else
        centerEnd = exEnd;
    end

    tirePoints = P(:, 1:tireEnd);
    exPoints = P(:, tireEnd + 1:exEnd);
    centerPoints = P(:, exEnd + 1:centerEnd).';
    nonexPoints = P(:, centerEnd + 1:end);

    annotations = struct();
    annotations.tire_points = tireStructFromMatrix(tirePoints, anno.tire_ids, tireLabels, precision, context);
    annotations.extremal_pairs = rowsToJsonRows(maybeRound(pairRows(exPoints, sprintf('%s.extremal_pairs', context)), precision));
    annotations.non_extremal_pairs = rowsToJsonRows(maybeRound(pairRows(nonexPoints, sprintf('%s.non_extremal_pairs', context)), precision));
    annotations.center_points = rowsToJsonRows(maybeRound(centerPoints, precision));
    annotations.has_mirror = getHasMirror(anno);
end

function tireStruct = tireStructFromMatrix(tirePoints, tireIds, tireLabels, precision, context)
    tireIds = double(tireIds(:).');
    nTirePoints = size(tirePoints, 2);

    if numel(tireIds) ~= nTirePoints
        error('write_3d_annotations_hw7_labeled:TireCountMismatch', ...
            '%s has %d tire_ids but %d tire point columns.', context, numel(tireIds), nTirePoints);
    end

    tireStruct = emptyTireStruct(tireLabels);
    seen = false(1, numel(tireLabels));

    for i = 1:nTirePoints
        tireId = requireIntegerScalar(tireIds(i), sprintf('%s.tire_ids(%d)', context, i));
        if tireId < 1 || tireId > numel(tireLabels)
            error('write_3d_annotations_hw7_labeled:BadTireId', ...
                '%s.tire_ids(%d)=%d is outside valid range 1..%d.', ...
                context, i, tireId, numel(tireLabels));
        end
        if seen(tireId)
            error('write_3d_annotations_hw7_labeled:DuplicateTireId', ...
                '%s has duplicate tire id %d (%s).', context, tireId, char(tireLabels(tireId)));
        end

        seen(tireId) = true;
        tireStruct.(char(tireLabels(tireId))) = maybeRound(tirePoints(:, i).', precision);
    end
end

function tireStruct = emptyTireStruct(tireLabels)
    tireStruct = struct();
    for i = 1:numel(tireLabels)
        tireStruct.(char(tireLabels(i))) = [-1, -1, -1];
    end
end

function pairRowsOut = pairRows(P, context)
    P = as3xN(P, context);
    nCols = size(P, 2);

    if nCols == 0
        pairRowsOut = zeros(0, 6);
        return;
    end

    if mod(nCols, 2) ~= 0
        error('write_3d_annotations_hw7_labeled:OddPairColumns', ...
            '%s has %d columns; expected an even number.', context, nCols);
    end

    pairRowsOut = zeros(nCols / 2, 6);
    for k = 1:(nCols / 2)
        pairRowsOut(k, :) = [P(:, 2*k - 1).', P(:, 2*k).'];
    end
end

function P = as3xN(P, context)
    P = double(P);

    if isempty(P)
        P = zeros(3, 0);
        return;
    end

    if size(P, 1) == 3
        return;
    end

    if size(P, 2) == 3
        P = P.';
        return;
    end

    error('write_3d_annotations_hw7_labeled:BadPointShape', ...
        '%s must be 3xN or Nx3; got %dx%d.', context, size(P, 1), size(P, 2));
end

function idx = requireIntegerScalar(value, context)
    isNonnegativeInteger = isnumeric(value) && isscalar(value) && ...
        isfinite(value) && value >= 0 && fix(value) == value;
    if ~isNonnegativeInteger
        error('write_3d_annotations_hw7_labeled:BadIndex', ...
            '%s must be a nonnegative integer scalar.', context);
    end

    idx = double(value);
end

function requireField(s, field, context)
    if ~isfield(s, field)
        error('write_3d_annotations_hw7_labeled:MissingField', ...
            '%s is missing required field "%s".', context, field);
    end
end

function hasMirror = getHasMirror(anno)
    if ~isfield(anno, 'has_mirror')
        hasMirror = 0;
        return;
    end

    if islogical(anno.has_mirror)
        hasMirror = double(anno.has_mirror);
    else
        hasMirror = anno.has_mirror;
    end
end

function rows = rowsToJsonRows(M)
    if isempty(M)
        rows = {};
        return;
    end

    rows = cell(size(M, 1), 1);
    for i = 1:size(M, 1)
        rows{i} = M(i, :);
    end
end

function x = maybeRound(x, precision)
    x = double(x);
    if ~isempty(precision)
        x = round(x, precision);
    end
end

function writeJsonFile(filename, objects)
    try
        txt = jsonencode(objects, 'PrettyPrint', true);
    catch
        txt = jsonencode(objects);
    end

    txt = char(txt);

    if numel(objects) == 1 && startsWith(strtrim(txt), '{')
        txt = ['[', txt, ']'];
    end

    fid = fopen(filename, 'w');
    if fid < 0
        error('write_3d_annotations_hw7_labeled:FileOpenFailed', ...
            'Could not open output JSON file: %s', filename);
    end

    try
        written = fwrite(fid, txt, 'char');
    catch ME
        fclose(fid);
        rethrow(ME);
    end

    closeStatus = fclose(fid);
    if closeStatus ~= 0
        error('write_3d_annotations_hw7_labeled:FileCloseFailed', ...
            'Could not close output JSON file after writing: %s', filename);
    end

    if written ~= numel(txt)
        error('write_3d_annotations_hw7_labeled:FileWriteFailed', ...
            'Only wrote %d of %d characters to output JSON file: %s', ...
            written, numel(txt), filename);
    end
end
