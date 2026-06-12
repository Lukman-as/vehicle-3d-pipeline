function P2 = read_calib_rope(file)
    fid = fopen(file);
    tline = fgetl(fid);
    data = split(tline);
    size(data);
    field = data(1,1);
    if strcmp(field,'P2:')
        disp(field);
        num = data(2:end,:);
        num = str2double(num);
        P2 = num';
    fclose(fid);
    P2 = reshape(P2,[4,3])';
%     P2 = [P2; 0 0 0 1];
end