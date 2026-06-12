function [P2_reshaped, R0_rect_aug, Tr_velo_to_cam_aug] = read_calib(data_path,file_name)
    fid = fopen(sprintf("%s\\calib\\%s.txt",data_path,file_name),'rt');
    while ~feof(fid)
        tline = fgetl(fid);
        data = split(tline);
        size(data);
        field = data(1,1);
        if strcmp(field,'P2:')
%             disp(field);
            num = data(2:end,:);
            num = str2double(num);
            P2 = num';
        elseif strcmp(field,'R0_rect:')
%             disp(field);
            num = data(2:end,:);
            num = str2double(num);
            R0_rect = num';
        elseif strcmp(field,'Tr_velo_to_cam:')
%             disp(field);
            num = data(2:end,:);
            num = str2double(num);
            Tr_velo_to_cam = num';
        end
    end
    fclose(fid);
    Tr_velo_to_cam = reshape(Tr_velo_to_cam,[4,3])';
    Tr_velo_to_cam_aug = [Tr_velo_to_cam; 0 0 0 1];
    R0_rect_aug = zeros([4,4]);
    %This transpose is to make up for the difference between python and matlab
    R0_rect_aug(1:3,1:3) = reshape(R0_rect,[3,3])';
    R0_rect_aug(4,4) = 1;
    P2_reshaped = reshape(P2,[4,3])';
end