function all_cars = read_cars(data_path,file_name)
    all_cars = [];
    %Get the 3D boxes in
    fid = fopen(sprintf("%s\\label_2\\%s.txt",data_path,file_name),'rt');
    file_revised = fopen(sprintf("data\\label_2\\%s.txt",file_name),'wt');
    while ~feof(fid)
        tline = fgetl(fid);
        data = split(tline);
        data = data'
        if strcmp(data(1,1),'Car') & str2double(data(1,2)) == 0 & strcmp(data(1,3),'0')
            num = data(1,5:end);
            num = str2double(num);
            all_cars = [all_cars ; num];
            fprintf(file_revised, "%s\n", tline);
        else
            fprintf(file_revised, "%s\n", 'Skip');
        end
    end
    fclose(fid);
end