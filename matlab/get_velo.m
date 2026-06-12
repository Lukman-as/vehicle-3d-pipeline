function velo = get_velo(data_path,file_name)
    fid = fopen(sprintf("%s\\velodyne\\%s.bin",data_path,file_name), 'r');
    velo = fread(fid, '*float32');
    fclose(fid);
    velo = reshape(velo,[4,size(velo,1)/4,])';
end

