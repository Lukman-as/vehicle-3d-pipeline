function corners_cam_aug = get_3d_bbox(all_cars, car_index)
    acar = all_cars(car_index,:);
    %Getting all the dimesion in
%     w = acar(5);
%     h = acar(6);
    h = acar(5);
    w = acar(6);
    l = acar(7);
    x = acar(8);
    y = acar(9);
    z = acar(10);
    ry = acar(11);
    
    %Get the rotation matrix to translate v
    R_bounding = [cos(ry) 0 sin(ry);
         0 1 0;
         -sin(ry) 0 cos(ry)];
    
    corners = [
        l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2;
        0  ,0  ,   0,   0, -h, -h,  -h,  -h;
        -w/2,w/2,w/2,-w/2,-w/2,w/2,w/2,-w/2;
    ];
    corners_cam = R_bounding*corners;
    corners_cam(1,:) = corners_cam(1,:) + x;
    corners_cam(2,:) = corners_cam(2,:) + y;
    corners_cam(3,:) = corners_cam(3,:) + z;

%     corners_cam = corners_cam + [x;y;z]; %This 3D coordinate from Camera 0 (reference camera coordinates)
    corners_cam_aug = [corners_cam; ones(1,size(corners_cam,2))];
end