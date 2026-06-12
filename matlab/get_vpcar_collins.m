function vp_car = get_vpcar_collins(anno_sympoints_homo, image,vp_points)
    %Find all the intersection
%     anno_sympoints_homo_translated = [anno_sympoints_homo(1,:)-size(image,2)/2;
%     anno_sympoints_homo(2,:)-size(image,1)/2;
%     ones(1,size(anno_sympoints_homo,2))
%     ];
    anno_sympoints_homo = [anno_sympoints_homo, vp_points(:,1),vp_points(:,3)]
    w = (size(image,2)+size(image,1))/4;
    anno_sympoints_homo_scaled = w*anno_sympoints_homo;
    
    all_lines = zeros(3,size(anno_sympoints_homo_scaled,2)/2);
    for i = 1:size(anno_sympoints_homo_scaled,2)/2
        formed_line = cross(anno_sympoints_homo_scaled(:,2*i-1), anno_sympoints_homo_scaled(:,2*i)); %Forms from a pair of point
        all_lines(:,i) = formed_line;
    end
    
    M = zeros(3,3);
    for i = 1:size(all_lines,2)
        a = all_lines(1,i);
        b = all_lines(2,i);
        c = all_lines(3,i);
        M = M + [a*a a*b a*c; a*b b*b b*c; a*c b*c c*c];
    end
    
    [V,d,it_num,rot_num] = jacobi_eigenvalue(M, 1000);
    vp_car = V(:,1);
    vp_car = vp_car./vp_car(3);
end