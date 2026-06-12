function vp_points = get_vppoints(k_matrix,R)
    count = 3;
    vp_points = zeros(3,3);
    for i = 1:count
        zer = zeros(3,1);
        zer(i) = 1;
        temp = k_matrix*R*zer; %Multiply with K for each point
        vp_points(:,i) = temp;
    end
end