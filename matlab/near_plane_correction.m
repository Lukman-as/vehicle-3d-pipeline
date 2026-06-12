function [near_tground_3D_corrected, near_tground_back_3D_corrected, near_plane_normalized_corrected] = ...
    near_plane_correction(k_matrix,R,sym_normal_unit, near_tground,near_tground_3D, near_tground_back,poleheight,image)
    temp =  inv(k_matrix*R)*near_tground_back;
    lambda = poleheight/temp(2);
    near_tground_back_3D = [temp(1)*lambda poleheight temp(3) * lambda]';
    tground_centroid = mean([near_tground_back_3D, near_tground_3D], 2);
    near_d = -dot(tground_centroid,sym_normal_unit);
    near_plane_normalized_corrected = [sym_normal_unit; near_d];


    tground_3D = get_refined_back(k_matrix,R,near_d,sym_normal_unit,...
        poleheight,[near_tground near_tground_back], true,image,tground_centroid,near_tground,near_tground_back);

    near_tground_3D_corrected = tground_3D(:,1);
    near_tground_back_3D_corrected = tground_3D(:,2);


    
   %Unit check both of these has to be 0
   assert(dot(near_tground_3D_corrected,sym_normal_unit) + near_plane_normalized_corrected(4) < 1e-12)
   assert(dot(near_tground_back_3D_corrected,sym_normal_unit) + near_plane_normalized_corrected(4) < 1e-12)
    
    %Visualization
    near_tground_3D_to_2D = k_matrix*R*near_tground_3D_corrected;
    near_tground_3D_to_2D = near_tground_3D_to_2D/near_tground_3D_to_2D(3);
    near_tground_back_3D_to_2D = k_matrix*R*near_tground_back_3D_corrected;
    near_tground_back_3D_to_2D = near_tground_back_3D_to_2D/near_tground_back_3D_to_2D(3);
      if false        %     %Visualize those points on image to confirm
        figure;
        %hold on
        imshow(image)
        hold on
        scatter(near_tground(1,:),near_tground(2,:),80,'blue','filled');
        scatter(near_tground_back(1,:),near_tground_back(2,:),80,'blue','filled'); 
        scatter(near_tground_3D_to_2D(1,:),near_tground_3D_to_2D(2,:),40,'green','filled');
        scatter(near_tground_back_3D_to_2D(1,:),near_tground_back_3D_to_2D(2,:),40,'green','filled');
        %%Plot the intersection of all vanishing lines
        axis equal
        axis on
        xlabel('X')
        ylabel('Y')
      end
end