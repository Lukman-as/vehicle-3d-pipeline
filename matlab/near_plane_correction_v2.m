function [near_tground_3D_corrected, near_tground_back_3D_corrected, near_plane_normalized_corrected] = ...
    near_plane_correction(k_matrix,R,sym_normal_unit, near_tground,near_tground_3D, near_tground_back,poleheight,image)
    near_plane_normalized_corrected = [sym_normal_unit; -dot(near_tground_3D,sym_normal_unit)];
    lambda_back = -near_plane_normalized_corrected(4)/dot(inv(k_matrix*R)*near_tground_back,sym_normal_unit);
    near_tground_back_3D_corrected = lambda_back*inv(k_matrix*R)*near_tground_back;
    near_tground_3D_corrected = near_tground_3D;

   %Unit check both of these has to be 0
   assert(dot(near_tground_3D_corrected,sym_normal_unit) + near_plane_normalized_corrected(4) < 1e-12)
   assert(dot(near_tground_back_3D_corrected,sym_normal_unit) + near_plane_normalized_corrected(4) < 1e-12)
    
    %Visualization
    near_tground_3D_to_2D = k_matrix*R*near_tground_3D_corrected;
    near_tground_3D_to_2D = near_tground_3D_to_2D/near_tground_3D_to_2D(3);
    near_tground_back_3D_to_2D = k_matrix*R*near_tground_back_3D_corrected;
    near_tground_back_3D_to_2D = near_tground_back_3D_to_2D/near_tground_back_3D_to_2D(3);
      if false        %     %Visualize those points on image to confirm
        figure
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