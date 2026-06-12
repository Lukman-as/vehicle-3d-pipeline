function t_vector = get_translation(P2_reshaped)
%     t_z = P2_reshaped(3,4);
%     t_y =  (P2_reshaped(2,4)- t_z*P2_reshaped(2,3))/P2_reshaped(2,2);
%     t_x =  (P2_reshaped(1,4)- t_z*P2_reshaped(1,3))/P2_reshaped(1,1);
%This is in KITTI way
    t_y =  P2_reshaped(2,4)/(-P2_reshaped(2,2));
    t_x =  P2_reshaped(1,4)/(-P2_reshaped(1,1));
    t_z = P2_reshaped(3,4);
    t_vector = [t_x t_y t_z]';
end