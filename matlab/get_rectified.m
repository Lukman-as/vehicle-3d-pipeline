function [non_ex1_l_rec non_ex1_r_rec] = get_rectified(non_ex1_l, non_ex1_r, vp_car)
    non_ex1_s = mean([non_ex1_l , non_ex1_r], 2);
    %Find rectified symmetry points
    non_ex1_vline = vp_car(1:2) - non_ex1_s; %Symmetry line
    non_ex1_left_proj = non_ex1_l - non_ex1_s;
    non_ex1_right_proj = non_ex1_r - non_ex1_s;
    non_ex1_l_rec = dot(non_ex1_left_proj,non_ex1_vline)...
        /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s;
    non_ex1_r_rec = dot(non_ex1_right_proj,non_ex1_vline)...
        /dot(non_ex1_vline,non_ex1_vline)*non_ex1_vline + non_ex1_s;
    non_ex1_l_rec = [non_ex1_l_rec; 1]; %Augment it
    non_ex1_r_rec = [non_ex1_r_rec; 1];
end