function [c,ceq] = vpcar_mle_con(horizon_line,x0)
    %Get the param
    ceq = [];
    vp_car = [x0(1,end), x0(2,end), 1]';
    ceq = [ceq dot(vp_car, horizon_line)];
    for k = 1: size(x0,2)/2 - 1%Exclude last pair
        sym_vis_rec = x0(1:2,2*k-1);
        sym_nvis_rec = x0(1:2,2*k);
        ceq = [ceq dot(vp_car,cross([sym_vis_rec;1],[sym_nvis_rec;1]))];
    end  
    c = 0;
end