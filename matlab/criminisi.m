
%Does not work for now
% vp_car_test = cross(cross(all_annos.(fieldname).ex(:,1),all_annos.(fieldname).ex(:,2)),horizon_line)
% vp_car_test = vp_car_test./vp_car_test(3)
% d1 = all_annos.(fieldname).ex(:,1) - vp_car_test
% d2 = all_annos.(fieldname).ex(:,2) - vp_car_test
% %Pixel
% r1 = 0.25
% r2 = 0.25
% tilda = 2*(r2*d1(1)*d1(2)+r1*d2(1)*d2(2))/(r2*(d1(1).^2-d1(2).^2) + r1*(d2(1).^2-d2(2).^2))
% l = [1+sqrt(1+tilda.^2),tilda,...
%     -(1+sqrt(1+tilda.^2))*vp_car_test(1)-tilda*vp_car_test(2)]'
% an_x = 300
% an_y = (-l(3)-l(1)*an_x)/l(2)
% F = [0 1 0;-1 0 0] %Calculate orthogonal projection
% x1 = all_annos.(fieldname).ex(:,1)
% x2 = all_annos.(fieldname).ex(:,2)
% x1_mle = [l(2)*dot(x1(1:2),F*l)-l(2)*l(3); -l(1)*dot(x1(1:2),F*l)-l(1)*l(3); l(1).^2+l(2).^2]
% x1_mle = x1_mle./x1_mle(3)
