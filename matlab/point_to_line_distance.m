function distance=point_to_line_distance(pt, v1, v2)
%Calculate distance between a point and a line in 2D or 3D.
% syntax:
% distance = point_to_line(pt, v1, v2)
% pt is a nx3 matrix with xyz coordinates for n points
% v1 and v2 are vertices on the line (each 1x3)
% d is a nx1 vector with the orthogonal distances
%
% 2D inputs are extended to 3D by setting all z-values to 0, all inputs should be either nx2 or nx3
% (1x2 or 1x3 for v1 and v2).
%
% The actual calculation is a slightly edit version of this line (which only works for 3D points):
% distance=norm(cross(v1-v2,pt-v2))/norm(v1-v2)
%
%check inputs
%prepare inputs
% v1=v1(:)';%force 1x3 or 1x2
% v2=v2(:)';%force 1x3 or 1x2

v1_ = repmat(v1(:)',size(pt,1),1);
v2_ = repmat(v2(:)',size(pt,1),1);
%actual calculation
a = v1_ - v2_;
b = pt - v2_;
distance = sqrt(sum(cross(a,b,2).^2,2)) ./ sqrt(sum(a.^2,2));
%this is equivalent to the following line for a single point
%distance=norm(cross(v1-v2,pt-v2))/norm(v1-v2)
end