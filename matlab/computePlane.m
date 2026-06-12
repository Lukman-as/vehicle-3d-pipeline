function plane=computePlane(p,planeNorm)
% %the plane equation is ax+by+cz+d=0, in this case the normal vector of the
% %plane is [a,b,c]

%take any point to calculate the parameter d in the equation
d=-(planeNorm*p');
plane=[planeNorm d];

end