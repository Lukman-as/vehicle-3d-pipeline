function d=minimumdistance2point(scale,orig,dir,lineSamples)

point=orig+scale*dir;
[dist, pos] = distancePointLine(point, lineSamples);
d=sum(dist);
end