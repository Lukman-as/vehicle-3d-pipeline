function worldPoint = mapIMGPointsToWorld(principal, focal, pixelOffset, imgPoint)

%MUST HAVE THE Y IMG COORDINATES FROM MATLAB *(-1) AND  you must have the
%principal point on the Y axis also (-1)
%mapping from img to world (pixelX*pixelXOffset -
%princPoint(1)*pixelXOffset, pixelY*pixelYOffset + princPoint(2)*pixelYOffset) |-> (X,Y,focal)

worldPoint = zeros(3,size(imgPoint,1));
worldPoint(3,:) = focal;
% for i = 1:size(imgPoint,1)
%     worldPoint(1,i) = imgPoint(i,1)*pixelOffset - principal(1)*pixelOffset;
%     worldPoint(2,i) = -(imgPoint(i,2)*pixelOffset - principal(2)*pixelOffset);
% end

worldPoint(1,:)= (imgPoint(:,1)*pixelOffset - principal(1)*pixelOffset)';
worldPoint(2,:)= (-(imgPoint(:,2)*pixelOffset - principal(2)*pixelOffset))';
