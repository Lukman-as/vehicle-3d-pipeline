function worldPoint = mapIMGPointsToWorldRegular(principal, focal, pixelOffset, imgPoint)



worldPoint = zeros(3,size(imgPoint,1));
worldPoint(3,:) = focal;
% for i = 1:size(imgPoint,1)
%     worldPoint(1,i) = imgPoint(i,1)*pixelOffset - principal(1)*pixelOffset;
%     worldPoint(2,i) = -(imgPoint(i,2)*pixelOffset - principal(2)*pixelOffset);
% end

worldPoint(1,:)= (imgPoint(:,1)*pixelOffset - principal(1)*pixelOffset)';
worldPoint(2,:)= ((imgPoint(:,2)*pixelOffset - principal(2)*pixelOffset))';
