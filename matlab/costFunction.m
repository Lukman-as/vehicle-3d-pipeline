function output=costFunction(arrayMatrix,sample2D,center,K)

rotationMatrix=reshape(arrayMatrix(:),3,3);
H=K*(rotationMatrix);
oldMatrix=inv(H);
sample2DH=[sample2D,ones(size(sample2D,1),1)];
estModelpre=sample2DH*(oldMatrix');
centrepre=[center,1]*(oldMatrix');
middlePoints3D=zeros(size(sample2D,1)/2,3);
counter=1;
for i=1:2:size(sample2D,1)
    middlePoints3D(counter,:)=mean(estModelpre(i:i+1,:));
    counter=counter+1;
end

d=zeros(size(middlePoints3D,1)-1,1);
for i=1:size(middlePoints3D,1)-1
    d(i) = distancePointLine3d(centrepre, [middlePoints3D(i,:),middlePoints3D(i+1,:)-middlePoints3D(i,:)]);
end
output=min(d);
end