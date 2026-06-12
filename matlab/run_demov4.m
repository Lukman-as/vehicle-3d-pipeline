clear;
close all;
clc;
load('Q1798zoom0angle1_fisheye_params.mat');
I=imread('v.png');
%[img1,camIntrisics] = undistortFisheyeImage(I,Q1798zoom0angle1_fisheye.Intrinsics,'OutputView','valid','ScaleFactor',[2,2]);
[img1,camIntrisics] = undistortFisheyeImage(I,Q1798zoom0angle1_fisheye.Intrinsics,'OutputView','valid');

% img2= undistortFisheyeImage(I,Q1798zoom0angle1_fisheye.Intrinsics,'OutputView','same');
% figure;
% imshowpair(img2, I, 'montage');
% figure;
% imshowpair(img1, I, 'montage')

load('sample2Dpoints.mat');  %points that you annotate on the image
% sample2D=flip(sample2D);
pixelSize=0.001;  

load('model3D.mat');


%camera is located at (0,0,0), ground level is at (0,0,-4.8)
focal=camIntrisics.FocalLength(1);
fovHorizontal=2*atand(camIntrisics.ImageSize(2)/2/camIntrisics.FocalLength(2));
fovVertical=2*atand(camIntrisics.ImageSize(1)/2/camIntrisics.FocalLength(1));
width=camIntrisics.ImageSize(2);
height=camIntrisics.ImageSize(1);
heightofPole=4.8;%4.8 meters
pp=camIntrisics.PrincipalPoint;
K=[focal,0,pp(1);0,focal,pp(2);0,0,1]; %Camera intrinsic values

load('cameraExtrinsic.mat');
vp_f=vp_info.vp;   %Camera extrinsic values

points2D=projectworld2Image(vp_f',focal*pixelSize,pp,pixelSize)';
horizonLine=[points2D(:,1),points2D(:,3)];
vphorizon=findVP(horizonLine,sample2D);

%using symmetry lines to find the vanishing point 
vectorNormal=tireGround(1:2)-vphorizon;
vectorNormal=vectorNormal/norm(vectorNormal);

vphorizonGS = mapIMGPointsToWorldRegular(pp, focal*pixelSize, pixelSize, vphorizon);
carBodyNorm=vphorizonGS/norm(vphorizonGS);

figure
scatter(vphorizon(1),vphorizon(2),100,'c','filled'); %Show the horizon point
hold on
imshow(img1)
hold on
scatter(sample2D(:,1),sample2D(:,2),100,'r','filled'); %Show the sample 2D points
scatter(tireGround(:,1),tireGround(:,2),100,'b','filled'); %Show the tire ground points
%scatter(center(:,1),center(:,2),120,'y','filled');
axis equal
xlabel('X')
ylabel('Y')

%DONE WITH THE 2D POINTS ON IMAGES


%the matlab coordinate used different space where y direction is revsersed
% points3D=[points2D;ones(1,3)*focal];
% points3D(1,:)=points3D(1,:)-pp(1);
% points3D(2,:)=points3D(2,:)-pp(2);
% points3D(:,1)=points3D(:,1)/norm(points3D(:,1));
% points3D(:,2)=points3D(:,2)/norm(points3D(:,2));
% points3D(:,3)=points3D(:,3)/norm(points3D(:,3));
% rotM=points3D;

%3D Roration matrix
rotM=vp_f;
rotM(:,2)=-rotM(:,2);
rotM(2,:)=-rotM(2,:);
rotationMatrix=rotM;

% rotationMatrix=vp_f;
H=K*(rotationMatrix);
Hinv=inv(H);

sample2DH=[sample2D,ones(size(sample2D,1),1)];

%The first one point is ground points which has Z=-4.8
tireG=[(tireGround-pp),1];
X = linsolve(H,tireG');
w=-heightofPole/X(3);

groundPoints=X'*w;

vphorizon2D=[vphorizon,1];
vphorizon3D=vphorizon2D*(Hinv');
vphorizon3D=vphorizon3D./vphorizon3D(:,3)*(-heightofPole);


plane=computePlane(groundPoints,carBodyNorm');  %Compute plane

leftPlanePoints=[1,3,5,7,9];
rightPlanePoints=[2,4,6,8,10];



s=-plane(4)./(plane(1)*estModelpre(rightPlanePoints,1)+plane(2)*estModelpre(rightPlanePoints,2)+plane(3)*estModelpre(rightPlanePoints,3));
estModel3DRight=estModelpre(rightPlanePoints,:).*s;

sl=estModel3DRight(:,3)./estModelpre(leftPlanePoints,3);
estModel3DLeft=estModelpre(leftPlanePoints,:).*sl;


figure
scatter3(estModel3DRight(:,1),estModel3DRight(:,2),estModel3DRight(:,3),100,'r','filled');
hold on
scatter3(estModel3DLeft(:,1),estModel3DLeft(:,2),estModel3DLeft(:,3),100,'r','filled');
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
pause(0.1)
pointleftCorrected=projPointOnLine3d(estModel3DLeft(1,:), [estModel3DRight repmat(uv,size(estModel3DRight,1),1)]);

allPoints=[estModel3DRight;estModel3DLeft];
% allPoints=[estModel3DRight;pointleftCorrected];
allPointstemp=allPoints;
allPointstemp(:,3)=-heightofPole;
allPoints=[allPoints;allPointstemp];
shp = alphaShape(allPoints(:,1),allPoints(:,2),allPoints(:,3),2);
figure
plot(shp)
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
pause(0.1)
allPoints2D=H*allPoints';
allPoints2D=allPoints2D./allPoints2D(3,:);

points2D=projectworld2Image(vp_f',focal*pixelSize,pp,pixelSize)';

figure
scatter(points2D(1,:),points2D(2,:),100,'k','filled')
hold on
imshow(img1)
hold on
scatter(points2D(1,:),points2D(2,:),100,'k','filled')

pause(0.1)

middlePoints3D=[];
for i=1:5
    middlePoints3D=[middlePoints3D;mean(allPoints2D(1:2,[i,i+5]),2)'];
end
[elements,nodes] = boundaryFacets(shp);
nodes2D=nodes*H';

figure
imshow(img1)
hold on
for i=1:size(elements,1)
    patch(nodes2D(elements(i,:),1)./nodes2D(elements(i,:),3),nodes2D(elements(i,:),2)./nodes2D(elements(i,:),3),'b');
end
scatter(sample2D(:,1),sample2D(:,2),100,'r','filled');
scatter(tireGround(:,1),tireGround(:,2),100,'b','filled');
scatter(allPoints2D(1,1:10),allPoints2D(2,1:10),100,'m','filled')
%scatter(allPoints2D(1,11:20),allPoints2D(2,11:20),100,'y','filled')
% scatter(middlePoints(:,1),middlePoints(:,2),70,'k','filled');
scatter(middlePoints3D(:,1),middlePoints3D(:,2),70,'c','filled');
scatter(vphorizon(1),vphorizon(2),100,'r','filled')
axis equal
xlabel('X')
ylabel('Y')






