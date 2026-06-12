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
figure;
imshowpair(img1, I, 'montage')

load('sample2Dpoints.mat'); 
% sample2D=flip(sample2D);
pixelSize=0.001;

load('model3D.mat'); %Template


%camera is located at (0,0,0), ground level is at (0,0,-4.8)
focal=camIntrisics.FocalLength(1);
fovHorizontal=2*atand(camIntrisics.ImageSize(2)/2/camIntrisics.FocalLength(2));
fovVertical=2*atand(camIntrisics.ImageSize(1)/2/camIntrisics.FocalLength(1));
width=camIntrisics.ImageSize(2);
height=camIntrisics.ImageSize(1);
heightofPole=4.8;%4.8 meters
pp=camIntrisics.PrincipalPoint;
K=[focal,0,pp(1);0,focal,pp(2);0,0,1];

load('cameraExtrinsic.mat');

%Precalculated with Yiming algorithm, CVPR pending, new algorithm
%Process the lines, 3 vanishing points of image, xyz of camera 
vp_f=vp_info.vp;

%Project 3D vanishhing points into 2D
points2D=projectworld2Image(vp_f',focal*pixelSize,pp,pixelSize)';

%Connect two horizontal vanishing points to make horizon line
horizonLine=[points2D(:,1),points2D(:,3)];

%Function find the point of the horizon line, perpendicular to body of the
%car, on the horizon line, blue dot
vphorizon=findVP(horizonLine,sample2D); 
%Do you have to change the code in herre? I think you do
%Because the least square in Elder's is homogenous form and then convert to
%euclidean to do the least square

% horizonLine = [horizonLine;focal focal] this part is to change to
% homogenous form as in ELDER, may have to verify

% Ns = inv(K*vp_f)*transpose([vphorizon 1]) Calculating Ns
% Verify about adding 1 as well.

%using symmetry lines to find the vanishing point 
%Pick one tire point, connect the two blue dots then you have a line
%parellel to the ground level
% Normal vector of the car body
vectorNormal=tireGround(1:2)-vphorizon;
vectorNormal=vectorNormal/norm(vectorNormal); %Normalization, unit vector, length 1

%
vphorizon3D = mapIMGPointsToWorldRegular(pp, focal*pixelSize, pixelSize, vphorizon);

%%% GONNA TO BE CHANGED


figure
%scatter(vphorizon(1),vphorizon(2),400,'blue','filled');
%hold on
imshow(img1)
hold on
scatter(points2D(1,1),points2D(2,1),400,'yellow','filled');
scatter(points2D(1,2),points2D(2,1),400,'yellow','filled');
scatter(points2D(1,3),points2D(2,3),400,'yellow','filled');
hold on
scatter(vphorizon(1),vphorizon(2),400,'blue','filled');
hold on
scatter(sample2D(:,1),sample2D(:,2),100,'r','filled');
scatter(tireGround(:,1),tireGround(:,2),100,'b','filled');
%scatter(center(:,1),center(:,2),120,'y','filled');
axis equal
xlabel('X')
ylabel('Y')

%the matlab coordinate used different space where y direction is revsersed
% Reverse of Positive and Negative of y axis, different from the common
% world coordinate
% points3D=[points2D;ones(1,3)*focal];
% points3D(1,:)=points3D(1,:)-pp(1);
% points3D(2,:)=points3D(2,:)-pp(2);
% points3D(:,1)=points3D(:,1)/norm(points3D(:,1));
% points3D(:,2)=points3D(:,2)/norm(points3D(:,2));
% points3D(:,3)=points3D(:,3)/norm(points3D(:,3));
% rotM=points3D;
rotM=vp_f;
rotM(:,2)=-rotM(:,2);
rotM(2,:)=-rotM(2,:);
rotationMatrix=rotM;

% rotationMatrix=vp_f;
H=K*(rotationMatrix);
Hinv=inv(H);

sample2DH=[sample2D,ones(size(sample2D,1),1)]; % Convert point to homo

%The first one point is ground points which has Z=-4.8
% Assuming camera at 0,0,0 ground 0,-4.8,0 tireGround point
% Different from Dr James code

tireG=[(tireGround-pp)/focal,1]; 
X = linsolve(H,tireG'); 
w=-heightofPole/X(3);

groundPoints=X'*w;

vphorizon2D=[vphorizon,1];
vphorizon3D=vphorizon2D*(Hinv');
vphorizon3D=vphorizon3D./vphorizon3D(:,3)*(-heightofPole);

temp=(vphorizon3D-groundPoints);
uv=temp/norm(temp);

estModelpre=sample2DH*(Hinv');
centrepre=[center,1]*(Hinv');
plane=computePlane(groundPoints,uv);

leftPlanePoints=[1,3,5,7,9];
rightPlanePoints=[2,4,6,8,10];

s=-plane(4)./(plane(1)*estModelpre(rightPlanePoints,1)+plane(2)*estModelpre(rightPlanePoints,2)+plane(3)*estModelpre(rightPlanePoints,3));
estModel3DRight=estModelpre(rightPlanePoints,:).*s;

sl=estModel3DRight(:,3)./estModelpre(leftPlanePoints,3);
estModel3DLeft=estModelpre(leftPlanePoints,:).*sl;

%THE END OF 2.1


figure
%VISUALIZATION OF THE POINTS 


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






