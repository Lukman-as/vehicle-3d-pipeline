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

load('sample2Dpoints.mat');
% sample2D=flip(sample2D);
pixelSize=0.001;

load('model3D.mat');



% figure
% hold on;
% scatter3(model3D(7:16,1),model3D(7:16,2),model3D(7:16,3),100,'r','filled');
% plot3([model3D(1,1),model3D(2,1)],[model3D(1,2),model3D(2,2)],[model3D(1,3),model3D(2,3)],'b','linewidth',3);
% plot3([model3D(2,1),model3D(4,1)],[model3D(2,2),model3D(4,2)],[model3D(2,3),model3D(4,3)],'b','linewidth',3);
% plot3([model3D(3,1),model3D(4,1)],[model3D(3,2),model3D(4,2)],[model3D(3,3),model3D(4,3)],'b','linewidth',3);
% plot3([model3D(3,1),model3D(1,1)],[model3D(3,2),model3D(1,2)],[model3D(3,3),model3D(1,3)],'b','linewidth',3);
% plot3([model3D(3,1),model3D(5,1)],[model3D(3,2),model3D(5,2)],[model3D(3,3),model3D(5,3)],'b','linewidth',3);
% plot3([model3D(4,1),model3D(6,1)],[model3D(4,2),model3D(6,2)],[model3D(4,3),model3D(6,3)],'b','linewidth',3);
% plot3([model3D(5,1),model3D(6,1)],[model3D(5,2),model3D(6,2)],[model3D(5,3),model3D(5,3)],'b','linewidth',3);
% plot3([model3D(7,1),model3D(5,1)],[model3D(7,2),model3D(5,2)],[model3D(7,3),model3D(5,3)],'b','linewidth',3);
% plot3([model3D(8,1),model3D(6,1)],[model3D(8,2),model3D(6,2)],[model3D(8,3),model3D(6,3)],'b','linewidth',3);
% plot3([model3D(8,1),model3D(7,1)],[model3D(8,2),model3D(7,2)],[model3D(8,3),model3D(7,3)],'b','linewidth',3);
% plot3([model3D(7,1),model3D(9,1)],[model3D(7,2),model3D(9,2)],[model3D(7,3),model3D(9,3)],'b','linewidth',3);
% plot3([model3D(8,1),model3D(10,1)],[model3D(8,2),model3D(10,2)],[model3D(8,3),model3D(10,3)],'b','linewidth',3);
% plot3([model3D(9,1),model3D(10,1)],[model3D(9,2),model3D(10,2)],[model3D(9,3),model3D(10,3)],'b','linewidth',3);
% plot3([model3D(11,1),model3D(9,1)],[model3D(11,2),model3D(9,2)],[model3D(11,3),model3D(9,3)],'b','linewidth',3);
% plot3([model3D(12,1),model3D(10,1)],[model3D(12,2),model3D(10,2)],[model3D(12,3),model3D(10,3)],'b','linewidth',3);
% plot3([model3D(11,1),model3D(12,1)],[model3D(11,2),model3D(12,2)],[model3D(11,3),model3D(12,3)],'b','linewidth',3);
% plot3([model3D(11,1),model3D(13,1)],[model3D(11,2),model3D(13,2)],[model3D(11,3),model3D(13,3)],'b','linewidth',3);
% plot3([model3D(14,1),model3D(12,1)],[model3D(14,2),model3D(12,2)],[model3D(14,3),model3D(12,3)],'b','linewidth',3);
% plot3([model3D(14,1),model3D(13,1)],[model3D(14,2),model3D(13,2)],[model3D(14,3),model3D(13,3)],'b','linewidth',3);
% plot3([model3D(13,1),model3D(15,1)],[model3D(13,2),model3D(15,2)],[model3D(13,3),model3D(15,3)],'b','linewidth',3);
% plot3([model3D(14,1),model3D(16,1)],[model3D(14,2),model3D(16,2)],[model3D(14,3),model3D(16,3)],'b','linewidth',3);
% plot3([model3D(15,1),model3D(16,1)],[model3D(15,2),model3D(16,2)],[model3D(15,3),model3D(16,3)],'b','linewidth',3);
% axis equal
% axis tight
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

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
vp_f=vp_info.vp;
points2D=projectworld2Image(vp_f',focal*pixelSize,pp,pixelSize)';
horizonLine=[points2D(:,1),points2D(:,3)];
vphorizon=findVP(horizonLine,sample2D);
%using symmetry lines to find the vanishing point 

figure
scatter(vphorizon(1),vphorizon(2),100,'c','filled');
hold on
imshow(img1)
hold on
scatter(sample2D(:,1),sample2D(:,2),100,'r','filled');
scatter(tireGround(:,1),tireGround(:,2),100,'b','filled');
scatter(center(:,1),center(:,2),120,'y','filled');

axis equal
xlabel('X')
ylabel('Y')



points3D=[points2D;ones(1,3)*focal];
points3D(1,:)=points3D(1,:)-pp(1);
points3D(2,:)=points3D(2,:)-pp(2);
points3D(:,1)=points3D(:,1)/norm(points3D(:,1));
points3D(:,2)=points3D(:,2)/norm(points3D(:,2));
points3D(:,3)=points3D(:,3)/norm(points3D(:,3));
rotM=points3D;
rotM(:,1)=-points3D(:,1);
rotM(:,2)=-points3D(:,3);
rotM(:,3)=-points3D(:,2);

rotationMatrix=rotM;

% rotationMatrixNew=cameraOptimization(rotationMatrix,sample2D,center,K);
H=K*(rotationMatrix);
Hinv=inv(H);

sample2DH=[sample2D,ones(size(sample2D,1),1)];
% sample2DH(:,2)=-sample2DH(:,2);
%The first one point is ground points which has Z=-4.8
tireGround=[tireGround,1];
tireGround3D=tireGround*(Hinv');
groundPoints=tireGround3D./tireGround3D(:,3)*(-heightofPole);

estModelpre=sample2DH*(Hinv');
centrepre=[center,1]*(Hinv');

pairs=zeros(size(estModelpre,1)/2,2);
for i=1:length(pairs)
symmetry=estModelpre(i*2-1:2*i,:);
norm1=symmetry(1,:)-symmetry(2,:)*symmetry(1,3)/symmetry(2,3);
norm1=norm1(1:2)/norm(norm1(1:2));
pairs(i,:)=norm1;
end

meanSymmetry=mean(pairs);
meanSymmetry3D=[meanSymmetry,0];
plane=computePlane(groundPoints,meanSymmetry3D);

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
pointleftCorrected=projPointOnLine3d(estModel3DLeft(1,:), [estModel3DRight repmat(meanSymmetry3D,size(estModel3DRight,1),1)]);
groundPlane=[0,0,-heightofPole,1,0,0,0,1,0];


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


figure
imshow(img1)
hold on
scatter(sample2D(:,1),sample2D(:,2),100,'r','filled');
scatter(tireGround(:,1),tireGround(:,2),100,'b','filled');
scatter(allPoints2D(1,1:10),allPoints2D(2,1:10),100,'m','filled')
scatter(allPoints2D(1,11:20),allPoints2D(2,11:20),100,'y','filled')
scatter(middlePoints(:,1),middlePoints(:,2),70,'k','filled');
scatter(middlePoints3D(:,1),middlePoints3D(:,2),70,'c','filled');
scatter(vest(1),vest(2),100,'r','filled')
axis equal
xlabel('X')
ylabel('Y')




