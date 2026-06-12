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
sample2D=flip(sample2D);
pixelSize=0.001;

load('model3D.mat');

figure
hold on;
scatter3(model3D(7:16,1),model3D(7:16,2),model3D(7:16,3),100,'r','filled');
plot3([model3D(1,1),model3D(2,1)],[model3D(1,2),model3D(2,2)],[model3D(1,3),model3D(2,3)],'b','linewidth',3);
plot3([model3D(2,1),model3D(4,1)],[model3D(2,2),model3D(4,2)],[model3D(2,3),model3D(4,3)],'b','linewidth',3);
plot3([model3D(3,1),model3D(4,1)],[model3D(3,2),model3D(4,2)],[model3D(3,3),model3D(4,3)],'b','linewidth',3);
plot3([model3D(3,1),model3D(1,1)],[model3D(3,2),model3D(1,2)],[model3D(3,3),model3D(1,3)],'b','linewidth',3);
plot3([model3D(3,1),model3D(5,1)],[model3D(3,2),model3D(5,2)],[model3D(3,3),model3D(5,3)],'b','linewidth',3);
plot3([model3D(4,1),model3D(6,1)],[model3D(4,2),model3D(6,2)],[model3D(4,3),model3D(6,3)],'b','linewidth',3);
plot3([model3D(5,1),model3D(6,1)],[model3D(5,2),model3D(6,2)],[model3D(5,3),model3D(5,3)],'b','linewidth',3);
plot3([model3D(7,1),model3D(5,1)],[model3D(7,2),model3D(5,2)],[model3D(7,3),model3D(5,3)],'b','linewidth',3);
plot3([model3D(8,1),model3D(6,1)],[model3D(8,2),model3D(6,2)],[model3D(8,3),model3D(6,3)],'b','linewidth',3);
plot3([model3D(8,1),model3D(7,1)],[model3D(8,2),model3D(7,2)],[model3D(8,3),model3D(7,3)],'b','linewidth',3);
plot3([model3D(7,1),model3D(9,1)],[model3D(7,2),model3D(9,2)],[model3D(7,3),model3D(9,3)],'b','linewidth',3);
plot3([model3D(8,1),model3D(10,1)],[model3D(8,2),model3D(10,2)],[model3D(8,3),model3D(10,3)],'b','linewidth',3);
plot3([model3D(9,1),model3D(10,1)],[model3D(9,2),model3D(10,2)],[model3D(9,3),model3D(10,3)],'b','linewidth',3);
plot3([model3D(11,1),model3D(9,1)],[model3D(11,2),model3D(9,2)],[model3D(11,3),model3D(9,3)],'b','linewidth',3);
plot3([model3D(12,1),model3D(10,1)],[model3D(12,2),model3D(10,2)],[model3D(12,3),model3D(10,3)],'b','linewidth',3);
plot3([model3D(11,1),model3D(12,1)],[model3D(11,2),model3D(12,2)],[model3D(11,3),model3D(12,3)],'b','linewidth',3);
plot3([model3D(11,1),model3D(13,1)],[model3D(11,2),model3D(13,2)],[model3D(11,3),model3D(13,3)],'b','linewidth',3);
plot3([model3D(14,1),model3D(12,1)],[model3D(14,2),model3D(12,2)],[model3D(14,3),model3D(12,3)],'b','linewidth',3);
plot3([model3D(14,1),model3D(13,1)],[model3D(14,2),model3D(13,2)],[model3D(14,3),model3D(13,3)],'b','linewidth',3);
plot3([model3D(13,1),model3D(15,1)],[model3D(13,2),model3D(15,2)],[model3D(13,3),model3D(15,3)],'b','linewidth',3);
plot3([model3D(14,1),model3D(16,1)],[model3D(14,2),model3D(16,2)],[model3D(14,3),model3D(16,3)],'b','linewidth',3);
plot3([model3D(15,1),model3D(16,1)],[model3D(15,2),model3D(16,2)],[model3D(15,3),model3D(16,3)],'b','linewidth',3);
axis equal
axis tight
xlabel('X');
ylabel('Y');
zlabel('Z');

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

% vp_f(:,1)=vp_info.vp(:,3);
% vp_f(:,2)=vp_info.vp(:,1);
% vp_f(:,3)=vp_info.vp(:,2);


% [euler_angles, ~] = findEulerAnglesBetween_CoordinateFrames(eye(3), vp_f, 'MW_ADJ');

% phi = euler_angles(1); 
% theta = euler_angles(2); 
% psi = euler_angles(3); 
% 
% F1(1,1) = cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
% F1(1,2) = -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
% F1(1,3) = sin(phi)*cos(theta);
% F1(2,1) = cos(theta)*sin(psi);
% F1(2,2) = cos(theta)*cos(psi);
% F1(2,3) = -sin(theta);
% F1(3,1) = -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
% F1(3,2) = sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
% F1(3,3) = cos(phi)*cos(theta);

%rad2deg(euler_angles)
%YXZ: Roll, Pitch, Yaw
% rotm = eul2rotm([euler_angles(3),euler_angles(1),euler_angles(2)]);
% eul = rad2deg(rotm2eul(vp_f))
rotationMatrix=rotM;

% Roll(in degrees) = -52.052566576100666, pitch = 69.8905074258095, yaw = 143.6885514933477,
% eul=[143.6885514933477,-52.052566576100666,69.8905074258095];
%Euler angle in matlab require ZYX order
% rotm = eul2rotm(deg2rad(eul));
%Matlab uses y-down/z-forward camera coordinate systems which equivalent to
%x2=x1, y2=-y1,z2=-z1
%H=K*[rotm',zeros(3,1)];%camera locate at 0,0,0, no translation so we can
%simplify the matrix as

H=K*(rotationMatrix);
Hinv=inv(H);

lineSamples=[];
for i=1:2:size(sample2D,1)
    p = polyfit(sample2D(i:i+1,1),sample2D(i:i+1,2),1);
    %y=p(1)x+p(2)
    lineSamples=[lineSamples;p(1) -1 p(2)];
end
vest = LSPointLines(lineSamples');

sample2DH=[sample2D,ones(size(sample2D,1),1)];
% sample2DH(:,2)=-sample2DH(:,2);
%The first one point is ground points which has Z=-4.8
tireGround=[tireGround,1];
tireGround3D=tireGround*(Hinv');
groundPoints=tireGround3D./tireGround3D(:,3)*(-heightofPole);

estModelpre=sample2DH*(Hinv');
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
% rightPlanePoints=[1,3,5,7,9];
% leftPlanePoints=[2,4,6,8,10];

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

allPoints=[estModel3DRight;pointleftCorrected];
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

figure
imshow(img1)
hold on
scatter(sample2D(:,1),sample2D(:,2),100,'r','filled');
scatter(tireGround(:,1),tireGround(:,2),100,'b','filled');
scatter(allPoints2D(1,:),allPoints2D(2,:),100,'m','filled')
scatter(vest(1),vest(2),100,'y','filled')
axis equal
xlabel('X')
ylabel('Y')



