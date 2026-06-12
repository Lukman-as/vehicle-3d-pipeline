
%% Visualize 3D bounding box on image

corners_bbox_2D = k_matrix*R*corners_bbox_world
corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:)

edges = [
    1 2;
    2 3;
    3 4;
    4 1;
    5 6;
    6 7;
    7 8;
    8 5;
    1 5;
    2 6;
    3 7;
    4 8;
];

figure
hold on
imshow(image)
%Yaxis vanishing
hold on
scatter(anno_sympoints_homo_and_ground(1,:),anno_sympoints_homo_and_ground(2,:),40,'r','filled'); %Plot the sample 2D 
% scatter(all_3d_points_to_2D(1,:),all_3d_points_to_2D(2,:),20,'blue','filled');
scatter(corners_bbox_2D(1,1),corners_bbox_2D(2,1),50,'blue','filled');
% scatter(corners_bbox_2D(1,2),corners_bbox_2D(2,2),50,'blue','filled');
for i = 1:size(edges,1)
    x_coor = [corners_bbox_2D(1, edges(i,1)) corners_bbox_2D(1, edges(i,2))];
    y_coor = [corners_bbox_2D(2, edges(i,1)) corners_bbox_2D(2, edges(i,2))];
    line(x_coor, y_coor,'Color','green','LineWidth',1)
end
