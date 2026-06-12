function [percent_overlap,count_inside_gt]  = get_point_overlap(ptCloud_world,corners_bbox_world,predicted_corners)
%     [k1,av1] = convhull(corners_bbox_world(1,:),corners_bbox_world(2,:),corners_bbox_world(3,:))
%     trisurf(k1,corners_bbox_world(1,:),corners_bbox_world(2,:),corners_bbox_world(3,:),'FaceColor','cyan')
%     axis equal
    testpts = ptCloud_world';
    indice = inhull(testpts,corners_bbox_world');
    count_inside_gt = sum(indice);
    inside_gt_pts = testpts(indice,:);
    indice = inhull(inside_gt_pts,predicted_corners');
    count_inside_pred = sum(indice);
%     inside_pred_pts = inside_gt_pts(indice,:)
    percent_overlap = count_inside_pred/count_inside_gt;
end