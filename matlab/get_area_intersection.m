function [area_truth, area_predicted, area_intersection] = get_area_intersection(predicted_ground_corners,truth_ground_corners)
%     a = predicted_ground_corners;
%     b = truth_ground_corners;
%     dx = min(max(a(1,:)), max(b(1,:))) - max(min(a(1,:)), min(b(1,:)))
%     dy = min(max(a(2,:)), max(b(2,:))) - max(min(a(2,:)), min(b(2,:)))
%     if dx>=0 && dy>=0
%         area = dx*dy;
%     else
%         area = 0;
%     end

    predicted_ground_corners = [predicted_ground_corners predicted_ground_corners(:,1)];
    truth_ground_corners = [truth_ground_corners truth_ground_corners(:,1)];
    
%   
%     figure
%     hold on
%     scatter(predicted_ground_corners(1,:), predicted_ground_corners(2,:),20,'green','filled')
%     scatter(truth_ground_corners(1,:), truth_ground_corners(2,:),20,'red','filled')  

%     [xi,yi] = polyxpoly(predicted_ground_corners(1,:),predicted_ground_corners(2,:),...
%         truth_ground_corners(1,:),truth_ground_corners(2,:))
%     
    
    
    pgon1 = polyshape(predicted_ground_corners(1,:),predicted_ground_corners(2,:));
    pgon2 = polyshape(truth_ground_corners(1,:),truth_ground_corners(2,:));
    polyout = intersect(pgon1,pgon2);
%     
%     mapshow(predicted_ground_corners(1,:), predicted_ground_corners(2,:),'DisplayType','polygon','LineStyle','none')
%     mapshow(truth_ground_corners(1,:), truth_ground_corners(2,:),'Marker','+')
%     mapshow(xi,yi,'Marker','o','Color', 'red')
    
    if false
        figure;
        hold on
        plot(pgon1)
        plot(pgon2)
        plot(polyout)
        axis equal
        axis on
        xlabel('X')
        ylabel('Y')
    end
    area_predicted = area(pgon1);
    area_truth = area(pgon2);
    area_intersection = area(polyout);
end