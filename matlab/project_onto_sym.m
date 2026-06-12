function points_on_sym = project_onto_sym(points,ds, sym_normal_unit)
    points_on_sym  = zeros(size(points,1), size(points,2));
    for k = 1:size(points,2) 
        point = points(:,k);
        points_on_sym(:,k) = point + (-ds-dot(point,sym_normal_unit))*sym_normal_unit;
    end
end