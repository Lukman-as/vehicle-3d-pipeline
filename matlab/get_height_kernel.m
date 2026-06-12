function mounting_height_gt = get_height_kernel(fobj, k_matrix, R,origin_2D)
    tire_ray = inv(k_matrix*R) * origin_2D;
    F = @(y) abs(y-fobj([y/tire_ray(2)*tire_ray(1),y/tire_ray(2)*tire_ray(3)]));
    y0 = [7];
    options = optimoptions('fsolve','FunctionTolerance',1e-20,'StepTolerance', ...
        1e-20,'Algorithm','trust-region-dogleg','Display','off');
    [mounting_height_gt,fval,exit_flag, output] = fsolve(F,y0,options);

    %Assertion after solving cannot deviate too much in both 3D and 2D
    pred_point_3D = [mounting_height_gt/tire_ray(2)*tire_ray(1),mounting_height_gt, mounting_height_gt/tire_ray(2)*tire_ray(3)]';
    %This has to be close to the ground
    a = fobj([pred_point_3D(1), pred_point_3D(3)]);
    b = pred_point_3D(2);
    assert(abs(a-b)<2, "The error in 3D cannot be more than 2cm");

    pred_point = k_matrix*R*pred_point_3D;
    pred_point = pred_point/pred_point(3);
    error = ((sum((pred_point-origin_2D).^2,'all'))...
        /size(pred_point,2))...
        .^(0.5);
    assert(error < 1, 'Reprojected point cannot be deviate too much from annotated point, no more than 1 pixel');