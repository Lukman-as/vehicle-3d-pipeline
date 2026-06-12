function b_pred = gaussian_pred(car_model_param, scenario, a_value)
    %Get the params of that scenarios
    params = car_model_param.(scenario);
    a_size = 1;
    b_size = 1;
    start_pos = 1;
    mu_a = params(start_pos:a_size);
    start_pos = start_pos + a_size;
    params(start_pos:start_pos+(a_size*b_size)-1);
    cov_ab = reshape(params(start_pos:start_pos+(a_size*b_size)-1),[b_size,a_size])'; %Different from python
    start_pos = start_pos + a_size*b_size;
    cov_bb_inv = reshape(params(start_pos:start_pos+(b_size*b_size)-1),[b_size,b_size])';
    start_pos = start_pos + b_size*b_size;
    mu_b = params(start_pos:start_pos+(b_size)-1);
    vars = split(scenario, '__');
    b_vars = strjoin(vars(2),'');
    a_vars = strjoin(vars(1),'');
    b_vars = split(b_vars, '_');
    a_vars = split(a_vars,'_');
    %Get vector into b_vector
    b_vector = [];
    b_vector = [b_vector a_value*100];
    b_pred = (mu_a'+cov_ab*cov_bb_inv*(b_vector'-mu_b'))/100;
end