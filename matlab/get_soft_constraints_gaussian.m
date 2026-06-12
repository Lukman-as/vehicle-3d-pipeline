function [extreme_front_point, extreme_rear_point, extreme_height_point] = ...
    get_soft_constraints_gaussian(x0_order, x0, all_3D_points,all_annos, fieldname,car_model_param,id_to_tire)
    %Extract tire points
    start_pos = 0;
    extracted_car_tires = struct;
    for i = 1:size(all_annos.(fieldname).tire_ids,2)
        extracted_car_tires.(id_to_tire(all_annos.(fieldname).tire_ids(i))) = all_3D_points(:,start_pos+i);
    end

    %Extract wheelbase
    if all_annos.(fieldname).wheelbase ~= -1
        %Extract from current optimization:
        if isfield(extracted_car_tires,"DF") && isfield(extracted_car_tires,"DR")
            wheelbase = norm(extracted_car_tires.DF - extracted_car_tires.DR);
        elseif isfield(extracted_car_tires,"PF") && isfield(extracted_car_tires,"PR")
            wheelbase = norm(extracted_car_tires.PF - extracted_car_tires.PR);
        end
    else
        wheelbase = -1;
    end


    %Extract width
    overall_width = x0(1:x0_order.w_unit);
%     use_height = false;
    if wheelbase ~= -1
%         if use_height
%             scenario = 'RH_FH__OH_OW_WB';
%             a_size = 2;
%             b_size = 3;
%         else
        scenario = 'RH_FH_OH__OW_WB';
        a_size = 3;
        b_size = 2;
    else
%         if use_height
%             scenario = 'RH_FH_WB__OH_OW';
%             a_size = 3;
%             b_size = 2;
%         else
        scenario = 'RH_FH_OH_WB__OW';
        a_size = 4;
        b_size = 1;
    end


    %Get the params of that scenarios
    params = car_model_param.(scenario);
    %Reshaping params
    start_pos = 1;
    mu_a = params(start_pos:a_size);
    start_pos = start_pos + a_size;
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
    for i=1:size(b_vars,1)
        word = strjoin(b_vars(i),'');
        if strcmp(word,'OH')
           b_vector = [b_vector overall_height*100];
        elseif strcmp(word,'OW')
           b_vector = [b_vector overall_width*100];
        elseif strcmp(word,'WB')
           b_vector = [b_vector wheelbase*100];
        end
    end

%    overall_width
   preds = mu_a'+cov_ab*cov_bb_inv*(b_vector'-mu_b');
   %Get vector into a_vector
   for i=1:size(a_vars,1)
        word = strjoin(a_vars(i),'');
        if strcmp(word,'RH')
           rear_hang = preds(i)/100; %Unit conversion
        elseif strcmp(word,'FH') 
           front_hang = preds(i)/100;
        elseif strcmp(word,'OH')
           overall_height = preds(i)/100;
        elseif strcmp(word,'WB')
           assert(wheelbase == -1, 'Wheelbase is unknown to be predicted');
           wheelbase = preds(i)/100;
        end
   end

%     %Get tire points
%     start_pos = 0;
%     extracted_car_tires = struct;
%     for i = 1:size(all_annos.(fieldname).tire_ids,2)
% %         all_3D_points(:,start_pos+i)
%         extracted_car_tires.(id_to_tire(all_annos.(fieldname).tire_ids(i))) = all_3D_points(:,start_pos+i);
%     end
    
    %Get extremal points
    if isfield(extracted_car_tires,"DF") || isfield(extracted_car_tires,"PF")
        if isfield(extracted_car_tires,"DF")
            point = extracted_car_tires.DF;
        elseif isfield(extracted_car_tires,"PF")
            point = extracted_car_tires.PF;
        end
        extreme_front_point = point -[front_hang,0,0,0]';
    else
        if isfield(extracted_car_tires,"PR")
            point =  extracted_car_tires.PR;
        elseif isfield(extracted_car_tires,"DR")
            point =  extracted_car_tires.DR;
        end
        assert(wheelbase ~= -1)
        extreme_front_point = point - [wheelbase,0,0,0]' - [front_hang,0,0,0]';
    end
    
    if isfield(extracted_car_tires,"PR") || isfield(extracted_car_tires,"DR")
        if isfield(extracted_car_tires,"PR")
            point = extracted_car_tires.PR;
        elseif isfield(extracted_car_tires,"DR")
            point = extracted_car_tires.DR;
        end
        extreme_rear_point = point + [rear_hang,0,0,0]'; %Rear gotta add due to direction
    else
        if isfield(extracted_car_tires,"DF")
            point =  extracted_car_tires.DF; 
        elseif isfield(extracted_car_tires,"PF")
            point =  extracted_car_tires.PF;
        end
        assert(wheelbase ~= -1)
        extreme_rear_point = point + [wheelbase,0,0,0]' + [rear_hang,0,0,0]';
    end

    if overall_height <=0
%         overall_height = abs(min(all_3D_points(2,:))); %Prevent negative case only
        overall_height = 3.05; %Prevent negative case only
    end
    
    assert(overall_height > 0, 'Heigt gotta be larger than 0')
    extreme_height_point = [0 -overall_height 0 1]';

    %Add extreme points to all points
%     extreme_front_point
%     extreme_rear_point
end