%% SETUP
data_path = '../data/raw';
gen_path = '../data/gen';
out_path = '../data/gen/matlab_output';
if ~exist(gen_path, 'dir')
    mkdir(gen_path);
end
if ~exist(out_path, 'dir')
    mkdir(out_path);
end
date = "20250329";
annotator = "thuy";
segment = "23";
batch = sprintf('%s_short_cams_%s_%s',segment,annotator,date);
annotation_3d_root = sprintf('%s/anno_3d_output',out_path);
point_cloud_folder = sprintf("%s/point_cloud", out_path);
% using_rectified = false;
car_model_param = load(sprintf('%s/car_model_param_202405.mat', 'raster'));
% use_uniform_frame = true;
use_lowess = false;
debug = false;
angle_algo = 'avg'; %% avg or pizlo
use_enforce = 'bounds'; %% bounds or no bounds
fileID_opt = fopen(sprintf('%s/hw7_results_%s_%s_%s.txt',out_path,annotator,angle_algo,use_enforce),'w');
fileID_failure_case = fopen(sprintf('%s/hw7_fails_%s_%s_%s.txt',out_path,annotator,angle_algo,use_enforce),'w');

%% All params
bias_check = [];
unit_test_invalid_cars = [];
not_within_roi_cars = [];
failed_valid_ex_sym_cars = [];
behind_cam_cars = [];
bad_annotation_cars = [];
bad_annotation_cars_syms = [];
raw_car_counts = 0;
valid_results_counts = 0;
mirror_one_side = 0.125;
estimated_dist_to_move = 0.1286;
category_select = 4;

strings = ["sc1", "sc2","sc3","sc4"];
for i = 1:length(strings)
    camera = strings(i);
    if debug
        camera = "sc1";
    end

    %% Start processing loop
    folder = sprintf("%s/formatted_anno/%s/%s/annotation",gen_path,batch,camera);
    filePattern = fullfile(folder, '*.json');
    all_files = dir(filePattern);
    %fprintf('folder: %s\n', folder)
    %fprintf('%d files\n', length(all_files))
    for k = 1 : length(all_files)
        baseFileName = all_files(k).name;
        parts = split(baseFileName,".");
        target = string(parts(1));
        if debug
            target = "0009";
            target_object_id = 5;
        end
    %     if str2num(target) <= 7095
    %         continue
    %     end
    
        %Load the ROI mask and terrain raster for each segment and camera
        if strcmp(camera,"sc2") && strcmp(segment,"23")
           load(fullfile('raster', '23_sc2_final_raster.mat'));
           load(fullfile('raster', '23_sc2_roi_maskings.mat'));
        elseif strcmp(camera,"sc1") && strcmp(segment,"23")
           load(fullfile('raster', '23_sc1_final_raster.mat'));
           load(fullfile('raster', '23_sc1_roi_maskings.mat'));
        elseif strcmp(camera,"sc3") && strcmp(segment,"23")
           load(fullfile('raster', '23_sc3_final_raster.mat'));
           load(fullfile('raster', '23_sc3_roi_maskings.mat'));
        elseif strcmp(camera,"sc4") && strcmp(segment,"23")
           load(fullfile('raster', '23_sc4_final_raster.mat'));
           load(fullfile('raster', '23_sc4_roi_maskings.mat'));
        elseif strcmp(camera,"lc1") && strcmp(segment,"23")
           load(fullfile('raster', '23_lc1_final_raster.mat'));
           load(fullfile('raster', '23_lc1_roi_maskings.mat'));
        elseif strcmp(camera,"lc2") && strcmp(segment,"23")
           load(fullfile('raster', '23_lc2_final_raster.mat'));
           load(fullfile('raster', '23_lc2_roi_maskings.mat'));
        end
    
        %% Read the raw_image file
        file = sprintf("%s/image/Seg%s/%s/%s.png",data_path,segment, camera, target);
        fid = fopen(file);
        raw_image = imread(file);
        if false
            figure;
            imshow(raw_image)
        end
        image = raw_image;
        
        %% ======= GET THE EXTRINSICS, INTRINSICS ==========
        file = sprintf("%s/calib/intrinsic_calibrations.json",data_path);
        fid = fopen(file);
        raw = fread(fid,inf);
        str = char(raw'); 
        fclose(fid); 
        intrinsics_data = jsondecode(str);
        k_matrix = intrinsics_data.(sprintf("%s",camera)).intrinsic_matrix(1:3,1:3);
        assert(k_matrix(1,2)==0);
        
        file = sprintf("%s/image/Seg%s/extrinsic_calibrations_%s.json",data_path,segment,segment);
        fid = fopen(file);
        raw = fread(fid,inf);
        str = char(raw'); 
        fclose(fid);
        extrinsic_allcams = jsondecode(str);
        extrinsics_data = extrinsic_allcams.(sprintf("T_%s_localgta",camera));
        T_velo_utm = extrinsic_allcams.("T_localgta_gta");
        
        %% ======= SET UP WORLD AT CAM ==========
        rot = extrinsics_data(1:3,1:3);
        t = extrinsics_data(1:3,4);
        assert(abs(det(rot)-1)<1e-5); %Check valid rotation
        M_cam_velo = [rot,t;0 0 0 1];
        R_velo_world = [[0 -1 0]' [0 0 -1]' [1 0 0]']; %Set up rotation of World with y-axis point down 
        assert(abs(det(R_velo_world)-1)<1e-5)
        R_cam_velo = M_cam_velo(1:3,1:3);
        M_cam_world = [R_cam_velo*R_velo_world,[0 0 0]'; 0 0 0 1]; %Transformation from world to cam
        M_world_velo = inv(M_cam_world)*M_cam_velo; %Transformation from velo to world
        assert(norm(M_cam_world*M_world_velo-M_cam_velo)<1e-6);
        R = M_cam_world(1:3,1:3);
        assert(abs(det(R)-1)<1e-5); %Change from 1e-5
    
        %% ======= GET LABELS IN ==========
        file = sprintf("%s/label/Seg%s/%s/%s.json",data_path,segment,camera,target);
        raw_labels = read_cars_hw7(file);
    
        %% ======= GET THE 3D BOUNDING BOXES OF ONE CAR ==========
        edges = [1 2;2 3;3 4;4 1;5 6;6 7;7 8;8 5;1 5;2 6;3 7;4 8;];
        edges_2D = [1 3;1 2;2 4;3 4;];
        
        if false
            figure;
            hold on
            imshow(image)
            %Yaxis vanishing
            hold on
            for l=1:size(raw_labels.valid_ids,2)
                obj_index = raw_labels.valid_ids(l)
                [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_hw7(raw_labels, obj_index);
        
                corners_bbox_world = M_world_velo*corners_velo_aug;
                corners_bbox_world = R*corners_bbox_world(1:3,:);
                corners_bbox_2D = k_matrix*corners_bbox_world;
                corners_bbox_2D = corners_bbox_2D./corners_bbox_2D(3,:);
        
        
                scatter(corners_bbox_2D(1,1),corners_bbox_2D(2,1),50,'red','filled');
                scatter(corners_bbox_2D(1,2),corners_bbox_2D(2,2),50,'green','filled');
                scatter(corners_bbox_2D(1,3),corners_bbox_2D(2,3),50,'blue','filled');
                for m = 1:size(edges,1)
                    x_coor = [corners_bbox_2D(1, edges(m,1)) corners_bbox_2D(1, edges(m,2))];
                    y_coor = [corners_bbox_2D(2, edges(m,1)) corners_bbox_2D(2, edges(m,2))];
                    line(x_coor, y_coor,'Color','red','LineWidth',1)
                end
                for m = 1:size(edges_2D,1)
                        x_coor = [bbox_2D(1, edges_2D(m,1)) bbox_2D(1, edges_2D(m,2))];
                        y_coor = [bbox_2D(2, edges_2D(m,1)) bbox_2D(2, edges_2D(m,2))];
                        line(x_coor, y_coor,'Color','white','LineWidth',1);
                end
                text(bbox_2D(1)+15,bbox_2D(2)-15,int2str(obj_index), 'Color', 'yellow','FontSize',8);
            end
        end
    
        %% =======  Get horizon line ======= 
        vp_points = get_vppoints(k_matrix,R);
        vp_points_2D = vp_points./vp_points(3,:);
        horizon_line = cross(vp_points(:,1),vp_points(:,3));
        
        if false
            figure;
            hold on
            imshow(image)
            hold on
            scatter(vp_points_2D(1,1),vp_points_2D(2,1),100,'red','filled');
            scatter(vp_points_2D(1,2),vp_points_2D(2,2),100,'green','filled');
            scatter(vp_points_2D(1,3),vp_points_2D(2,3),50,'blue','filled');
            line([vp_points_2D(1,1), vp_points_2D(1,3)],[vp_points_2D(2,1),...
                vp_points_2D(2,3)],'Color','red','LineWidth',2)
            hold on
            axis equal
            axis on
        end
    
        %% Get the annotation
        file = sprintf("%s/formatted_anno/%s/%s/annotation/%s.json",gen_path,batch,camera,target);
        fid = fopen(file);
        try
            raw = fread(fid,inf);
        catch ME
            fprintf("An error occurred: %s\n", ME.message);
            disp(file)
            return
        end
        str = char(raw'); 
        fclose(fid); 
        anno = jsondecode(str);
    
        %Load annotations in but skip those without a tire annotated
        annotations = struct;
        all_cars_ids = [];
        for l = 1:size(anno, 1)
            if isfield(anno(l),'skip_reason') || ~isfield(anno(l),'annotations')
                continue
            end   
            fieldname = sprintf("obj_%s",int2str(anno(l).obj_id)); %Difference old farmat and new format
            annotations.(fieldname) = anno(l).annotations;
            %Skip the car that 
            all_cars_ids = [all_cars_ids anno(l).obj_id];
        end
        raw_car_counts = raw_car_counts + size(all_cars_ids,2);
    
        %% Start processing points
        all_annos = struct;
    
        % Process whether having mirror or not
        for l = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(l);
            fieldname = sprintf("obj_%s",int2str(annotated_car_id));
            one_anno = annotations.(fieldname);
            all_annos.(fieldname).('has_mirror') = one_anno.has_mirror;
        end
    
        % Process all the car tires
        for l = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(l);
            fieldname = sprintf("obj_%s",int2str(annotated_car_id));
            one_anno = annotations.(fieldname);
            tire_fields = ["DR","PR","DF","PF"];
            tires = struct;
            for m=1:size(tire_fields,2)
                a_tire = one_anno.tire_points.(tire_fields(m));
                if a_tire(1) > 0 || a_tire(2) > 0
                    tires.(tire_fields(m)) = [a_tire; 1];
                end
            end
            all_annos.(fieldname).('tires') = tires;
        end
        
%         for l = 1:size(all_cars_ids,2)
%             annotated_car_id = all_cars_ids(l);
%             fieldname = sprintf("obj_%s",int2str(annotated_car_id));
%             one_anno = annotations.(fieldname);
%             ex_pairs = reshape(one_anno.extremal_pairs,[2,size(one_anno.extremal_pairs,2)/2]);
%             ex_pairs = [ex_pairs; ones(1,size(ex_pairs,2))];
%             all_annos.(fieldname).('ex') = ex_pairs;
%         end

        %Process of extremal points
        for l = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(l);
            fieldname = sprintf("obj_%s",int2str(annotated_car_id));
            one_anno = annotations.(fieldname);
            all_pairs = zeros(3,size(one_anno.extremal_pairs,1)*2);
            for m = 1:size(one_anno.extremal_pairs,1)
                a_pair = reshape(one_anno.extremal_pairs(m,:),2,[]);
                a_pair = [a_pair; ones(1,size(a_pair,2))];
                all_pairs(:,2*m-1:2*m) =  a_pair;
            end
            all_annos.(fieldname).('ex') = all_pairs;
        end
        
        %Process of non extremal points
        for l = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(l);
            fieldname = sprintf("obj_%s",int2str(annotated_car_id));
            one_anno = annotations.(fieldname);
            all_pairs = zeros(3,size(one_anno.non_extremal_pairs,1)*2);
            for m = 1:size(one_anno.non_extremal_pairs,1)
                a_pair = reshape(one_anno.non_extremal_pairs(m,:),2,[]);
                a_pair = [a_pair; ones(1,size(a_pair,2))];
                all_pairs(:,2*m-1:2*m) =  a_pair;
            end
            all_annos.(fieldname).('nonex') = all_pairs;
        end
        
        % Process center points #No center points will be processed
        if false
            for l = 1:size(all_cars_ids,2)
                annotated_car_id = all_cars_ids(l);
                fieldname = sprintf("obj_%s",int2str(annotated_car_id));
                one_anno = annotations.(fieldname);
                if size(one_anno.center_points, 1) > 0
                    pairs = [one_anno.center_points ones(size(one_anno.center_points,1),1)]';
                    all_annos.(fieldname).('center') = pairs;
                end
            end
        end
        
       

    
    
        %% Visualize all annotations THESIS VIS
        if false
            figure;
            hold on
            imshow(image)
            hold on
            for l = 1:size(all_cars_ids,2)
                annotated_car_id = all_cars_ids(l);
                fieldname = sprintf("obj_%s",int2str(annotated_car_id))
                tires = all_annos.(fieldname).('tires');
                fields = fieldnames(tires)
                for m = 1:size(fields,1)
                    that_tire = string(fields(m));
                    scatter(tires.(that_tire)(1),tires.(that_tire)(2),60,'magenta','filled');
                    text(tires.(that_tire)(1)+5,tires.(that_tire)(2)-5,fields(m), 'Color', 'yellow','FontSize',20)
                end
                text(tires.(that_tire)(1)-25,tires.(that_tire)(2)+15,int2str(annotated_car_id), ...
                    'Color', 'yellow','FontSize',40)
            end
        
            for l = 1:size(all_cars_ids,2)
                annotated_car_id = all_cars_ids(l);
                fieldname = sprintf("obj_%s",int2str(annotated_car_id))
                points = all_annos.(fieldname).('ex');
                scatter(points(1,:),points(2,:),30,'magenta','filled');
                points = all_annos.(fieldname).('nonex');
                scatter(points(1,:),points(2,:),10,"magenta",'filled'); 
    %             scatter(points(1,1:2),points(2,1:2),10,"red",'filled');
    %             scatter(points(1,3:4),points(2,3:4),10,"red",'filled');
    %             scatter(points(1,7:8),points(2,7:8),10,"red",'filled');
    %             scatter(points(1,9:10),points(2,9:10),10,"red",'filled');
                if isfield(all_annos.(fieldname),'center')
                    points = all_annos.(fieldname).('center');
                    scatter(points(1,:),points(2,:),10,"cyan",'filled');
                end
            end
    %         for k=1:size(all_cars_ids,2)
    %             obj_index = all_cars_ids(k)
    %             [yaw_angle, corners_velo_aug, bbox_2D] = get_3d_bbox_road(raw_labels, obj_index);
    %     %         text(bbox_2D(1)+15,bbox_2D(2)-15,int2str(obj_index), 'Color', 'red','FontSize',10);
    %         end
    
            axis on
            axis equal
        end
    
        %% Manual mapping for now get 3D bbox
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = get_bbox_world_hw7(all_annos, annotated_car_id,raw_labels,...
               M_world_velo);
        end
    
        %% Read LIDAR: read LIDAR and visualize on image     
        file = sprintf("%s/image/Seg%s/lidar_local_gta/%s.csv",data_path,segment, num2str(str2num(target)-1)); %Difference due to indexing
        ptCloud = readmatrix(file);
    
        %% Check in LiDAR coordinate - All good
        if false
            figure
            hold on
            pcshow(ptCloud(:,1:3), 'blue','MarkerSize',5);
            pcshow([0,0,0], 'magenta','MarkerSize',100);
        end
    
        %% Transformation to World Coordinate system
        ptCloud_world = M_world_velo*[ptCloud';ones(1,size(ptCloud,1))];
        ptCloud_world = ptCloud_world(1:3,:);
        %Only infront of camera, this is
        ptCloud_world_cam = R*ptCloud_world;
        ptCloud_world = ptCloud_world(:, ptCloud_world_cam(3,:) > 0); %Crucial to remove behind cam

        %All points even not within the image


        ptCloud_above = ptCloud_world(:,ptCloud_world(2,:)<9.2);
        ptCloud_above_2D = k_matrix*R*ptCloud_above;
        ptCloud_above_2D = ptCloud_above_2D./ptCloud_above_2D(3,:);

         if false
            figure
            hold on
            pcshow(ptCloud_world(1:3,:)', 'blue','MarkerSize',5);
            pcshow([0,0,0], 'magenta','MarkerSize',100);
        end
    
    
        %% Filter out points to be within the mask and above the terrain model
        % if false %%%%%%%%%%%%%%%%%%%%%%%%%
            %Within ROI only
            if false
                ptCloud_world_2D = k_matrix*R*ptCloud_world;
                ptCloud_world_2D = ptCloud_world_2D./ptCloud_world_2D(3,:);
                in = inpolygon(ptCloud_world_2D(1,:),ptCloud_world_2D(2,:),mask(:,1),mask(:,2));
                ptCloud_world = ptCloud_world(:,in); %Filtering out on 2D
            end

            %For checking even out of ROI to check for infrastructure
            if false
               ptCloud_world_2D = k_matrix*R*ptCloud_world;
               ptCloud_world_2D = ptCloud_world_2D./ptCloud_world_2D(3,:);
               buffer = 0;
               ptCloud_world = ptCloud_world(:,ptCloud_world_2D(1,:)>=(-buffer) & ptCloud_world_2D(1,:)<=(size(image,2)+buffer) & ...
               ptCloud_world_2D(2,:)>=(-buffer) & ptCloud_world_2D(2,:)<=(size(image,1)+buffer));
            end

            if false
                figure;
                hold on
                imshow(image)
                hold on
                % Draw mask
                drawpolygon('Position',mask,'Color','r','FaceAlpha',0,'MarkerSize',0.3);
    
                % Draw all points
                colors = ptCloud_above(2,:);
                scatter(ptCloud_above_2D(1,:),ptCloud_above_2D(2,:),3,colors,'filled')
    
    
                % Selected points only within image only
                ptCloud_2D = k_matrix*R*ptCloud_world;
                ptCloud_2D = ptCloud_2D./ptCloud_2D(3,:);
                within_indice = ptCloud_2D(1,:)>=0 & ptCloud_2D(1,:)<=size(image,2) & ...
                ptCloud_2D(2,:)>=0 & ptCloud_2D(2,:)<=size(image,1);
                ptCloud_2D = ptCloud_2D(:,within_indice);
                colors = round(ptCloud_world(2,within_indice),2);
                scatter(ptCloud_2D(1,:),ptCloud_2D(2,:),3,colors,'filled')

                colormap(gca,"parula")
                 c = colorbar;
                c.Label.String = 'Vertical depth (m)';
                c.FontSize = 15;
                c.Label.FontSize = 15;
                multiple = 0.2;
                c.Position = [0.9 0.093 0.025 0.87];
                axis equal;
            end

        % end
    
        %% Dictionary for tire avail
        id_to_tire =["DF","PF","PR","DR"];
    
        %% Print out algorithmic scenarior first and go from there
        %Configuration for each cars
        cars_in_category = [];
        for l = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(l);
            fieldname = sprintf("obj_%s",int2str(annotated_car_id));
            one_anno = all_annos.(fieldname);
            tires = all_annos.(fieldname).('tires');
            fields = fieldnames(tires);
            string_fields = string(fields);
            all_annos.(fieldname).tire_ids = [];
            for m = 1:size(string_fields,1)
                all_annos.(fieldname).tire_ids = ...
                    [all_annos.(fieldname).tire_ids find(id_to_tire == string_fields(m))];
            end
            all_annos.(fieldname).tire_ids = sort(all_annos.(fieldname).tire_ids);
            flag = char(fields(1));
            flag = flag(1);
            all_annos.(fieldname).('tire_both_sides') = 0;
            for m = 1:size(fields,1)
                that_tire = char(fields(m));
                if that_tire(1) ~= flag
                    all_annos.(fieldname).('tire_both_sides') = 1; %Check tire using the word
                    break
                end
            end
            all_annos.(fieldname).('num_sym_pairs') = size(one_anno.('nonex'),2)/2 ...
                + size(one_anno.('ex'),2)/2;
        end
  
        
        %% Find visible
        for l = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(l);
            all_annos = find_visible(annotated_car_id, all_annos);
        end
    
        %% Determine the height of tires
        cars_in_roi = [];
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           fieldname = sprintf("obj_%s",int2str(annotated_car_id));
           origin_2D = all_annos.(fieldname).tires.(id_to_tire(all_annos.(fieldname).car_origin));
           mask = v1; %Use the polygon directly as mask
           if use_lowess
              [in_roi, mounting_height] = get_height_kernel_mean_hw7(mask, f,k_matrix, R,id_to_tire, annotated_car_id, all_annos); 
           else
              [in_roi, mounting_height] = get_height_raster(mask,id_to_tire, annotated_car_id, all_annos, final_raster); 
           end       
           
           if in_roi
                cars_in_roi = [cars_in_roi annotated_car_id];
                assert(mounting_height>0);
                all_annos.(fieldname).mounting_height = mounting_height;
           else
                not_within_roi_cars = [not_within_roi_cars; sprintf("%s_%s_%s_not_within_roi",camera,target,int2str(annotated_car_id))];
           end
        end
        all_cars_ids = cars_in_roi; %Only evaluate cars in ROI
        if debug
            assert(size(all_cars_ids,2) > 0, 'At least have some cars to debug')
        end
    
        %% ======= DETERMINING VP CAR==========
        for l = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(l);
            fieldname = sprintf("obj_%s",int2str(annotated_car_id));
            one_anno = all_annos.(fieldname);
            anno_sympoints_homo = [one_anno.ex one_anno.nonex];

            %METHOD ANGLE ABLATION
            if strcmp(angle_algo, 'avg')
                vp_car = get_vpcar_avg_intersects_v2(anno_sympoints_homo, horizon_line);  
            elseif strcmp(angle_algo, 'pizlo')
                vp_car = get_vpcar_pizlo(anno_sympoints_homo, horizon_line,k_matrix,R,vp_points_2D, image);
            end
            assert(abs(dot(vp_car,horizon_line)) < 1e-6, 'VP car gotta be on horizon line')
            all_annos.(fieldname).vp_car = vp_car;
        end
    
        %% Visualization of found vp_car for one car THESIS VIS
        if false
            figure;
            hold on
            imshow(image)
            hold on
            for l = 1:size(all_cars_ids,2)
                annotated_car_id = all_cars_ids(l);
    %             if annotated_car_id ~= 20
    %                continue
    %             end
                fieldname = sprintf("obj_%s",int2str(annotated_car_id));
                one_anno = all_annos.(fieldname);
                vp_car = all_annos.(fieldname).vp_car;
                scatter(vp_car(1,:),vp_car(2,:),300,'blue','filled');
    
                %Plot tires
                tires = all_annos.(fieldname).('tires');
                fields = fieldnames(tires)
                for m = 1:size(fields,1)
                    that_tire = string(fields(m));
                    scatter(tires.(that_tire)(1),tires.(that_tire)(2),60,'magenta','filled');
                    text(tires.(that_tire)(1)+5,tires.(that_tire)(2)-5,fields(m), 'Color', 'yellow','FontSize',20)
                end
                text(tires.(that_tire)(1)-25,tires.(that_tire)(2)+15,int2str(annotated_car_id), ...
                    'Color', 'yellow','FontSize',40)
    
                %Plotting non-extremal
                anno_sympoints_homo = [one_anno.nonex]
    %             for l = 1:size(anno_sympoints_homo,2)/2
    %                     %  line([anno_sympoints_homo(1,2*l), anno_sympoints_homo(1,2*l-1)], [anno_sympoints_homo(2,2*l), anno_sympoints_homo(2,2*l-1)],'Color','yellow','LineWidth',2,'LineStyle','--'); 
    %                     line([anno_sympoints_homo(1,2*l), vp_car(1,:)], [anno_sympoints_homo(2,2*l), vp_car(2,:)],'Color','yellow','LineWidth',2,'LineStyle','--'); 
    %             end
                for l = 1:size(anno_sympoints_homo,2)/2
                     scatter([anno_sympoints_homo(1,2*l)], [anno_sympoints_homo(2,2*l)],30,'green','filled');
                     scatter([anno_sympoints_homo(1,2*l-1)], [anno_sympoints_homo(2,2*l-1)],30,'red','filled');
                end
    
    
                %Plotting extremal
                anno_sympoints_homo = [one_anno.ex]
    %             for l = 1:size(anno_sympoints_homo,2)/2
    %                     % line([anno_sympoints_homo(1,2*l), anno_sympoints_homo(1,2*l-1)], [anno_sympoints_homo(2,2*l), anno_sympoints_homo(2,2*l-1)],'Color','yellow','LineWidth',2,'LineStyle','--'); 
    %                     line([anno_sympoints_homo(1,2*l), vp_car(1,:)], [anno_sympoints_homo(2,2*l), vp_car(2,:)],'Color','yellow','LineWidth',2,'LineStyle','--'); 
    %             end
                for l = 1:size(anno_sympoints_homo,2)/2
                        scatter([anno_sympoints_homo(1,2*l)], [anno_sympoints_homo(2,2*l)],80,'green','filled');
                        scatter([anno_sympoints_homo(1,2*l-1)], [anno_sympoints_homo(2,2*l-1)],80,'red','filled');
                end
            end
    
            %Vanishing line plotting
            horizon_line_2D_vec = vp_points_2D(:,3)-vp_points_2D(:,1);
            vp_point_vis_1 = vp_points_2D(:,3) - 0.003*horizon_line_2D_vec;
            vp_point_vis_2 = vp_points_2D(:,3) + 0.002*horizon_line_2D_vec;
    
    %         scatter(vp_points_2D(1,1),vp_points_2D(2,1),100,'red','filled');
    %         scatter(vp_points_2D(1,2),vp_points_2D(2,2),100,'green','filled');
    %         scatter(vp_points_2D(1,3),vp_points_2D(2,3),50,'blue','filled');
    
            line([vp_point_vis_1(1), vp_point_vis_2(1)],[vp_point_vis_1(2),...
                    vp_point_vis_2(2)],'Color','red','LineWidth',2);
    
    %         line([vp_points_2D(1,2), vp_points_2D(1,3)],[vp_points_2D(2,2),...
    %             vp_points_2D(2,3)],'Color','red','LineWidth',2)
            axis equal
    %         axis on
    %         xlabel('X')
    %         ylabel('Y')
        end
    
        %% ======= GETTING THE SYMMETRY NORMAL ==========
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = get_sym_normal(annotated_car_id, all_annos,k_matrix,R);
        end
    
        %% Localizing planes
        cars_with_valid_sym_points = [];
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           [bad_annotation_sym, all_annos] = localize_planes(k_matrix,R,annotated_car_id,all_annos,id_to_tire,image);
           if ~bad_annotation_sym
               cars_with_valid_sym_points = [cars_with_valid_sym_points annotated_car_id];
           else
               bad_annotation_cars_syms = [bad_annotation_cars_syms; sprintf("%s_%s_%s_bad_annotations_syms",camera,target,int2str(annotated_car_id))];
           end
        end
        all_cars_ids = cars_with_valid_sym_points;
    
       %%  Locating all symmetry points
        cars_in_front_cam = [];
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
%            assert(1==0, 'Gotta fix the behind car cam process to be different from coop')
           [points_behind_cam, all_annos] = sym_points_3D_pizlo(k_matrix,R,annotated_car_id,all_annos,image);
           if ~points_behind_cam
               cars_in_front_cam = [cars_in_front_cam annotated_car_id];
           else
               behind_cam_cars = [behind_cam_cars; sprintf("%s_%s_%s_behind_cam_points",camera,target,int2str(annotated_car_id))];
           end
        end
    
    
        %Not really, this is not the same as COOP due to multiple camera view
    %     all_cars_ids = cars_in_front_cam; %Only evaluate cars with all points in front only
    
%         if debug
%             assert(size(all_cars_ids,2) > 0, 'At least have some cars to debug')
%         end
        
        %% Visualize and calculate reprojection error
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = get_reproj_error(k_matrix,R,annotated_car_id,all_annos,id_to_tire);
        end
        
        %% Determine wheelbase
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = eval_wheelbase(annotated_car_id,all_annos,id_to_tire);
        end
    
        %% IOU Eval
        valid_ex_sym_cars = [];
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           valid_ex_sym = check_ex_sym(all_annos,annotated_car_id);
           if valid_ex_sym
               valid_ex_sym_cars = [valid_ex_sym_cars annotated_car_id];
               all_annos = get_pred_bbox(k_matrix,R,annotated_car_id,all_annos,id_to_tire,car_model_param,image,mirror_one_side);
           else
               failed_valid_ex_sym_cars = [failed_valid_ex_sym_cars; sprintf("%s_%s_%s_failed_ex_sym",camera,target,int2str(annotated_car_id))];
           end
        end
        all_cars_ids = valid_ex_sym_cars;
        
        %% All unit testing before OPT
        threshold = 1e-6;
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           [valid_result, reason] = all_unit_tests(annotated_car_id,all_annos,id_to_tire,threshold,horizon_line, false);
           if debug & ~valid_result
               assert(1<0,'Debug unit tests before opt')
           end
        end
    
        %% Do the LM non-linear optimization
        if true
            cars_with_valid_anno = [];
            for l = 1:size(all_cars_ids,2)
               annotated_car_id = all_cars_ids(l);
               [bad_annotation, all_annos] = do_opt(annotated_car_id,...
                   all_annos,id_to_tire,horizon_line,k_matrix,R,image,...
                   car_model_param,use_enforce,mirror_one_side,estimated_dist_to_move);
               % [bad_annotation, all_annos] = do_opt_hw7(annotated_car_id,all_annos,id_to_tire,horizon_line,k_matrix,R,image, car_model_param);
               if ~bad_annotation
                   cars_with_valid_anno = [cars_with_valid_anno annotated_car_id];
               else
                   bad_annotation_cars = [bad_annotation_cars; sprintf("%s_%s_%s_bad_annotations",camera,target,int2str(annotated_car_id))];
               end
            end
            all_cars_ids = cars_with_valid_anno;
        end

        %% Get the box containing point only
        for i = 1:size(all_cars_ids,2)
            annotated_car_id = all_cars_ids(i);
            all_annos = get_lidar_box_non_propagate(annotated_car_id,all_annos, k_matrix,R,image,id_to_tire,ptCloud_world);
        end

        %% Visualization in sideview 
        % if debug
        %     for i = 1:size(all_cars_ids,2)
        %         annotated_car_id = all_cars_ids(i);
        %         if annotated_car_id ~= target_object_id
        %            continue
        %         end
        %         visualize_side_view(annotated_car_id,all_annos, k_matrix,R,image,id_to_tire,ptCloud_world,1)
        %     end
        % end

    
        %% Rerun evaluation agains
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = get_reproj_error_with_3D_points(k_matrix,R,annotated_car_id,all_annos);
        end
    
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = eval_width(annotated_car_id,all_annos);
        end
    
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = eval_height(annotated_car_id,all_annos);
        end
    
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = eval_length(annotated_car_id,all_annos);
        end
        
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = eval_iou(annotated_car_id,all_annos);
        end
    
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = heading_from_normal(annotated_car_id, all_annos);
        end
    
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           all_annos = eval_angle_coop(annotated_car_id,all_annos);
        end

    
        %% ALl unit testing run again
        valid_cars = [];
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           [valid_result, reason] = all_unit_tests(annotated_car_id,all_annos,id_to_tire,threshold,horizon_line, true);
           if ~valid_result
                unit_test_invalid_cars = [unit_test_invalid_cars; sprintf("%s_%s_%s_%s_unit_testing",camera,target,int2str(annotated_car_id), reason)];
           else
                valid_cars = [valid_cars annotated_car_id];
           end
        end
        all_cars_ids = valid_cars;
    
    
        %% Save 3d points
        %write_3d_annotations_hw7_labeled(annotation_3d_root, camera, target, all_annos, all_cars_ids, id_to_tire);
        
        %fieldname = sprintf("obj_%s",int2str(target_object_id));
        %points = all_annos.(fieldname).all_3D_points;  % your 3xN matrix
        %points = points';  % transpose to make it N×3 (rows = points)
        %write_results_hw7(fileID_opt,all_annos,annotated_car_id,target,id_to_tire, camera);
        %writematrix(points, sprintf('%s/%s_%s_%s.csv',out_path,camera,target,annotated_car_id));
        write_3d_annotations_hw7(point_cloud_folder, camera, target, all_annos, all_cars_ids);


        %% After nonlinear opt. TODO: bbox_2D_height
        for l = 1:size(all_cars_ids,2)
           annotated_car_id = all_cars_ids(l);
           write_results_hw7(fileID_opt,all_annos,annotated_car_id,target,id_to_tire, camera);
           valid_results_counts = valid_results_counts + 1;
           fieldname = sprintf("obj_%s",int2str(annotated_car_id));

           bias_check = [bias_check [all_annos.(fieldname).tires_3D.(id_to_tire(all_annos.(fieldname).car_origin))' all_annos.(fieldname).dist_base_bbox_diff]];
        end
    
    
        %% Visualize box in 3D 
        if false
            figure;
            hold on
            for l = 1:size(all_cars_ids,2)
               annotated_car_id = all_cars_ids(l);
               fieldname = sprintf("obj_%s",int2str(annotated_car_id));
               pcshow([all_annos.(fieldname).pred_bbox(1,:)',all_annos.(fieldname).pred_bbox(2,:)',all_annos.(fieldname).pred_bbox(3,:)'],'green','MarkerSize',500);
               pcshow([all_annos.(fieldname).corners_bbox_world(1,:)',all_annos.(fieldname).corners_bbox_world(2,:)',all_annos.(fieldname).corners_bbox_world(3,:)'],'red','MarkerSize',500);
               pcshow([all_annos.(fieldname).all_3D_points(1,:)',all_annos.(fieldname).all_3D_points(2,:)',all_annos.(fieldname).all_3D_points(3,:)'],'green','MarkerSize',300);
            end
            pcshow(ptCloud_world','blue','MarkerSize',10);
            % pcshow(ptCloud_ground, 'blue','MarkerSize',20);
            pcshow(ptCloud_above', 'yellow','MarkerSize',20);  %%%%%%%%
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
        end
    
        %% Visualize after on 2D images all visualizaiton on 2D images happen here THESIS VIS
        if false
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
            edges_2D = [
                     1 3;
                     1 2;
                     2 4;
                     3 4;
                ];
            figure;
            imshow(image)
            hold on
            for l = 1:size(all_cars_ids,2)
                annotated_car_id = all_cars_ids(l);
                if annotated_car_id ~=target_object_id
                   continue
                end
                fieldname = sprintf("obj_%s",int2str(annotated_car_id));
%                 scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),...
%                     all_annos.(fieldname).all_annotated_points(2,:),80,"red",'filled')
%                 scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),...
%                     all_annos.(fieldname).all_3D_points_to_2D(2,:),50,"green",'filled')
                 scatter(all_annos.(fieldname).all_annotated_points(1,:),...
                        all_annos.(fieldname).all_annotated_points(2,:),70,"green",'o','filled')
                scatter(all_annos.(fieldname).all_3D_points_to_2D(1,:),...
                        all_annos.(fieldname).all_3D_points_to_2D(2,:),50,"red",'o','filled')

                %Visualize tires
                start_pos = 0;
                for i = 1:size(all_annos.(fieldname).tire_ids,2)
%                     atire = all_annos.(fieldname).all_annotated_points(:,start_pos+i);
%                     scatter(atire(1), atire(2), 70, 'red', 'filled', 'MarkerEdgeColor', 'black', 'LineWidth', 2)
                    % atire = all_annos.(fieldname).all_3D_points_to_2D(:,start_pos+i);
                    % scatter(atire(1), atire(2), 50, 'green', 'filled', 'MarkerEdgeColor', 'black', 'LineWidth', 2)
                end


                if false
                    bbox = all_annos.(fieldname).corners_bbox_world;
                    sym_plane_vis = [mean([bbox(:,1), bbox(:,2)],2),...
                        mean([bbox(:,5), bbox(:,6)],2),...
                        mean([bbox(:,3), bbox(:,4)],2),...
                        mean([bbox(:,7), bbox(:,8)],2)];
                    sym_plane_vis_2D = k_matrix*R*sym_plane_vis;
                    sym_plane_vis_2D = sym_plane_vis_2D./sym_plane_vis_2D(3,:);
                    for m = 1:size(edges_2D,1)
                        x_coor = [sym_plane_vis_2D(1, edges_2D(m,1)) sym_plane_vis_2D(1, edges_2D(m,2))];
                        y_coor = [sym_plane_vis_2D(2, edges_2D(m,1)) sym_plane_vis_2D(2, edges_2D(m,2))];
                        line(x_coor, y_coor,'Color','red','LineWidth',1);
                    end
                end
    
                %If outputing sym plan visualization
                if false
                    bbox = all_annos.(fieldname).pred_bbox;
                    sym_plane_vis = [mean([bbox(:,1), bbox(:,2)],2),...
                        mean([bbox(:,5), bbox(:,6)],2),...
                        mean([bbox(:,3), bbox(:,4)],2),...
                        mean([bbox(:,7), bbox(:,8)],2)];
                    sym_plane_vis_2D = k_matrix*R*sym_plane_vis;
                    sym_plane_vis_2D = sym_plane_vis_2D./sym_plane_vis_2D(3,:);
    
                    for m = 1:size(edges_2D,1)
                        x_coor = [sym_plane_vis_2D(1, edges_2D(m,1)) sym_plane_vis_2D(1, edges_2D(m,2))];
                        y_coor = [sym_plane_vis_2D(2, edges_2D(m,1)) sym_plane_vis_2D(2, edges_2D(m,2))];
                        line(x_coor, y_coor,'Color','green','LineWidth',1);
                    end
                end
                %If output bounding box
                if false
                    pred_bbox_2D = k_matrix*R*all_annos.(fieldname).pred_bbox;
                    pred_bbox_2D = pred_bbox_2D./pred_bbox_2D(3,:);
%                     scatter(pred_bbox_2D(1,1),pred_bbox_2D(2,1),20,'red','filled');
%                     scatter(pred_bbox_2D(1,2),pred_bbox_2D(2,2),20,'green','filled');
%                     scatter(pred_bbox_2D(1,3),pred_bbox_2D(2,3),20,'blue','filled');
                    for m = 1:size(edges,1)
                        x_coor = [pred_bbox_2D(1, edges(m,1)) pred_bbox_2D(1, edges(m,2))];
                        y_coor = [pred_bbox_2D(2, edges(m,1)) pred_bbox_2D(2, edges(m,2))];
                        line(x_coor, y_coor,'Color','green','LineWidth',2)
                    end
                    gt_bbox_2D = k_matrix*R*all_annos.(fieldname).corners_bbox_world;
                    gt_bbox_2D = gt_bbox_2D./gt_bbox_2D(3,:);
%                     scatter(gt_bbox_2D(1,1),gt_bbox_2D(2,1),20,'red','filled');
%                     scatter(gt_bbox_2D(1,2),gt_bbox_2D(2,2),20,'green','filled');
%                     scatter(gt_bbox_2D(1,3),gt_bbox_2D(2,3),20,'blue','filled');
%                     text(gt_bbox_2D(1,1)+15,gt_bbox_2D(2,1)-15,fieldname, 'Color', 'yellow','FontSize',8);

                    for m = 1:size(edges,1)
                        x_coor = [gt_bbox_2D(1, edges(m,1)) gt_bbox_2D(1, edges(m,2))];
                        y_coor = [gt_bbox_2D(2, edges(m,1)) gt_bbox_2D(2, edges(m,2))];
                        line(x_coor, y_coor,'Color','red','LineWidth',2)
                    end
                    end
            end
            % qw{1} = plot(nan, 'o','Color','red','MarkerSize',8,'MarkerFaceColor','red');
            % qw{2} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green');
            % qw{3} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','green','MarkerEdgeColor', 'black', 'LineWidth', 3);
            % qw{4} = plot(nan, '-','Color','red','LineWidth',4);
            % qw{5} = plot(nan, '-','Color','green','LineWidth',4);
            % qw{6} = plot(nan, '.','Color','blue','MarkerSize',15);
            % legend([qw{:}], {'Annotations of symmetry points and tire-ground contact points',...
            %             'Predicted symmetry points',...
            %             'Predicted tire-ground contact points',...
            %             'LiDAR-derived bounding box',...
            %             'Symmetry-derived bounding box',...
            %             'LiDAR points'}, 'location', 'northoutside', 'FontSize', 18)
            qw{1} = plot(nan, 'o','Color','green','MarkerSize',8,'MarkerFaceColor','red');
            qw{2} = plot(nan, 'o','Color','red','MarkerSize',8,'MarkerFaceColor','green');
            legend([qw{1}, qw{2}], {'2D annotated points','3D annotated points'}, 'location', 'northoutside', 'FontSize', 18)
                if false %If add Lidar points
                    colors = ptCloud_above(2,:);
                    scatter(ptCloud_above_2D(1,:),ptCloud_above_2D(2,:),3,colors,'filled')
                    colormap(gca,"parula")
                    colorbar
                end
        end
        if debug
          break
        end
    end
    if debug
       break
    end
end
     
%% Displaying the invalid cars
raw_car_counts
valid_results_counts
not_within_roi_cars
failed_valid_ex_sym_cars
behind_cam_cars
bad_annotation_cars
bad_annotation_cars_syms
unit_test_invalid_cars


%% Need to write all these to files
if size(not_within_roi_cars,1) > 0
    fprintf(fileID_failure_case, '%s\n', not_within_roi_cars);
end
if size(failed_valid_ex_sym_cars,1) > 0
    fprintf(fileID_failure_case, '%s\n', failed_valid_ex_sym_cars);
end
if size(behind_cam_cars,1) > 0
    fprintf(fileID_failure_case, '%s\n', behind_cam_cars);
end
if size(bad_annotation_cars,1) > 0
    fprintf(fileID_failure_case, '%s\n', bad_annotation_cars);
end
if size(bad_annotation_cars_syms,1) > 0
    fprintf(fileID_failure_case, '%s\n', bad_annotation_cars_syms);
end
if size(unit_test_invalid_cars,1) > 0
    fprintf(fileID_failure_case, '%s\n', unit_test_invalid_cars);
end