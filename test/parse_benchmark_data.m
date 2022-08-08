% Parse benchmark results from C++ implementation
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2022

close all; clear; clc;

disp('*******************************************************************')
disp('****** Parser for benchmark results from C++ implementation *******')
disp('*******************************************************************')

%% Retrieve data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Options for static and continuous cases, please change accordingly
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt = 'static';
date = '2021-11-09'; % Results in paper
% date = '2022-08-03'; % Recent implementation

% opt = 'continuous';
% date = '2022-06-28'; % Results in paper
% date = '2022-08-03'; % Recent implementation

%% Parse and store benchmark data for each geometric pair
% parameters
input_path = [opt, '/', date , '/'];
accuracy_threshold = 1e-5;

store_folder_name = ['../data/', opt, '/data_parsed/'];
mkdir(store_folder_name)

shape_type = {'E', 'E'; 'E', 'PE'; 'PE', 'PE';
    'SQ', 'E'; 'SQ', 'PE'; 'SQ', 'SQ'};

for id_pair = 1:size(shape_type, 1)
    folder_name = ['../data/', input_path,...
        shape_type{id_pair, 1}, '-', shape_type{id_pair, 2}, '/'];
    
    if strcmp(opt, 'static')
        file_prefix = [folder_name, 'bench_result_',...
            shape_type{id_pair, 1}, '_', shape_type{id_pair, 2}];
        
        name = {'FCL', 'Implicit', 'CN-LS', 'CN-FP',...
            'CFC-Dist-LS', 'CFC-CN-LS', 'CFC-CN-FP'};
        label = [6,7,5,4,2,3,1];
        
    elseif strcmp(opt, 'continuous')
        file_prefix = [folder_name, 'bench_result_continuous_',...
            shape_type{id_pair, 1}, '_', shape_type{id_pair, 2}];
        
        name = {'CFC-Dist-LS-Trans', 'CFC-Dist-LS-Linear'};
        label = [2,1];
    end
    
    file_details = dir([file_prefix, '_*.csv']);
    
    % Read and extract data
    num_files = size(file_details,1);
    bench_data = cell(num_files,1);
    
    for i = 1:num_files
        bench_data{i}.name = file_details(i).name;
        bench_data{i}.raw = csvread([folder_name, '/', bench_data{i}.name], 1, 1);
        bench_data{i}.num_trials = size(bench_data{i}.raw, 1);
        
        % Collision detection results
        bench_data{i}.is_collision = bench_data{i}.raw(:,1);
        bench_data{i}.distance = bench_data{i}.raw(:,2);
        
        if strcmp(opt, 'continuous')
            bench_data{i}.optimal_time = bench_data{i}.raw(:,3);
        end
        
        % Optimization solutions
        bench_data{i}.optimal_normal = bench_data{i}.raw(:,end-16:end-14);
        bench_data{i}.witness_point_s1 = bench_data{i}.raw(:,end-13:end-11);
        bench_data{i}.witness_point_s2 = bench_data{i}.raw(:,end-10:end-8);
        bench_data{i}.minksum_point = bench_data{i}.raw(:,end-7:end-5);
        
        % Necessary condition for accuracy
        bench_data{i}.necessary_condition = bench_data{i}.raw(:,end-4);
        
        % Computational time and iteration
        bench_data{i}.run_time.initial = bench_data{i}.raw(:,end-3);
        bench_data{i}.run_time.query = bench_data{i}.raw(:,end-2);
        bench_data{i}.run_time.total = bench_data{i}.raw(:,end-1);
        
        bench_data{i}.num_iteration = bench_data{i}.raw(:,end);
    end
    
    %% Result average
    for i = 1:num_files
        idx = label(i);
        
        bench_data_avg.name{i} = name{i};
        
        bench_data_avg.run_time.initial(1,i) = mean(bench_data{idx}.run_time.initial);
        bench_data_avg.run_time.query(1,i) = mean(bench_data{idx}.run_time.query);
        bench_data_avg.run_time.total(1,i) = mean(bench_data{idx}.run_time.total);
        bench_data_avg.run_time.initial(2,i) = std(bench_data{idx}.run_time.initial);
        bench_data_avg.run_time.query(2,i) = std(bench_data{idx}.run_time.query);
        bench_data_avg.run_time.total(2,i) = std(bench_data{idx}.run_time.total);
        
        idx_condition = bench_data{idx}.necessary_condition < accuracy_threshold;
        bench_data_avg.necessary_condition(1,i) = mean(...
            bench_data{idx}.necessary_condition(idx_condition));
        bench_data_avg.necessary_condition(2,i) = std(...
            bench_data{idx}.necessary_condition(idx_condition));
        bench_data_avg.accuracy(i) = sum(idx_condition) / bench_data{idx}.num_trials;
        
        bench_data_avg.num_iteration(i) = mean(bench_data{idx}.num_iteration);
    end
    
    %% Store parsed data
    save_filename = [store_folder_name, 'bench_data_',...
        shape_type{id_pair, 1}, '_', shape_type{id_pair, 2}, '.mat'];
    save(save_filename);
    
    disp(['Parsed data saved to: ', save_filename])
end