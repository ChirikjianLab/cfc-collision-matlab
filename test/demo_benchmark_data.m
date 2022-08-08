% Demo script for visualizing benchmark results for C++ implemnetation
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2022

close all; clear; clc;

%% Retrieve data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Options for static and continuous cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt = 'static';
% opt = 'continuous';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if strcmp(opt, 'static')
    label_idx = 1:7;
    algo_names = {'FCL', 'Implicit', 'CN-LS', 'CN-FP',...
        '\textbf{CFC-Dist-LS}', '\textbf{CFC-CN-LS}', '\textbf{CFC-CN-FP}'};
elseif strcmp(opt, 'continuous')
    label_idx = 1:2;
    algo_names = {'\textbf{CFC-Dist-LS-Trans}', '\textbf{CFC-Dist-LS-Linear}'};
end

shape_names = {'E-E', 'E-PE', 'PE-PE', 'SQ-E', 'SQ-PE', 'SQ-SQ'};

bench_data_path = ['../data/', opt, '/data_parsed/'];

bench_data_prefix = [bench_data_path, 'bench_data'];
bench_data_details = dir([bench_data_prefix, '_*.mat']);

if size(bench_data_details, 1) == 0
    error('Please run parse_benchmark_data.m first...')
end

%% Summarized benchmark statistics
% Each column refers to one algorithm listed in "algo_names"
% Each row refers to the geometric pairs listed in "shape_names"
time_init = [];
time_query = [];
time_query_std = [];
time_total = [];
accuracy = [];
num_iteration = [];

for j = 1:size(bench_data_details, 1)
    load([bench_data_path, bench_data_details(j,1).name]);
    
    time_init = [time_init; bench_data_avg.run_time.initial(1,label_idx)];
    time_query = [time_query; bench_data_avg.run_time.query(1,label_idx)];
    time_query_std = [time_query_std; bench_data_avg.run_time.query(2,label_idx)];
    
    time_total(:,:,j) = [time_init(j,:)', time_query(j,:)'];
    
    accuracy = [accuracy; bench_data_avg.accuracy(1,label_idx)];
    num_iteration = [num_iteration; bench_data_avg.num_iteration(1,label_idx)];
end

%% Plot running time comparisons
f = figure;
b = bar(time_query*1000);

hold on;
for j = 1:length(b)
    x_bar(:,j) = b(j).XEndPoints';
    y_bar(:,j) = b(j).YEndPoints';
end
errorbar(x_bar(:), y_bar(:), time_query_std(:)*1000, 'k.')

xlabel('Geometric pairs', 'Interpreter', 'latex')
xticklabels(shape_names)
ylabel('Query time/ $\mu s$', 'Interpreter', 'latex')
legend(algo_names, 'Location', 'eastoutside', 'Interpreter', 'latex')

if strcmp(opt, 'static')
    ylim([0,100])
end

f.Position(3:4) = [850,400];
set(gca, 'FontSize', 15)

%% Plot accuracy comparisons
if strcmp(opt, 'static')
    idx_display = 2:size(bench_data,1);
elseif strcmp(opt, 'continuous')
    idx_display = 1:size(bench_data,1);
end

f = figure;
bar(accuracy(:,idx_display)*100);

hold on;
xlabel('Geometric pairs', 'Interpreter', 'latex')
xticklabels(shape_names)
ylabel('Accuracy/$\%$', 'Interpreter', 'latex')
legend(algo_names{idx_display},...
    'Location', 'eastoutside', 'Interpreter', 'latex')

f.Position(3:4) = [850,400];
set(gca, 'FontSize', 15)

%% Plot number of iteration comparisons
f = figure;
bar(num_iteration(:,idx_display));

hold on;
xlabel('Geometric pairs', 'Interpreter', 'latex')
xticklabels(shape_names)
ylabel('Number of iterations', 'Interpreter', 'latex')
legend(algo_names{idx_display},...
    'Location', 'eastoutside', 'Interpreter', 'latex')

f.Position(3:4) = [850,400];
set(gca, 'FontSize', 15)