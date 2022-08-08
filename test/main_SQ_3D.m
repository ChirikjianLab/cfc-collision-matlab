% Main script for demonstrating collision detection algorithms
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

clear; close all; clc
add_path();

disp('*******************************************************************')
disp('********** Main script for collision detection methods ************')
disp('*******************************************************************')

NN = 50;
N = [10,10];
algs = {'CFC-CN-FP', 'CFC-Dist-LS', 'Implicit', 'GJK'};

%% Superquadrics
s1 = SuperQuadrics({0.5+10*rand(1,3), 0.1+1.8*rand(1,2), [0,0],...
    zeros(3,1), [1,0,0,0], N});
s1_param = [s1.a, s1.eps, s1.tc', s1.q];

pose1.x = s1.tc; pose1.q = s1.q;
R1 = quat2rotm(pose1.q);

% CFC-CN-FP
figure; hold on; axis equal; grid on;
lightangle(gca,45,30);
lighting gouraud;

s1.PlotShape('g', 0.8);

% CFC-Dist-LS
figure; hold on; axis equal; grid on;
lightangle(gca,45,30);
lighting gouraud;

s1.PlotShape('g', 0.8);

% Implicit surface, interior-point approach
figure; hold on; axis equal; grid on;
lightangle(gca,45,30);
lighting gouraud;

s1.PlotShape('g', 0.8);

% GJK-based method
figure; hold on; axis equal; grid on;
lightangle(gca,45,30);
lighting gouraud;

s1.PlotShape('g', 0.8);
s1_surf = s1.GetSurf();

%% Experiment
for loop = 1:NN
    %% Set up parameters for the moving superquadrics
    s2 = SuperQuadrics({0.5+5*rand(1,3), 0.1+1.8*rand(1,2), [0,0],...
        10*(2*rand(3,1)-1), rand(1,4), N});
    s2_param = [s2.a, s2.eps, s2.tc', s2.q];
    
    pose2.x = s2.tc;
    pose2.q = s2.q;
    R2 = quat2rotm(pose2.q);
    
    s2_surf = s2.GetSurf();
    
    %% Minkowski, optimization for Gradient
    cfcFPStart = tic;
    [cfcFPStatus(loop), cfcFPDist(loop), cfcFPPcls(loop,:),...
        cfcFPCond(loop)] = distance_cfc(s1, s2);
    cfcFPEllapsed(loop) = toc(cfcFPStart);
    
    %% Minkowski, optimization for Spherical coordinates
    cfcDistLSStart = tic;
    [cfcDistLSStatus(loop), cfcDistLSDist(loop), cfcDistLSPcls(loop,:),...
        cfcDistLSCond(loop)] = distance_cfc(s1, s2, 'least-squares');
    cfcDistLSEllapsed(loop) = toc(cfcDistLSStart);
    
    %% Implicit surface, interior-point approach
    implStart = tic;
    [implPcls(loop,:), implDist(loop), implStatus(loop),...
        implCond(loop)] = distance_implicit(s1, s2);
    implEllapsed(loop) = toc(implStart);
    
    %% Comparison: with GJK
    gjkStart = tic;
    [gjkDist(loop), pts, gjkS1, gjkS2] = GJK_dist(s1_surf,s2_surf);
    gjkStatus(loop) = GJK(s1_surf, s2_surf, 100);
    gjkEllapsed(loop) = toc(gjkStart);
    
    %% Plot the results
    % CFC-CN-FP
    figure(1);
    plot_result_object(s1, s2, cfcFPStatus(loop), cfcFPPcls(loop,:));
    
    % CFC-Dist-LS
    figure(2);
    plot_result_object(s1, s2, cfcDistLSStatus(loop), cfcDistLSPcls(loop,:));
    
    % Implicit surface, interior-point approach
    figure(3);
    plot_result_object(s1, s2, implStatus(loop), implPcls(loop,:));
    
    % GJK
    figure(4);
    if gjkStatus(loop) == 0
        plot3(s2.tc(1), s2.tc(2), s2.tc(3), 'bo');
        s2.PlotShape('b', 0.6);
    elseif gjkStatus(loop) == 1
        plot3(s2.tc(1), s2.tc(2), s2.tc(3), 'r*');
        s2.PlotShape('r', 0.6);
    end
    
    plot3([gjkS1(1) gjkS2(1)], [gjkS1(2) gjkS2(2)], [gjkS1(3) gjkS2(3)],...
        'g','LineWidth',2)
end

%% Display the comparison results
avg_cfc_fp = mean(cfcFPEllapsed);
avg_cfc_dist_ls = mean(cfcDistLSEllapsed);
avg_impl = mean(implEllapsed);
avg_gjk = mean(gjkEllapsed);

figure(1);
title(['CFC-CN-FP: ', num2str(avg_cfc_fp), 's'])
figure(2);
title(['CFC-Dist-LS: ', num2str(avg_cfc_dist_ls), 's'])
figure(3);
title(['Implicit: ', num2str(avg_impl), 's'])
figure(4);
title(['GJK: ', num2str(avg_gjk), 's'])

figure;
boxplot([cfcFPEllapsed; cfcDistLSEllapsed; implEllapsed; gjkEllapsed]')
ylim([0,0.05])
xticklabels(algs)
xtickangle(30)

figure; hold on;
plot(1:NN, cfcFPStatus, 'o')
plot(1:NN, cfcDistLSStatus, '.')
plot(1:NN, implStatus, 'd')
plot(1:NN, gjkStatus, '--')
title('Status comparison')
legend(algs)

figure; hold on;
plot(1:NN, cfcFPDist, '-o')
plot(1:NN, cfcDistLSDist, '-.')
plot(1:NN, implDist, '-d')
plot(1:NN, gjkDist, '--')
title('Distance comparison')
legend(algs)

figure; hold on;
plot(1:NN, cfcFPCond, 1:NN, cfcDistLSCond, 1:NN, implCond)
title('Necessary condition')
legend(algs(1:3))
ylim([0,1e-4])