% Benchmark script for collision detection algorithms: the proposed 
% CFC-based, implicit method and GJK.
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

clear; close all; clc
add_path();

NN = 1000;
N = [10,10];
algs = {'CFC-CN-FP', 'CFC-Dist-LS', 'Implicit', 'GJK'};

%% Experiment
for loop = 1:NN
    clc
    disp('***************************************************************')
    disp('********* Benchmark for collision detection methods ***********')
    disp('***************************************************************')
    disp([num2str(loop/NN*100), '%'])
    
    %% Set up parameters for two superquadrics
    s1 = SuperQuadrics({0.5+10*rand(1,3), 0.01+1.98*rand(1,2), [0,0],...
        zeros(3,1), [1,0,0,0], N});
    s2 = SuperQuadrics({0.5+10*rand(1,3), 0.01+1.98*rand(1,2), [0,0],...
        10*(2*rand(3,1)-1), rand(1,4), N});
 
    s1_surf = s1.GetSurf();
    hold on;
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
end

%% Display the comparison results
avg_cfc_fp = mean(cfcFPEllapsed);
avg_cfc_dist_ls = mean(cfcDistLSEllapsed);
avg_impl = mean(implEllapsed);
avg_gjk = mean(gjkEllapsed);

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
avg_cfc_fp_cond = mean(cfcFPCond, 'omitnan');
avg_cfc_dist_ls_cond = mean(cfcDistLSCond, 'omitnan');
avg_impl_cond = mean(implCond, 'omitnan');
plot(1:NN, cfcFPCond, 1:NN, cfcDistLSCond, 1:NN, implCond)
title('Necessary condition')
legend(algs(1:3))
ylim([0,1e-4])
