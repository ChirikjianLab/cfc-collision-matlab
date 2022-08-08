% Test script for optimization objective cost in time space for the
% continuous case
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

clear; close all; clc;
add_path();

disp('*******************************************************************')
disp('********** Cost for CCD using CFC-Dist-LS in time space ***********')
disp('*******************************************************************')

% Parameters
separated = false;
N = [20,20];
N_step = 200;

t_s = 0:1/(N_step-1):1;

%% Fixed s1
s1 = SuperQuadrics({[15,12,8], [1.45,0.5], [0,0],...
    [2;1;5.5], [0.2, 0.12, 0.14, -0.36], N...
    [[45.7; 24.6; -18.3]; pi/2 * [1.5; -2.5; 1.6]]});

% Start and goal poses
t_max = 1;

g1_init = [quat2rotm(s1.q), s1.tc; 0,0,0,1];
g1_goal = [g1_init(1:3,1:3) * expm(skew(s1.vel(4:6))*t_max),...
    g1_init(1:3,4) + s1.vel(1:3) * t_max; 0,0,0,1];

%% Experiment
% Poly-ellipsoid
s2 = PolyEllipsoid({[10.2,5.7,6,5.3,2.6,9.4],...
    [15.6;-18.2;18.6], [0.62,2.16,0.12,-1.8], N,...
    [[-30; -13.6; -16.3]; pi/2 * [2.15; -1.95; -1.43]]});

if ~separated
    s2.vel = [[8.4; 45.8; -16.3]; pi/2 * [1.15; 0.95; -0.53]];
end

g2_init = [quat2rotm(s2.q), s2.tc; 0,0,0,1];
g2_goal = [g2_init(1:3,1:3) * expm(skew(s2.vel(4:6))*t_max),...
    g2_init(1:3,4) + s2.vel(1:3) * t_max; 0,0,0,1];

%% Gradient-parameterization with least-squares optimization
[flag_ccd, dist_ccd, t_opt_ccd, pt_cls_ccd, cond_ccd] = ...
    continuous_distance_cfc(s1, s2, t_max, 'least-squares');

g1_opt_ccd = update_pose(g1_init, s1.vel, t_opt_ccd, 'PCG');
g2_opt_ccd = update_pose(g2_init, s2.vel, t_opt_ccd, 'PCG');

%% Function value in parameter space
F = nan([N, N_step]);
min_F = nan(1, N_step);

for i = 1:length(t_s)
    g1_t = update_pose(g1_init, s1.vel, t_s(i), 'PCG');
    g2_t = update_pose(g2_init, s2.vel, t_s(i), 'PCG');
    
    s1.q = rotm2quat(g1_t(1:3,1:3));
    s1.tc = g1_t(1:3,4);
    s2.q = rotm2quat(g2_t(1:3,1:3));
    s2.tc = g2_t(1:3,4);
    
    % Discrete collision detection
    [~, dist_t, ~, ~] = distance_cfc(s1, s2, 'least-squares');
    min_F(i) = abs(dist_t);
end

%% Plots
plot(t_s, min_F)
[dist_discrete, idx_discrete] = min(min_F);
dist_discrete
t_s(idx_discrete)
