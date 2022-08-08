% Demo script for continuous collision detection using CFC
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

close all; clear; clc;
add_path();

N = [20,20];

disp('*******************************************************************')
disp('********* Demo for collision detection (continuous case) **********')
disp('*******************************************************************')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Changable parameter: indicates whether two objects are separated
separated = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters
% Superquadrics
s1 = SuperQuadrics({[15,12,8], [1.45,0.5], [0,0],...
    [2;1;5.5], [0.2, 0.12, 0.14, -0.36], N...
    [[45.7; 24.6; -18.3]; pi/2 * [1.5; -2.5; 1.6]]});

% Poly-ellipsoid
s2 = PolyEllipsoid({[10.2,5.7,6,5.3,2.6,9.4],...
    [15.6;-18.2;18.6], [0.62,2.16,0.12,-1.8], N,...
    [[-30; -13.6; -16.3]; pi/2 * [2.15; -1.95; -1.43]]});

if ~separated
    s2.vel = [[8.4; 45.8; -16.3]; pi/2 * [1.15; 0.95; -0.53]];
end

% Start and goal poses
t_max = 1;

g1_init = [quat2rotm(s1.q), s1.tc; 0,0,0,1];
g2_init = [quat2rotm(s2.q), s2.tc; 0,0,0,1];

g1_goal = [g1_init(1:3,1:3) * expm(skew(s1.vel(4:6))*t_max),...
    g1_init(1:3,4) + s1.vel(1:3) * t_max; 0,0,0,1];
g2_goal = [g2_init(1:3,1:3) * expm(skew(s2.vel(4:6))*t_max),...
    g2_init(1:3,4) + s2.vel(1:3) * t_max; 0,0,0,1];

%% Gradient-parameterization with least-squares optimization
t_start = tic;
[flag_ccd, dist_ccd, t_opt_ccd, pt_cls_ccd, cond_ccd] = ...
    continuous_distance_cfc(s1, s2, t_max, 'least-squares');
ellapse_time = toc(t_start);

g1_opt_ccd = update_pose(g1_init, s1.vel, t_opt_ccd, 'PCG');
g2_opt_ccd = update_pose(g2_init, s2.vel, t_opt_ccd, 'PCG');

% Trajectory
idx = 1;
N_step = 50;
t_s = 0:1/(N_step-1):1;

for i = 1:length(t_s)
    g1_t = update_pose(g1_init, s1.vel, t_s(i), 'PCG');
    pose1(:,i) = [g1_t(1:3,4); rotm2quat(g1_t(1:3,1:3))'];
    g2_t = update_pose(g2_init, s2.vel, t_s(i), 'PCG');
    pose2(:,i) = [g2_t(1:3,4); rotm2quat(g2_t(1:3,1:3))'];
    
    s1.q = rotm2quat(g1_t(1:3,1:3));
    s1.tc = g1_t(1:3,4);
    s2.q = rotm2quat(g2_t(1:3,1:3));
    s2.tc = g2_t(1:3,4);
    
    [flag_s(i), dist_s(i)] = distance_cfc(s1, s2, 'fixed_point');
end

idx_contact = flag_s == true;
t_s_min = t_s(idx_contact);
dist_s_min = dist_s(idx_contact);

%% Plot results
% Plot start and goal poses
figure; hold on; axis equal; axis off;

% Trajectory
for i = 1:length(t_s)
    if flag_s(i) == true
        s1.PlotShape('r', 1);
        s2.PlotShape('r', 1);
        
        break;
    end
    
    s1.q = pose1(4:7,i)';
    s1.tc = pose1(1:3,i);
    s2.q = pose2(4:7,i)';
    s2.tc = pose2(1:3,i);

    plot_surf(s1.GetSurf);
    plot_surf(s2.GetSurf);
end
plot3(pose1(1,:), pose1(2,:), pose1(3,:), 'k', 'LineWidth', 2)
plot3(pose2(1,:), pose2(2,:), pose2(3,:), 'k', 'LineWidth', 2)

% Plot CFC at optimum
figure; hold on; axis equal; axis off;

% Optimal solution
s1.q = rotm2quat(g1_opt_ccd(1:3,1:3));
s1.tc = g1_opt_ccd(1:3,4);
s2.q = rotm2quat(g2_opt_ccd(1:3,1:3));
s2.tc = g2_opt_ccd(1:3,4);

plot_surf(s1.GetSurf, 0.5);
plot_surf(s2.GetSurf, 0.5);

% Contact space at optimum
m1 = s1.GetGradientsCanonical();
mink = MinkSumClosedForm(s1, s2, quat2rotm(s1.q), quat2rotm(s2.q));
x_mink = mink.GetMinkSumFromGradient(m1) + s1.tc;

N = [sqrt(size(m1,2)), sqrt(size(m1,2))];
X = reshape(x_mink(1,:), N(1), N(2));
Y = reshape(x_mink(2,:), N(1), N(2));
Z = reshape(x_mink(3,:), N(1), N(2));
surf(X, Y, Z,...
 'EdgeColor', 'k', 'EdgeAlpha', 0.3,...
 'FaceAlpha', 0.1, 'FaceColor', 'b');

% Witness points at optimum
plot3(pt_cls_ccd.s1(1), pt_cls_ccd.s1(2), pt_cls_ccd.s1(3),...
    '*r', 'LineWidth', 2)
plot3(pt_cls_ccd.s2(1), pt_cls_ccd.s2(2), pt_cls_ccd.s2(3),...
    '*r', 'LineWidth', 2)
plot3([pt_cls_ccd.s1(1), pt_cls_ccd.s2(1)],...
    [pt_cls_ccd.s1(2), pt_cls_ccd.s2(2)],...
    [pt_cls_ccd.s1(3), pt_cls_ccd.s2(3)],...
    'r', 'LineWidth',1.5)

try
    plot3(s2.tc(1), s2.tc(2), s2.tc(3), 'ro', 'LineWidth', 2)
    plot3(pt_cls_ccd.mink(1), pt_cls_ccd.mink(2), pt_cls_ccd.mink(3),...
        'r+', 'LineWidth', 2)
    plot3([pt_cls_ccd.mink(1) s2.tc(1)],...
        [pt_cls_ccd.mink(2) s2.tc(2)],...
        [pt_cls_ccd.mink(3) s2.tc(3)],...
        'r--', 'LineWidth', 1.5)
catch
end

plot3(pose1(1,:), pose1(2,:), pose1(3,:), 'k', 'LineWidth', 2)
plot3(pose2(1,:), pose2(2,:), pose2(3,:), 'k', 'LineWidth', 2)