% Test script for continuous collision detection using the proposed
% CFC-based algorithm for superquadrics
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

close all; clear; clc;
add_path();

disp('*******************************************************************')
disp('*** Continuous Collision detection test for CFC-Dist-LS method ****')
disp('*******************************************************************')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Changable parameter: indicates whether to animate object motions
is_animate = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters
s1 = SuperQuadrics({1+10*rand(1,3), 0.2+1.6*rand(1,2), [0,0],...
    30*(2*rand(3,1)-1), rand(1,4), [20,20],...
    [30*(2*rand(3,1)-1);pi/2*(2*rand(3,1)-1)]});

s2 = SuperQuadrics({1+10*rand(1,3), 0.2+1.6*rand(1,2), [0,0],...
    10*rand(3,1), rand(1,4), [20,20],...
    [50*(2*rand(3,1)-1);pi/2*(2*rand(3,1)-1)]});

t_max = 1;

g1_init = [quat2rotm(s1.q), s1.tc; 0,0,0,1];
g2_init = [quat2rotm(s2.q), s2.tc; 0,0,0,1];

g1_goal = [g1_init(1:3,1:3) * expm(skew(s1.vel(4:6))*t_max),...
    g1_init(1:3,4) + s1.vel(1:3) * t_max; 0,0,0,1];
g2_goal = [g2_init(1:3,1:3) * expm(skew(s2.vel(4:6))*t_max),...
    g2_init(1:3,4) + s2.vel(1:3) * t_max; 0,0,0,1];

%% CCD using CFC-Dist-LS
disp('CCD by CFC-Dist-LS...')
t_start = tic;
[flag_ccd, dist_ccd, t_opt_ccd, pt_cls_ccd, cond_ccd] = ...
    continuous_distance_cfc(s1, s2, t_max, 'least-squares');
ellapse_time_ccd = toc(t_start);

g1_opt_ccd = update_pose(g1_init, s1.vel, t_opt_ccd, 'PCG');
g2_opt_ccd = update_pose(g2_init, s2.vel, t_opt_ccd, 'PCG');

% Plot results
figure; hold on; axis equal; axis off;
plot_result_ccd(s1, s2, g1_opt_ccd, g2_opt_ccd,...
    g1_init, g2_init, pt_cls_ccd, dist_ccd,...
    flag_ccd, cond_ccd, ellapse_time_ccd);

disp('Solution ---')
disp(['Minimum distance: ', num2str(dist_ccd)]);
disp(['Time step at minimum distance or first time of contact: ',...
    num2str(t_opt_ccd)]);

disp('-------------------------------------------------------------------')

%% Verification: use naive discrete collision detection
disp('Using naive discrete collision detection...')
idx = 1;
N_step = 100;
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

if isempty(t_s_min)
    dist_s_min = min(dist_s);
    t_s_min = t_s(dist_s == dist_s_min);
end

% Plot results
figure; hold on; axis equal; axis off;
lightangle(gca,45,30);
lighting gouraud;

for i = 1:length(t_s)
    s1.q = pose1(4:7,i)';
    s1.tc = pose1(1:3,i);
    s2.q = pose2(4:7,i)';
    s2.tc = pose2(1:3,i);
    
    s1.PlotShape('y', 1);
    s2.PlotShape('g', 1);
    
    if flag_s(i) == true
        break;
    end
end
plot3(pose1(1,:), pose1(2,:), pose1(3,:), 'k', 'LineWidth', 2)
plot3(pose2(1,:), pose2(2,:), pose2(3,:), 'k', 'LineWidth', 2)

% Start
s1.q = rotm2quat(g1_init(1:3,1:3));
s1.tc = g1_init(1:3,4);
s2.q = rotm2quat(g2_init(1:3,1:3));
s2.tc = g2_init(1:3,4);

s1.PlotShape('m', 1);
s2.PlotShape('m', 1);

% Goal
s1.q = rotm2quat(g1_t(1:3,1:3));
s1.tc = g1_t(1:3,4);
s2.q = rotm2quat(g2_t(1:3,1:3));
s2.tc = g2_t(1:3,4);

s1.PlotShape('c', 1);
s2.PlotShape('c', 1);

disp('Solution ---')
disp(['Minimum distance: ', num2str(dist_s_min)]);
disp(['Time step at minimum distance or first time of contact: ',...
    num2str(t_s_min)]);

%% Animation
if is_animate
    v = VideoWriter("video_cfc_continuous.avi");
    v.open();

    figure;
    
    steps = 100;
    dist_min = inf;
    t_min = 0;
    for i = 1:steps
        t_step = (i-1)*t_max/(steps-1);
        
        g1_step = [g1_init(1:3,1:3) * expm(skew(s1.vel(4:6))*t_step),...
            g1_init(1:3,4) + s1.vel(1:3) * t_step; 0,0,0,1];
        g2_step = [g2_init(1:3,1:3) * expm(skew(s2.vel(4:6))*t_step),...
            g2_init(1:3,4) + s2.vel(1:3) * t_step; 0,0,0,1];
        
        s1.q = rotm2quat(g1_step(1:3,1:3));
        s1.tc = g1_step(1:3,4);
        s2.q = rotm2quat(g2_step(1:3,1:3));
        s2.tc = g2_step(1:3,4);
        
        [flag, dist, pt_cls, cond] = distance_cfc(s1, s2);
        
        if dist < dist_min
            dist_min = dist;
            t_min = t_step;
            
            if dist_min < 1e-10
                break;
            end
        end
        
        % Plots
        s1.PlotShape('g', 0.7);
        
        hold on; axis off; axis equal;
        lightangle(gca,45,30);
        lighting gouraud;
        
        s2.PlotShape('b', 0.7);
        
        plot3(pt_cls.s1(1), pt_cls.s1(2), pt_cls.s1(3), '*r', 'LineWidth', 2)
        plot3(pt_cls.s2(1), pt_cls.s2(2), pt_cls.s2(3), 'or', 'LineWidth', 2)
        plot3([pt_cls.s1(1), pt_cls.s2(1)], [pt_cls.s1(2), pt_cls.s2(2)],...
            [pt_cls.s1(3), pt_cls.s2(3)],...
            'r', 'LineWidth',1.5)
        
        title({['Current distance: ', num2str(dist)],...
            ['Current time step: ', num2str(t_step)],...
            ['Minimum distance solved by CCD: ',...
            num2str(dist_ccd)],...
            ['Time step at minimum distance solved by CCD:',...
            num2str(t_opt_ccd)]})
        
        hold off;
        pause(.1)

        f = getframe(gcf);
        v.writeVideo(f);
    end

    v.close();
end