% Demo script for static collision detection using CFC
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

close all; clear; clc;
add_path();

N = [20,20];

disp('*******************************************************************')
disp('********** Demo for collision detection (static case) *************')
disp('*******************************************************************')

%% Geometric models
% Superquadrics
sq = SuperQuadrics({[15,12,8], [1.45,0.5], [0,0],...
    [2;1;5.5], [0.2, 0.12, 0.14, -0.36], N});
m1 = sq.GetGradientsCanonical();

% Poly-ellipsoid
pe1 = PolyEllipsoid({[10.2,5.7,6,5.3,2.6,9.4],...
    [15.6;-2.2;1.6], [1.6,-2.16,0.2,-1], N});

pe2 = PolyEllipsoid({[10.2,5.7,6,5.3,2.6,9.4],...
    [15.6;-18.2;18.6], [0.62,2.16,0.12,-1.8], N});

%% Static collision detection based on CFC: Collision case
[flag1, dist1, pt_cls1, condition1] = distance_cfc(sq, pe1, 'least-squares');

mink1 = MinkSumClosedForm(sq, pe1, quat2rotm(sq.q), quat2rotm(pe1.q));
x_mink1 = mink1.GetMinkSumFromGradient(m1) + sq.tc;

% Plot
figure; axis equal; axis off; hold on;

plot_surf(mink1.s1.GetSurf, 0.4);
plot_surf(mink1.s2.GetSurf, 0.4);

plot3(pt_cls1.s1(1), pt_cls1.s1(2), pt_cls1.s1(3), 'r*', 'LineWidth', 2)
plot3(pt_cls1.s2(1), pt_cls1.s2(2), pt_cls1.s2(3), 'r*', 'LineWidth', 2)
plot3([pt_cls1.s1(1) pt_cls1.s2(1)],...
    [pt_cls1.s1(2) pt_cls1.s2(2)],...
    [pt_cls1.s1(3) pt_cls1.s2(3)],...
    'r-', 'LineWidth', 1.5)

X = reshape(x_mink1(1,:), N(1), N(2));
Y = reshape(x_mink1(2,:), N(1), N(2));
Z = reshape(x_mink1(3,:), N(1), N(2));
surf(X, Y, Z,...
 'EdgeColor', 'k', 'EdgeAlpha', 0.2,...
 'FaceAlpha', 0.1, 'FaceColor', 'b');

plot3(mink1.s2.tc(1), mink1.s2.tc(2), mink1.s2.tc(3), 'ro', 'LineWidth', 2)
plot3(pt_cls1.mink(1), pt_cls1.mink(2), pt_cls1.mink(3), 'r+', 'LineWidth', 2)
plot3([pt_cls1.mink(1) mink1.s2.tc(1)],...
    [pt_cls1.mink(2) mink1.s2.tc(2)],...
    [pt_cls1.mink(3) mink1.s2.tc(3)],...
    'r--', 'LineWidth', 1.5)

%% Static collision detection based on CFC: Separated case
[flag2, dist2, pt_cls2, condition2] = distance_cfc(sq, pe2, 'least-squares');

mink2 = MinkSumClosedForm(sq, pe2, quat2rotm(sq.q), quat2rotm(pe2.q));
x_mink2 = mink2.GetMinkSumFromGradient(m1) + sq.tc;

% Plot
figure; axis equal; axis off; hold on;

plot_surf(mink2.s1.GetSurf, 0.4);
plot_surf(mink2.s2.GetSurf, 0.4);

plot3(pt_cls2.s1(1), pt_cls2.s1(2), pt_cls2.s1(3), 'r*', 'LineWidth', 2)
plot3(pt_cls2.s2(1), pt_cls2.s2(2), pt_cls2.s2(3), 'r*', 'LineWidth', 2)
plot3([pt_cls2.s1(1) pt_cls2.s2(1)],...
    [pt_cls2.s1(2) pt_cls2.s2(2)],...
    [pt_cls2.s1(3) pt_cls2.s2(3)],...
    'r-', 'LineWidth', 1.5)

X = reshape(x_mink2(1,:), N(1), N(2));
Y = reshape(x_mink2(2,:), N(1), N(2));
Z = reshape(x_mink2(3,:), N(1), N(2));
surf(X, Y, Z,...
 'EdgeColor', 'k', 'EdgeAlpha', 0.2,...
 'FaceAlpha', 0.1, 'FaceColor', 'b');

plot3(mink2.s2.tc(1), mink2.s2.tc(2), mink2.s2.tc(3), 'ro', 'LineWidth', 2)
plot3(pt_cls2.mink(1), pt_cls2.mink(2), pt_cls2.mink(3), 'r+', 'LineWidth', 2)
plot3([pt_cls2.mink(1) mink2.s2.tc(1)],...
    [pt_cls2.mink(2) mink2.s2.tc(2)],...
    [pt_cls2.mink(3) mink2.s2.tc(3)],...
    'r--', 'LineWidth', 1.5)