% Demo script for geometric models
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

close all; clear; clc;
add_path();


disp('*******************************************************************')
disp('******************* Demo for geometric models *********************')
disp('*******************************************************************')

N = [20,20];
eta = -pi/2:pi/(N(1)-1):pi/2;
omega = -pi:2*pi/(N(2)-1):pi;
[Eta,Omega] = meshgrid(eta,omega);

%% Geometric models
% Ellipsoid
e1 = Ellipsoid({[10.2,5.7,6], [0,0],...
    [5.7;-2.1;6.5], [0.12, -0.22, 0.64, 0.39], N});

% Superquadrics
sq1 = SuperQuadrics({[15,12,8], [1.45,0.5], [0,0],...
    [2;1;5.5], [0.2, 0.12, 0.14, -0.36], N});
m1 = sq1.GetGradientsCanonical();
u1 = sq1.GetHypersphereFromGradient(m1);

sq2 = SuperQuadrics({[9.3,6.3,3.5], [0.6,0.26], [0,0],...
    [-2.5;1.6;-3.5], [-0.12, 0.27, 0.4, -0.6], N});

% Poly-ellipsoid
pe1 = PolyEllipsoid({[10.2,5.7,6,5.3,2.6,9.4],...
    [15.6;-2.2;1.6], [1.6,-2.16,0.2,-1], N});

% Plots
figure; axis equal; axis off; hold on;
plot_surf(sq1.GetSurf);

figure; axis equal; axis off; hold on;
plot_surf(e1.GetSurf);

figure; axis equal; axis off; hold on;
plot_surf(pe1.GetSurf);

%% Closed-form contact space
mink = MinkSumClosedForm(sq1, pe1, quat2rotm(sq1.q), quat2rotm(pe1.q));
x_mink = mink.GetMinkSumFromGradient(m1) + sq1.tc;

% Plots
figure; axis equal; axis off; hold on;

plot_surf(mink.s1.GetSurf, 1);

mink.s2.tc = x_mink(:,150);
plot_surf(mink.s2.GetSurf, 1);

X = reshape(x_mink(1,:), N(1), N(2));
Y = reshape(x_mink(2,:), N(1), N(2));
Z = reshape(x_mink(3,:), N(1), N(2));
surf(X, Y, Z,...
 'EdgeColor', 'k', 'EdgeAlpha', 0.5,...
 'FaceAlpha', 0.1, 'FaceColor', 'b');