% Test scipt for superquadric geometric model
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

clear; close all; clc;
add_path();

disp('*******************************************************************')
disp('************* Demo for superquadric geometric model ***************')
disp('*******************************************************************')

N = [20,20];
eta = -pi/2:pi/(N(1)-1):pi/2;
omega = -pi:2*pi/(N(2)-1):pi;
[Eta,Omega] = meshgrid(eta,omega);

a1 = 1+10*rand(1,3);
eps1 = 0.1+1.8*rand(1,2);
q1 = rand(1,4);
X1 = [2;1;5.5];

R1 = quat2rotm(q1);

%% Generate points from angles
s1 = SuperQuadrics({a1, eps1, [0,0], X1, q1, N});
pts1 = s1.GetPoints();

s12 = SuperQuadrics({a1, eps1, [0,0], X1, q1, N, [], 'uniform'});
pts2 = s12.GetPoints();

figure; axis equal; axis off; hold on;
plot3(pts1(1,:), pts1(2,:), pts1(3,:), 'k.')
plot3(pts2(1,:), pts2(2,:), pts2(3,:), 'r.')

%% Generate points
% From gradients
m1 = s1.GetGradientsFromSpherical([Eta(:), Omega(:)]);
pts_m1 = R1 * s1.GetPointsFromParamTapered(m1, 'gradient') + s1.tc;
m1_rot = s1.GetGradients();

% From normals
n1 = m1./sqrt(sum(m1.^2, 1));
pts_n1 = R1 * s1.GetPointsFromParamTapered(n1, 'normal') + s1.tc;
n1_rot = s1.GetNormals();

figure; axis equal; axis off; hold on;
lightangle(gca,45,30);
lighting gouraud;

s1.PlotShape('y', 1);

plot3(pts_m1(1,:), pts_m1(2,:), pts_m1(3,:), 'k.')
plot3([pts_m1(1,:); pts_m1(1,:)+m1_rot(1,:)],...
    [pts_m1(2,:); pts_m1(2,:)+m1_rot(2,:)],...
    [pts_m1(3,:); pts_m1(3,:)+m1_rot(3,:)], 'b')

plot3(pts_n1(1,:), pts_n1(2,:), pts_n1(3,:), 'ko')
plot3([pts_n1(1,:); pts_n1(1,:)+n1_rot(1,:)],...
    [pts_n1(2,:); pts_n1(2,:)+n1_rot(2,:)],...
    [pts_n1(3,:); pts_n1(3,:)+n1_rot(3,:)], 'r--')

%% Closed-form Minkowski sums
s2 = SuperQuadrics({1+10*rand(1,3), 0.1+1.8*rand(1,2), [0,0],...
    5*rand(3,1), rand(1,4), N});

mink = MinkSumClosedForm(s1, s2, quat2rotm(s1.q), quat2rotm(s2.q));

% Using gradient
mink_gradient = mink.GetMinkSumFromGradient(m1) + s1.tc;

% Using normal
mink_normal = mink.GetMinkSumFromNormal(n1) + s1.tc;

figure; axis equal; axis off; hold on;
lightangle(gca,45,30);
lighting gouraud;

s1.PlotShape('y', 1);

s2.tc = mink_gradient(:,25);
s2.PlotShape('g', 1);

s2.tc = mink_normal(:,300);
s2.PlotShape('b', 1);

plot3(mink_gradient(1,:), mink_gradient(2,:), mink_gradient(3,:), 'k.')
plot3(mink_normal(1,:), mink_normal(2,:), mink_normal(3,:), 'r.')