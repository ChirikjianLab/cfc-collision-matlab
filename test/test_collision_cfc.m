% Test script for CFC-based collision detection algorithm
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

close all; clear; clc;
add_path();

disp('*******************************************************************')
disp('**** Collision detection test for proposed CFC-based methods ******')
disp('*******************************************************************')

%% TEST 1: Superquadric-Superquadric
disp('Superquadric-Superquadric...')
s1 = SuperQuadrics({1+10*rand(1,3), 0.1+1.8*rand(1,2), [0,0],...
    10*(2*rand(3,1)-1), rand(1,4), [20,20]});
s2 = SuperQuadrics({1+10*rand(1,3), 0.1+1.8*rand(1,2), [0,0],...
    10*(2*rand(3,1)-1), rand(1,4), [20,20]});

test_cfc(s1, s2);

%% TEST 2: Ellipsoid-Ellipsoid
disp('Ellipsoid-Ellipsoid...')
e1 = Ellipsoid({1+10*rand(1,3), [0,0], 10*(2*rand(3,1)-1), rand(1,4), [20,20]});
e2 = Ellipsoid({1+10*rand(1,3), [0,0], 10*(2*rand(3,1)-1), rand(1,4), [20,20]});

test_cfc(e1, e2);

%% TEST 3: PolyEllipsoid-PolyEllipsoid
disp('PolyEllipsoid-PolyEllipsoid...')
pe1 = PolyEllipsoid({1+10*rand(1,6), 10*(2*rand(3,1)-1), rand(1,4),...
    [20,20]});
pe2 = PolyEllipsoid({1+10*rand(1,6), 10*(2*rand(3,1)-1), rand(1,4),...
    [20,20]});

test_cfc(pe1, pe2);

%% Main algorithm
function test_cfc(s1, s2)
disp('CFC + Common Normal + Fix-point iteration...')
t_start = tic;
[flag, dist, pt_cls, condition] = distance_cfc(s1, s2, 'fixed-point');
ellapse_time = toc(t_start);

figure; hold on; axis equal;
plot_result(s1, s2, pt_cls, dist, flag, condition, ellapse_time)

disp('CFC + Point-to-surface Distance + Constrained optimization...')
t_start = tic;
[flag, dist, pt_cls, condition] = distance_cfc(s1, s2, 'constrained');
ellapse_time = toc(t_start);

figure; hold on; axis equal;
plot_result(s1, s2, pt_cls, dist, flag, condition, ellapse_time)

disp('CFC + Point-to-surface Distance + Nonlinear Least-Squares...')
t_start = tic;
[flag, dist, pt_cls, condition] = distance_cfc(s1, s2, 'least-squares');
ellapse_time = toc(t_start);

figure; hold on; axis equal;
plot_result(s1, s2, pt_cls, dist, flag, condition, ellapse_time)
end