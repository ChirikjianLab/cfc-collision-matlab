% Test script for collision detection using common-normal concept with
% fixed-point iteration method
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

close all; clear; clc;
add_path();

disp('*******************************************************************')
disp('********** Collision detection test for "CN-FP" method ************')
disp('*******************************************************************')

%% TEST 1: Superquadric-Superquadric
disp('Superquadric-Superquadric...')
s1 = SuperQuadrics({1+10*rand(1,3), 0.1+1.8*rand(1,2), [0,0],...
    10*(2*rand(3,1)-1), rand(1,4), [20,20]});
s2 = SuperQuadrics({1+10*rand(1,3), 0.1+1.8*rand(1,2), [0,0],...
    10*(2*rand(3,1)-1), rand(1,4), [20,20]});

test_cn_fp(s1, s2);

%% TEST 2: Ellipsoid-Ellipsoid
disp('Ellipsoid-Ellipsoid...')
e1 = Ellipsoid({1+10*rand(1,3), [0,0], 10*(2*rand(3,1)-1), rand(1,4), [20,20]});
e2 = Ellipsoid({1+10*rand(1,3), [0,0], 10*(2*rand(3,1)-1), rand(1,4), [20,20]});

test_cn_fp(e1, e2);

%% Main algorithm
function test_cn_fp(s1, s2)
t_start = tic;
[flag, dist, pt_cls, condition] = ...
    distance_common_normal_fixed_point(s1, s2);
ellapse_time = toc(t_start);

% Plot results
figure; hold on; axis equal;
plot_result(s1, s2, pt_cls, dist, flag, condition, ellapse_time)
end