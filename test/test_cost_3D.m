% Test script for optimization objective cost in 3D case
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

clear; close all; clc;
add_path();

NN = 1;

disp('*******************************************************************')
disp('************* Cost for CFC-Dist-LS in parameter space *************')
disp('*******************************************************************')

% Plot the original superquadric
N = [100,100];
eta = -pi:2*pi/(N(1)-1):pi;
omega = -pi/2:pi/(N(2)-1):pi/2;
[Eta,Omega] = meshgrid(eta,omega);

%% Superquadrics
a1 = 4; b1 = 2; c1 = 1; eps1 = 0.1+1.8*rand(1,2);
q1 = [0.2,0.1,0.15,-0.2]; q1 = q1/norm(q1);
X1 = [2;1;5.5];
Vec1 = [a1;b1;c1];

s1 = SuperQuadrics({[a1,b1,c1], eps1, [0,0], X1, q1, N});

%% Experiment
for loop = 1:NN
    %% Set up parameters for the moving ellipsoids
    a2 = 1; b2 = 0.8; c2 = 0.5;
    eps2 = 0.1+1.8*rand(1,2);
    q2 = rand(1,4);
    X2 = [10*rand-3; 6*rand+0.5; 8*rand];
    Vec2 = [a2;b2;c2];
    
    s2 = SuperQuadrics({[a2,b2,c2], eps2, [0,0], X2, q2, N});
    
    tic
    %% Closed-Form Minkowski Operations
    minkObj = MinkSumClosedForm(s1, s2, quat2rotm(s1.q), quat2rotm(s2.q));
    m = s1.GetGradients();
    p_CF = minkObj.GetMinkSumFromGradient(m) + s1.tc;
    
    P_CF(:,:,1) = reshape(p_CF(1,:), N(1), N(2));
    P_CF(:,:,2) = reshape(p_CF(2,:), N(1), N(2));
    P_CF(:,:,3) = reshape(p_CF(3,:), N(1), N(2));
    
    %% Cost function
    func = sqrt( (P_CF(:,:,1)-X2(1)).^2 +...
        (P_CF(:,:,2)-X2(2)).^2 + (P_CF(:,:,3)-X2(3)).^2 );
    
    %% Optimal solution
    [func_s,idx] = min(func(:))
    eta = Eta(:); omega = Omega(:);
    eta_s = eta(idx); omega_s = omega(idx);
    toc
    
    figure; hold on;
    surf(Eta,Omega,func)
    plot3(eta_s, omega_s, func_s,'g*','LineWidth',2)
    
end
