function [flag, dist, t_opt, pt_cls, condition] = ...
    continuous_distance_cfc(s1, s2, t_max, opt)
% continuous_distance_cfc computes the minimum separation distance between
% two bodies within a time interval. Closed-form contact space (CFC) and
% nonlinear optimizations are used. The surfaces are parameterized by 
% un-normalized gradient.
%
%  Inputs:
%    s1, s2       Geometric class 
%                   'SuperQuadrics', 'Ellipsoid', 'PolyEllipsoid'
%    t_max        Max time
%    opt          Options for optimization
%                   'least-squares' (default), 'constrained'
%
%  Outputs:
%    flag         Status of contact
%                   1 -- in collision
%                   0 -- separated
%    dist         Separation distance
%    t_opt        Time of minimum distance/first-time contact
%    pt_cls       A structure of closest points information
%                   s1   -- point on s1
%                   s2   -- point s2
%                   mink -- point on Minkowski sum boundary
%    condition    Necessary condition for result correctness, the line
%                   connecting two SQs is colinear with the normal vector
%                   at the computed point
%
%  Author:
%    Sipu Ruan, ruansp@nus.edu.eg, 2021

if nargin == 3
    opt = 'least-squares';
end

% Initial condition: spherical coordinates of s2 center as viewed in s1
% Transform into local frame of s1
R1 = quat2rotm(s1.q);
R2 = quat2rotm(s2.q);
s2_tc_in_s1 = R1' * (s2.tc - s1.tc);
psi0 = [atan2( s2_tc_in_s1(3), norm(s2_tc_in_s1(1:2)) ),...
        atan2( s2_tc_in_s1(2), s2_tc_in_s1(1) )];
m0 = s1.GetGradientsFromSpherical(psi0);

% Initialize Minkowski sums object
minkObj = MinkSumClosedForm(s1, s2, R1, R2);

if strcmp(opt, 'least-squares')
    % Optimization to solve min_{psi,t} |p0(t) - x_{1+2}(psi,t)|
    var0 = [psi0, 0];
    
    option = optimoptions('lsqnonlin',...
        'Algorithm', 'trust-region-reflective',...
        'display', 'none',...
        'FunctionTolerance', 1e-12);
    var_opt = lsqnonlin(@(var) func_lsq(var, minkObj), var0,...
        [-inf,-inf,0], [inf,inf,t_max], option);
    
    psi_opt = var_opt(1:2);
    t_opt = var_opt(3);
    
    % Distance and closest points
    % gradient in local frame
    m_opt = s1.GetGradientsFromSpherical(psi_opt);
    
elseif strcmp(opt, 'constrained')
    % Optimization to solve min_{m1,t} |p0(t) - x_{1+2}(m1,t)|
    %                       s.t.      \Phi(x_1{m1,t}) = 0
    var0 = [m0; 0];
    
    option = optimoptions('fmincon',...
        'Algorithm', 'interior-point',...
        'display', 'none',...
        'FunctionTolerance', 1e-12);
    var_opt = fmincon(@(var) func_con(var, minkObj), var0,...
        [], [], [], [], [-inf;-inf;-inf;0], [inf;inf;inf;t_max],...
        @(var) nlcon(var, s1), option);
    
    m_opt = var_opt(1:3);
    t_opt = var_opt(4);
end

% Distance and closest points
% points in global frame
x_opt = s1.GetPointsFromGradient(m_opt);

% Optimal solution
R1_opt = R1 * expm(skew(s1.vel(4:6)) * t_opt);
R2_opt = R2 * expm(skew(s2.vel(4:6)) * t_opt);

minkObj.M1 = R1_opt;
minkObj.M2 = R2_opt;

pt_cls.s1 = R1_opt * x_opt + s1.tc + s1.vel(1:3) * t_opt;
pt_cls.mink = minkObj.GetMinkSumFromGradient(m_opt) +...
    s1.tc + s1.vel(1:3) * t_opt;

[flag, dist, pt_cls, condition] = get_results(pt_cls,...
    s2.tc + s2.vel(1:3) * t_opt, R1_opt*m_opt);
end

%% Objective least squares: distance from point to Minkowski sums boundary
function F = func_lsq(var, minkObj)
% Extract variables
psi = var(1:2);
t = var(3);

% Update orientation
minkObj.M1 = quat2rotm(minkObj.s1.q) *...
    expm(skew(minkObj.s1.vel(4:6)) * t);
minkObj.M2 = quat2rotm(minkObj.s2.q) *...
    expm(skew(minkObj.s2.vel(4:6)) * t);

% Minkowski sums with parameter \psi, t
m1 = minkObj.s1.GetGradientsFromSpherical(psi);
tc1 = minkObj.s1.tc + minkObj.s1.vel(1:3) * t;
mink = minkObj.GetMinkSumFromGradient(m1) + tc1;

% Cost function
p0 = minkObj.s2.tc + minkObj.s2.vel(1:3) * t;
F = p0 - mink;
end

%% Objective for fmincon: distance from point to Minkowski sums boundary
function F = func_con(var, minkObj)
% Extract variables
m1 = var(1:3);
t = var(4);

% Update orientation
minkObj.M1 = quat2rotm(minkObj.s1.q) *...
    expm(skew(minkObj.s1.vel(4:6)) * t);
minkObj.M2 = quat2rotm(minkObj.s2.q) *...
    expm(skew(minkObj.s2.vel(4:6)) * t);

% Minkowski sums with parameter m, t
tc1 = minkObj.s1.tc + minkObj.s1.vel(1:3) * t;
mink = minkObj.GetMinkSumFromGradient(m1) + tc1;

p0 = minkObj.s2.tc + minkObj.s2.vel(1:3) * t;

% Cost function
F = sum((p0 - mink).^2);
end

% Nonlinear constraint for gradient
function [c, ceq] = nlcon(var, s1)
c = [];

% Extract gradient
m1 = var(1:3);

x1 = s1.GetPointsFromGradient(m1);
ceq = s1.GetImplicitFunction(x1);
end