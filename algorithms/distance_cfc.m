function [flag, dist, pt_cls, condition] = distance_cfc(s1, s2, opt)
% distance_cfc computes the separation distance between two bodies, using 
% the closed-form contact space (CFC) and nonlinear optimization. 
% The surfaces are parameterized by gradient of s1.
%
%  Inputs:
%    s1, s2       Geometric class 
%                   'SuperQuadrics', 'Ellipsoid', 'PolyEllipsoid'
%    opt          Option of optimization algorithms
%                   'fixed-point' (default), 'constrained', 'least-squares'
%
%  Outputs:
%    flag         Status of contact
%                   1 -- in collision
%                   0 -- separated
%    dist         Separation distance/penetration depth (negative value)
%    pt_cls       A structure of closest points information
%                   s1   -- point on s1
%                   s2   -- point s2
%                   mink -- point on Minkowski sum boundary
%    condition    Necessary condition for result correctness, the line
%                  connecting two bodies is colinear with the normal vector
%                  at the computed point
%
%  Author:
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

% Option for choosing algorithm
if nargin == 2 || ( ~strcmp(opt, "fixed-point") &&...
        ~strcmp(opt, "constrained") && ~strcmp(opt, "least-squares") )
    opt = 'fixed-point';
end

% Initial condition: s2 center as viewed from s1 frame
R1 = quat2rotm(s1.q);
R2 = quat2rotm(s2.q);
s2_tc_in_s1 = R1' * (s2.tc-s1.tc);
psi0 = [atan2( s2_tc_in_s1(3), norm(s2_tc_in_s1(1:2)) ),...
            atan2( s2_tc_in_s1(2), s2_tc_in_s1(1) )];
m0 = s1.GetGradientsFromSpherical(psi0);

% Initialize Minkowski sums object
minkObj = MinkSumClosedForm(s1, s2, R1, R2);

% Optimization to solve
switch opt
    case 'fixed-point'
        m_opt = fixed_point(m0, minkObj);
        
    case 'constrained'    
        % min_{m_1} |p0 - x_{1+2}(m_1)|,
        % s.t.      \Phi( \Nabla \Phi^{-1}(m_1) ) - 1 = 0
        option = optimoptions('fmincon', 'Algorithm', 'interior-point',...
            'display', 'none');
        m_opt = fmincon(@(m) func_con(m, minkObj), m0,...
            [], [], [], [], [], [], @(m) nlcon(m, s1), option);
        
    case 'least-squares'
        % Optimization to solve min_{psi} |p0 - x_{1+2}(psi)|
        option = optimoptions('lsqnonlin',...
            'Algorithm', 'levenberg-marquardt',...
            'display', 'none',...
            'FunctionTolerance', 1e-8,...
            'OptimalityTolerance', 1e-8);
        
        psi_opt = lsqnonlin(@(psi) func_lsq(psi, minkObj), psi0(1,:),...
            [], [], option);
        
        % Solution gradient in local frame
        m_opt = s1.GetGradientsFromSpherical(psi_opt);
end

% Distance and closest points
% Optimal point on S1 in local frame
x_opt = s1.GetPointsFromGradient(m_opt);

% Witness points in global frame
pt_cls.s1 = R1 * x_opt + s1.tc;
pt_cls.mink = minkObj.GetMinkSumFromGradient(m_opt) + s1.tc;

% Results
[flag, dist, pt_cls, condition] = get_results(pt_cls, s2.tc, R1*m_opt);
end

%% Fixed-point iteration algorithm
function [m_new, flag] = fixed_point(m_init, minkObj)
tol = 1e-10;
p0 = minkObj.s2.tc;
R1 = quat2rotm(minkObj.s1.q);
flag = 1;

m01 = m_init;

mink = minkObj.GetMinkSumFromGradient(m01) + minkObj.s1.tc;
m02 = minkObj.s1.GetGradientsFromDirection( ...
    m01 + R1'*(p0-mink)/norm(p0-mink) * norm(m01) );

for i = 1:100
    % calculate the next two guesses for the fixed point.
    m_new = fixed_point_iteration(m01, m02, minkObj);
    
    % Test convergence
    if norm(m_new-m02)<tol
        flag = 0;
        return;
    end
    
    % Update gradients
    m01 = m02;
    m02 = m_new;
end
end

% Fixed point iteration step
function m1 = fixed_point_iteration(m01, m02, minkObj)
R1 = quat2rotm(minkObj.s1.q);

mink1 = R1'*minkObj.GetMinkSumFromGradient(m01);
mink2 = R1'*minkObj.GetMinkSumFromGradient(m02);

n01 = m01/norm(m01);
n02 = m02/norm(m02);
r1 = -(n02 - (n02'*n01)*n01)' * (mink2-mink1) /...
    (cross(n02, n01)' * cross(n02, n01)) * n02;

m1 = minkObj.s1.GetGradientsFromDirection(...
    R1'*(minkObj.s2.tc - minkObj.s1.tc) - (mink2 + r1) );
end

%% Constrained optimization cost and constraint
% Distance cost: distance from point to Minkowski sums boundary
function F = func_con(m, minkObj)
% Minkowski sums with parameter x
p0 = minkObj.s2.tc;
% m = minkObj.s1.GetGradientsFromCartesian(x);
mink = minkObj.GetMinkSumFromGradient(m) + minkObj.s1.tc;

% Cost function
F = sum((p0 - mink).^2);
end

% Nonlinear constraint for gradient vector
function [c,ceq] = nlcon(m, s1)
x = s1.GetPointsFromGradient(m);
ceq = s1.GetImplicitFunction(x);
c = [];
end

%% Least squares optimization cost
function F = func_lsq(psi, minkObj)
% Minkowski sums with parameter \psi
p0 = minkObj.s2.tc;
m = minkObj.s1.GetGradientsFromSpherical(psi);

mink = minkObj.GetMinkSumFromGradient(m) + minkObj.s1.tc;

% Cost function
F = p0 - mink;
end