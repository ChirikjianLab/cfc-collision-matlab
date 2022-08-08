function [pt_cls, dist, flag, condition] = distance_implicit(s1, s2)
% distance_implicit computes the minimum separation distance between
% two bodies. Implicit function of the surfaces and convex optimizations 
% via interior-point method are used.
%
%  Inputs:
%    s1, s2       Geometric class 
%                   'SuperQuadrics', 'Ellipsoid', 'PolyEllipsoid'
%
%  Outputs:
%    flag         Status of contact
%                   1 -- in collision
%                   0 -- separated
%    dist         Separation distance
%    pt_cls       A structure of closest points information
%                   s1   -- point on s1
%                   s2   -- point on s2
%    condition    Necessary condition for result correctness, the line
%                   connecting two SQs is colinear with the normal vector
%                   at the computed point
%
%  Author:
%    Sipu Ruan, ruansp@nus.edu.eg, 2021

% Optimization
option = optimoptions('fmincon', 'Algorithm', 'interior-point',...
    'display', 'none');
    S = fmincon(@(x) func(x), zeros(6,1), [], [], [],...
        [], [], [], @(x) nlcon(x, s1, s2), option);

% Distance and closest points
pt_cls.s1 = S(1:3);
pt_cls.s2 = S(4:6);
dist = norm(pt_cls.s2 - pt_cls.s1);

% Collision status
if dist > 1e-3
    flag = 0;
else
    flag = 1;
    dist = -dist;
end

% Verification
R1 = quat2rotm(s1.q);
m_s1 = R1 * s1.GetGradientsFromCartesian( R1'*(pt_cls.s1-s1.tc) );
t_s1s2 = pt_cls.s2 - pt_cls.s1;
condition = norm(cross(m_s1, t_s1s2));
end

%% Objective function
function F = func(x)
F = norm(x(1:3) - x(4:6))^2;
end

function [c, ceq] = nlcon(x, s1, s2)
x(1:3) = quat2rotm(s1.q)' * (x(1:3)-s1.tc);
x(4:6) = quat2rotm(s2.q)' * (x(4:6)-s2.tc);

ceq = [];
c = [s1.GetImplicitFunction(x(1:3));
    s2.GetImplicitFunction(x(4:6))];
end