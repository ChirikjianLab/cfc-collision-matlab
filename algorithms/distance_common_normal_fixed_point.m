function [flag, dist, pt_cls, condition] = distance_common_normal_fixed_point(s1, s2)
% distance_common_normal_fixed_point computes the separation distance 
% between two bodies, using normal parameterization and fixed-point 
% iteration approach.
%
%  Inputs:
%    s1, s2       Geometric class 
%                   'SuperQuadrics', 'Ellipsoid', 'PolyEllipsoid'
%
%  Outputs:
%    flag         Status of contact
%                   1 -- in collision
%                   0 -- separated
%    dist         Separation distance (+) / penetration depth (-)
%    pt_cls       A structure of witness points information
%                   s1   -- point on ellipsoid 1
%                   s2   -- point on ellipsoid 2
%    condition    Necessary condition for result correctness, the line
%                  connecting two bodies is colinear with the normal vector
%                  at the computed point
%
%  Author:
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

% Parameters
R1 = quat2rotm(s1.q);
R2 = quat2rotm(s2.q);

A1 = R1 * diag(s1.a) * R1';
A2 = R2 * diag(s2.a) * R2';

% Initial guess for normal vector
n0 = s2.tc-s1.tc/norm(s2.tc-s1.tc);

% Optimization
n_opt = fixed_point(n0, A1, A2, s1.tc, s2.tc);

% Distance and closest points
% points in global frame
pt_cls.s1 = A1^2*n_opt / norm(A1*n_opt) + s1.tc;
pt_cls.s2 = A2^2*(-n_opt) / norm(A2*(-n_opt)) + s2.tc;
dist = norm(pt_cls.s2 - pt_cls.s1);

% Collision status
if dist > 1e-3
    flag = 0;
else
    flag = 1;
    dist = -dist;
end

% Verification
t_s1s2 = pt_cls.s2 - pt_cls.s1;
condition = norm(cross(n_opt, t_s1s2));
end

%% Objective, constraint, jacobian
% Fixed-point method
function [n_new, flag] = fixed_point(n_init, A1, A2, m, s)
tol = 1e-10;

n01 = n_init;
r_pq = (A2^2*n01 / norm(A2*n01) + s) - (A1^2*n01 / norm(A1*n01) + m);
n02 = n01 + r_pq/norm(r_pq);
n02 = n02/norm(n02);

for i = 1:100
    % calculate the next two guesses for the fixed point.
    n_new = fixed_point_iteration(n01, n02, A1, A2, m, s);
    
    if norm(n_new-n02)<tol
        flag = 0;
        return;
    end
    
    n01 = n02;
    n02 = n_new;
end

% Test convergence
if abs(n_new-n02)>tol
    flag = 1;
end

end

% Fixed point iteration
function n1 = fixed_point_iteration(n01, n02, A1, A2, m, s)
r_mp_01 = A1^2*n01 / norm(A1*n01);
r_mp_02 = A1^2*n02 / norm(A1*n02);
r_sq_01 = A2^2*(-n01) / norm(A2*(-n01));
r_sq_02 = A2^2*(-n02) / norm(A2*(-n02));

r_p1 = -(n02 - (n02'*n01)*n01)' * (r_mp_02-r_mp_01) /...
    (cross(n02, n01)' * cross(n02, n01)) * n02;
r_q1 = -(n02 - (n02'*n01)*n01)' * (r_sq_02-r_sq_01) /...
    (cross(n02, n01)' * cross(n02, n01)) * n02;

n1 = (s - m) - (r_mp_02 + r_p1) + (r_sq_02 + r_q1);
n1 = n1/norm(n1);
end