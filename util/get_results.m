function [is_collision, dist, pt_cls, condition] = get_results(pt_cls, s2_tc, normal)
% get_results Get collision detection results from solved point on contact
% space
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

% Witness point on s2
pt_cls.s2 = pt_cls.s1 + (s2_tc - pt_cls.mink);

% Distance
dist = norm(pt_cls.s1 - pt_cls.s2);

% Collision status
is_collision = normal' * (pt_cls.s2 - pt_cls.s1) < 0;
if is_collision; dist = -dist; end

% Verification: line from closest points on SQ and E is coliearn with
% normal
t_s1s2 = pt_cls.s2 - pt_cls.s1;
condition = norm(cross(normal, t_s1s2));