function plot_result_ccd(s1, s2, g1_opt, g2_opt,...
    g1_init, g2_init, pt_cls, dist, flag, cond, ellapse_time)
% plot_result_ccd Plot continuous collision detection results
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

% Initial and trajectory
s1.PlotShape('m', 0.2);
s2.PlotShape('m', 0.2);

idx = 1;
for t = 0:1/49:1
    g1_t = update_pose(g1_init, s1.vel, t, 'PCG');
    tc1(:,idx) = g1_t(1:3,4);
    g2_t = update_pose(g2_init, s2.vel, t, 'PCG');
    tc2(:,idx) = g2_t(1:3,4);
    
    idx = idx+1;
end
plot3(tc1(1,:), tc1(2,:), tc1(3,:), 'k', 'LineWidth', 2)
plot3(tc2(1,:), tc2(2,:), tc2(3,:), 'k', 'LineWidth', 2)

% Optimal solution
s1.q = rotm2quat(g1_opt(1:3,1:3));
s1.tc = g1_opt(1:3,4);
s2.q = rotm2quat(g2_opt(1:3,1:3));
s2.tc = g2_opt(1:3,4);

% Contact space at optimum
m1 = s1.GetGradientsCanonical();
mink = MinkSumClosedForm(s1, s2, quat2rotm(s1.q), quat2rotm(s2.q));
x_mink = mink.GetMinkSumFromGradient(m1) + s1.tc;

N = [sqrt(size(m1,2)), sqrt(size(m1,2))];
X = reshape(x_mink(1,:), N(1), N(2));
Y = reshape(x_mink(2,:), N(1), N(2));
Z = reshape(x_mink(3,:), N(1), N(2));
surf(X, Y, Z,...
 'EdgeColor', 'k', 'EdgeAlpha', 0.4,...
 'FaceAlpha', 0.1, 'FaceColor', 'b');

plot_result(s1, s2, pt_cls, dist, flag, cond, ellapse_time)

% Goal
s1.q = rotm2quat(g1_t(1:3,1:3));
s1.tc = g1_t(1:3,4);
s2.q = rotm2quat(g2_t(1:3,1:3));
s2.tc = g2_t(1:3,4);

s1.PlotShape('c', 0.2);
s2.PlotShape('c', 0.2);