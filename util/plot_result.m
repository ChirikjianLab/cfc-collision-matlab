function plot_result(s1, s2, pt_cls, ...
    dist, is_collision, condition, ellapse_time)
% plot_result Plot collision detection results
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021

s1.PlotShape('y', 0.7);
s2.PlotShape('g', 0.7);

plot3(pt_cls.s1(1), pt_cls.s1(2), pt_cls.s1(3), '*r', 'LineWidth', 2)
plot3(pt_cls.s2(1), pt_cls.s2(2), pt_cls.s2(3), '*r', 'LineWidth', 2)
plot3([pt_cls.s1(1), pt_cls.s2(1)], [pt_cls.s1(2), pt_cls.s2(2)],...
    [pt_cls.s1(3), pt_cls.s2(3)],...
    'r', 'LineWidth',1.5)

try
    plot3(s2.tc(1), s2.tc(2), s2.tc(3), 'ro', 'LineWidth', 2)
    plot3(pt_cls.mink(1), pt_cls.mink(2), pt_cls.mink(3), 'r+', 'LineWidth', 2)
    plot3([pt_cls.mink(1) s2.tc(1)],...
        [pt_cls.mink(2) s2.tc(2)],...
        [pt_cls.mink(3) s2.tc(3)],...
        'r--', 'LineWidth', 1.5)
catch
end

title({['Collision? : ', num2str(is_collision)],...
    ['Distance: ', num2str(dist)],...
    ['Necessary condition: ', num2str(condition)],...
    ['Time: ', num2str(ellapse_time), ' s']})

lightangle(gca,45,30);
lighting gouraud;