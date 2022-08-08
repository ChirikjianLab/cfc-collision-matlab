function plot_result_object(s1, s2, status, pcls)

if status == 0
    plot3(s2.tc(1), s2.tc(2), s2.tc(3), 'bo');
    s2.PlotShape('b', 0.6);
elseif status == 1
    plot3(s2.tc(1), s2.tc(2), s2.tc(3), 'r*');
    s2.PlotShape('r', 0.6);
end

try
    p_mink_sph = pcls.Mink;
catch
end
p_s1_sph = pcls.s1;
p_s2_sph = pcls.s2;
%     plot3([p_mink_sph(1) s2.tc(1)], [p_mink_sph(2) s2.tc(2)],...
%         [p_mink_sph(3) s2.tc(3)], 'm', 'LineWidth', 2)
plot3([p_s1_sph(1) p_s2_sph(1)], [p_s1_sph(2) p_s2_sph(2)],...
    [p_s1_sph(3) p_s2_sph(3)], 'g', 'LineWidth', 2)