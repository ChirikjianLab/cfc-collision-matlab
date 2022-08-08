function s = plot_surf(s, alpha)
s.EdgeColor = 'k';
s.FaceAlpha = 0.7;
s.FaceColor = 'w';

if nargin > 1
    s.FaceAlpha = alpha;
end
end