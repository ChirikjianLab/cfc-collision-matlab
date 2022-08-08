function [cos_t, sin_t] = trig_subs(t)
cos_t = (1-t.^2) ./ (1+t.^2);
sin_t = (2.*t) ./ (1+t.^2);