function g_update = update_pose(g_init, vel, t, opt)
% update_pose Update object intermediate pose for a motion
%
%  Inputs
%    g_init   Initial pose, 4x4 homogeneous transformation matrix
%    vel      Velocity of the movement
%    t        Intermediate time step to be computed
%    opt      Option of interpolation method, i.e., 'SE', 'PCG'
%
%  Output
%    g_update Updated intermediate pose
%
%  Author
%    Sipu Ruan, ruansp@nus.edu.sg, 2021
%
%  See also
%    expm, skew

g_update = eye(size(g_init,1));

if nargin == 3 || strcmp(opt, 'PCG')
    g_update = [g_init(1:3,1:3) * expm(skew(vel(4:6))*t),...
        g_init(1:3,4) + vel(1:3) * t; 0,0,0,1];
    
elseif strcmp(opt, 'SE')
    g_update = g_init * [expm(skew(vel(4:6))*t), vel(1:3) * t; 0,0,0,1];
end