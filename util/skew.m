function W = skew(w)
% skew Construct skew-symmetric matrix from a 3D vector

W = [0, -w(3), w(2);
    w(3), 0, -w(1);
    -w(2), w(1), 0];