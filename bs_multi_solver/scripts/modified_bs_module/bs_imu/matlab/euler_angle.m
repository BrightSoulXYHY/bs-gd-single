syms sin_a cos_a;
syms sin_b cos_b;
syms sin_c cos_c;

Rz = [
     cos_c  sin_c   0;
    -sin_c  cos_c   0;
     0      0       1;
];
Ry = [
    cos_b  0  -sin_b;
    0      1   0;
    sin_b  0   cos_b;
];
Rx = [
    1    0      0;
    0    cos_a  sin_a;
    0   -sin_a  cos_a;

];

Rx*Ry*Rz
% [                    cos_b*cos_c,                     cos_b*sin_c,      -sin_b]
% [cos_c*sin_a*sin_b - cos_a*sin_c, cos_a*cos_c + sin_a*sin_b*sin_c, cos_b*sin_a]
% [sin_a*sin_c + cos_a*cos_c*sin_b, cos_a*sin_b*sin_c - cos_c*sin_a, cos_a*cos_b]
