syms w x y z;

% 都是欧拉角的半角
syms sin_x cos_x;
syms sin_y cos_y;
syms sin_z cos_z;

vec = [x;y;z];
mat_vec = [
    [  w  z  -y ];
    [ -z  w   x ];
    [  y -x   w ];
];

vec*vec.' + mat_vec*mat_vec


mat_qx = [
    [cos_x -sin_x 0 0];
    [sin_x cos_x 0 0];
    [0 0 cos_x sin_x];
    [0 0 -sin_x cos_x];
];
mat_qy = [
    [cos_y 0 -sin_y 0];
    [0 cos_y 0 -sin_y];
    [sin_y 0 cos_y 0];
    [0 sin_y 0 cos_y];
];
qz = [cos_z;0;0;sin_z];

mat_qx*mat_qy*qz