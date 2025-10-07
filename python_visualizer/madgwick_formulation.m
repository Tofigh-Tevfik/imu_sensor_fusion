clc; clear;

%% params
syms qw qx qy qz real;
syms ax ay az real;

% objective function
f = [2*(qx*qz - qw*qy) - ax;
     2*(qw*qx + qy*qz) - ay;
     2*(0.5 - qx^2 - qy^2) - az];

q = [qw qx qy qz];

J = jacobian(f, q);