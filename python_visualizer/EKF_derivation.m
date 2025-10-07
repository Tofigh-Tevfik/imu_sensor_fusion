clc; clear;

%% params
syms qw qx qy qz real; % quaternions [4x1]
syms bx by bz real; % gyro bias [3x1]
syms Q1 Q2 Q3 Q4 Q5 Q6 Q7 real; % gyro and bias noise variance [3x3]
syms R1 R2 R3 real; % accel noise variance [3x3]
syms ax ay az real; % accel data [3x1]
syms gx gy gz real; % gyro data [3x1]
syms g dt real; % gravity constant and time sample
syms P1 P2 P3 P4 P5 P6 P7 real; % state covariance matrix

%% constructing vectors
q = [qw; qx; qy; qz];
b = [bx; by; bz];
Q = diag([Q1, Q2, Q3, Q4, Q5, Q6, Q7]);
R = diag([R1, R2, R3]);
Pk = diag([P1, P2, P3, P4, P5, P6, P7]);
% state vector
% consists of quaternion and bias
% [7x1]
xk = [q; b];
% measurement vector
zk = [ax; ay; az];
w = [gx; gy; gz];
omega = w - b;
omega_x = omega(1); omega_y = omega(2); omega_z = omega(3);
% skew symetric matrix
Omega = [  0,       -omega_x, -omega_y, -omega_z;
           omega_x,     0,     omega_z, -omega_y;
           omega_y, -omega_z,     0,     omega_x;
           omega_z,  omega_y, -omega_x,     0 ];
%% continous time dynamics
qdot = 0.5 * Omega * q;
bdot = zeros(3, 1); % modeled as random walk.
f = [qdot; bdot];
%% process predict
% propogating state mean
xk_1 = xk + f * dt;
% normalize after
% jacobian of the state
% taken with respect to the state [7x7]
F = simplify(jacobian(xk_1, xk));
Pk_1 = F*Pk*F' + Q;
%% measurement update
Rq = [1 - 2*qy^2 - 2*qz^2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw;
       2*qx*qy + 2*qz*qw,     1 - 2*qx^2 - 2*qz^2,     2*qy*qz - 2*qx*qw;
       2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx^2 - 2*qy^2];

h = Rq' * [0; 0; g];
% measurement residual
yk = zk - h;
% jacobian of h with respect to state vector
H = simplify(jacobian(h, xk)); % [3x7]
% innovation covariance
Sk = H*Pk_1*H' + R;
% near optimal kalman gain
K = Pk_1 * H' / Sk;
% % updated state estimate
% x_k = xk_1 + K * yk;
% % updated covariance estimate
% P_k = (eye(7) - K * H) * Pk_1;