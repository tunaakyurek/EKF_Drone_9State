function [x_dot] = drone_dynamics_imu(t, x, imu, params)
% drone_dynamics_imu - 6DOF mechanization for EKF prediction using only IMU data
%   x = [pos; vel; att] where:
%       pos = [x; y; z] (NED, m)
%       vel = [vx; vy; vz] (NED, m/s)
%       att = [roll; pitch; yaw] (rad)
%   imu = [accel_meas; gyro_meas] (accel in m/s^2, gyro in rad/s, both in body frame)
%   params = struct of physical parameters

% Unpack state
pos = x(1:3);
vel = x(4:6);
att = x(7:9); % [phi; theta; psi]

% Unpack IMU
accel_meas = imu(1:3); % body frame
gyro_meas = imu(4:6);  % body frame

g = params.g;

% 1. Update attitude using gyro (Euler integration)
phi = att(1); theta = att(2); psi = att(3);
E = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0, cos(phi),           -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
att_dot = E * gyro_meas;

% 2. Update velocity using accel (rotate to NED, add gravity)
R = rotation_matrix(phi, theta, psi);
vel_dot = R * accel_meas + g;

% 3. Position derivative
pos_dot = vel;

% 4. State derivative
x_dot = zeros(9,1);
x_dot(1:3) = pos_dot;
x_dot(4:6) = vel_dot;
x_dot(7:9) = att_dot;
end 