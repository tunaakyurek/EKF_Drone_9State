function [q_dot] = quaternion_utils(omega_body, q)
% quaternion_utils - Quaternion utilities for attitude representation
%   omega_body: angular velocity in body frame [rad/s]
%   q: quaternion [qw; qx; qy; qz]

% Quaternion derivative from angular velocity
wx = omega_body(1); wy = omega_body(2); wz = omega_body(3);
qw = q(1); qx = q(2); qy = q(3); qz = q(4);

q_dot = 0.5 * [
    -qx*wx - qy*wy - qz*wz;
     qw*wx - qz*wy + qy*wz;
     qz*wx + qw*wy - qx*wz;
    -qy*wx + qx*wy + qw*wz
];
end

function R = quat_to_rotmat(q)
% Convert quaternion to rotation matrix
% q: [qw; qx; qy; qz]

qw = q(1); qx = q(2); qy = q(3); qz = q(4);

R = [
    1-2*qy^2-2*qz^2, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
    2*(qx*qy+qw*qz), 1-2*qx^2-2*qz^2, 2*(qy*qz-qw*qx);
    2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*qx^2-2*qy^2
];
end

function q = euler_to_quat(phi, theta, psi)
% Convert Euler angles to quaternion
% phi: roll, theta: pitch, psi: yaw

cph = cos(phi/2); sph = sin(phi/2);
cth = cos(theta/2); sth = sin(theta/2);
cps = cos(psi/2); sps = sin(psi/2);

q = [
    cph*cth*cps + sph*sth*sps;
    sph*cth*cps - cph*sth*sps;
    cph*sth*cps + sph*cth*sps;
    cph*cth*sps - sph*sth*cps
];
end

function [phi, theta, psi] = quat_to_euler(q)
% Convert quaternion to Euler angles
% Returns: [roll, pitch, yaw] in radians

qw = q(1); qx = q(2); qy = q(3); qz = q(4);

% Roll (x-axis rotation)
phi = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));

% Pitch (y-axis rotation)
theta = asin(2*(qw*qy - qz*qx));

% Yaw (z-axis rotation)
psi = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
end

function q_norm = normalize_quat(q)
% Normalize quaternion to unit length
q_norm = q / norm(q);
end 