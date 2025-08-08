function sensors = sensor_model(x, params, t)
% sensor_model - Simulate IMU, GPS, barometer, and magnetometer measurements
%   x: true state [pos; vel; att]
%   params: parameters struct
%   t: current time (s)
%   sensors: struct with fields accel, gyro, gps, baro, mag

persistent accel_bias gyro_bias prev_t prev_vel prev_att
if isempty(accel_bias)
    accel_bias = params.IMU.accel_bias_instab * randn(3,1);
    gyro_bias  = params.IMU.gyro_bias_instab  * randn(3,1);
    prev_t     = t;
    prev_vel   = x(4:6);
    prev_att   = x(7:9);
end

% True values
pos = x(1:3);
vel = x(4:6);
att = x(7:9);
 g = params.g;

% Time step for finite differences: align to IMU period for stability
dt_fd = params.Ts.IMU;

% IMU (body frame)
R = eul2rotm(att');

% True linear acceleration in NED via finite difference
a_true_ned = (vel - prev_vel) / dt_fd;

% Specific force in body frame: f_b = R' * (a - g)
f_body = R' * (a_true_ned - g);

% True Euler angle rates via finite difference
att_dot = (att - prev_att) / dt_fd;

% Map Euler angle rates to body rates: att_dot = E * omega_body
phi = att(1); theta = att(2);
E = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

if rcond(E) > 1e-6
    omega_body = E \ att_dot;
else
    omega_body = zeros(3,1);
end

% IMU noise per sample (discrete-time) using noise densities
accel_noise = params.IMU.accel_noise_density / sqrt(params.Ts.IMU) * randn(3,1);
gyro_noise  = params.IMU.gyro_noise_density  / sqrt(params.Ts.IMU) * randn(3,1);

% Compose IMU measurements
sensors.accel = f_body + accel_bias + accel_noise;
sensors.gyro  = omega_body + gyro_bias  + gyro_noise;

% GPS (position)
gps_noise = [params.GPS.sigma_xy*randn(2,1); params.GPS.sigma_z*randn(1,1)];
sensors.gps = pos + gps_noise;

% Barometer (altitude = -z in NED)
baro_noise = params.Baro.sigma_z * randn;
sensors.baro = -pos(3) + baro_noise;

% Magnetometer (yaw/heading)
mag_noise = params.Mag.sigma_rad * randn;
sensors.mag = att(3) + mag_noise;

% Update persistence
prev_t = t;
prev_vel = vel;
prev_att = att;

end 