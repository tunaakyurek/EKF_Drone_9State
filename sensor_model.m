function sensors = sensor_model(x, params, t)
% sensor_model - Simulate IMU, GPS, barometer, and magnetometer measurements
%   x: true state [pos; vel; att]
%   params: parameters struct
%   t: current time (s)
%   sensors: struct with fields accel, gyro, gps, baro, mag

persistent accel_bias gyro_bias
if isempty(accel_bias)
    accel_bias = params.IMU.accel_bias_instab * randn(3,1);
    gyro_bias  = params.IMU.gyro_bias_instab  * randn(3,1);
end

% True values
pos = x(1:3);
vel = x(4:6);
att = x(7:9);
g = params.g;

% IMU (body frame)
R = eul2rotm(att');
acc_true = R' * (vel - g); % neglecting drag, etc.
gyro_true = [0; 0; 0]; % For simplicity, no rotation rate input here

accel_noise = params.IMU.accel_noise_density / sqrt(params.Ts.IMU) * randn(3,1);
gyro_noise  = params.IMU.gyro_noise_density  / sqrt(params.Ts.IMU) * randn(3,1);

sensors.accel = acc_true + accel_bias + accel_noise;
sensors.gyro  = gyro_true + gyro_bias  + gyro_noise;

% GPS (position)
gps_noise = [params.GPS.sigma_xy*randn(2,1); params.GPS.sigma_z*randn(1,1)];
sensors.gps = pos + gps_noise;

% Barometer (altitude = -z in NED)
baro_noise = params.Baro.sigma_z * randn;
sensors.baro = -pos(3) + baro_noise;

% Magnetometer (yaw/heading)
mag_noise = params.Mag.sigma_rad * randn;
sensors.mag = att(3) + mag_noise;

end 