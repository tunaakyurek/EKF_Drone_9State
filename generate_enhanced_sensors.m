function sensors = generate_enhanced_sensors(x_true, params, t)
% generate_enhanced_sensors - Comprehensive sensor model generation
% Based on MATLAB Simulink Autonomous Flight Design Tutorial methodology
%
% Generates realistic sensor measurements including:
% - IMU (accelerometer + gyroscope) at 200 Hz
% - GPS at 20 Hz with realistic noise and dropouts
% - Barometer at 20 Hz with atmospheric pressure model
% - Magnetometer at 20 Hz with declination and interference
% - Sensor failures and health monitoring
%
% Inputs:
%   x_true - True state [pos(3); vel(3); att(3)]
%   params - System parameters with sensor characteristics
%   t - Current time (for time-varying effects)
%
% Outputs:
%   sensors - Structure containing all sensor measurements

% Initialize sensor structure
sensors = struct();

% Extract true state components
pos_ned = x_true(1:3);    % Position in NED frame [m]
vel_ned = x_true(4:6);    % Velocity in NED frame [m/s]
att_ned = x_true(7:9);    % Attitude [roll; pitch; yaw] [rad]

% Rotation matrix from NED to body frame
R_ned_to_body = rotation_matrix_ned_to_body(att_ned(1), att_ned(2), att_ned(3));

%% IMU SENSORS (High Frequency - 200 Hz)
% Based on Pixhawk 6C specifications from tutorial

% ACCELEROMETER
% True acceleration in NED frame
accel_ned = calculate_true_acceleration(x_true, params, t);

% Transform to body frame
accel_body_true = R_ned_to_body * (accel_ned - params.g);

% Add realistic accelerometer noise and bias
accel_noise = params.IMU.accel_noise_density * randn(3,1) / sqrt(params.Ts.IMU);
accel_bias = params.IMU.accel_bias_instab * (0.5 - rand(3,1)); % Slowly varying bias

sensors.accel = accel_body_true + accel_noise + accel_bias;

% GYROSCOPE
% True angular rates (simplified - assume hovering for this basic model)
% In full implementation, this would come from angular acceleration integration
omega_body_true = calculate_true_angular_rates(x_true, t);

% Add realistic gyroscope noise and bias
gyro_noise = params.IMU.gyro_noise_density * randn(3,1) / sqrt(params.Ts.IMU);
gyro_bias = params.IMU.gyro_bias_instab * (0.5 - rand(3,1)); % Slowly varying bias

sensors.gyro = omega_body_true + gyro_noise + gyro_bias;

%% GPS SENSOR (20 Hz with realistic characteristics)
% Based on u-blox M10 specifications

% GPS availability model (realistic dropouts)
gps_available = is_gps_available(pos_ned, t);

if gps_available
    % Horizontal position noise (higher accuracy)
    gps_noise_xy = params.GPS.sigma_xy * randn(2,1);
    
    % Vertical position noise (lower accuracy)
    gps_noise_z = params.GPS.sigma_z * randn(1,1);
    
    % Combine noise
    gps_noise = [gps_noise_xy; gps_noise_z];
    
    % Add multipath and atmospheric errors (simplified)
    multipath_error = calculate_multipath_error(pos_ned, t);
    atmospheric_error = calculate_atmospheric_gps_error(pos_ned);
    
    sensors.gps = pos_ned + gps_noise + multipath_error + atmospheric_error;
    
    % GPS health indicator
    sensors.gps_health = true;
    sensors.gps_hdop = 1.2 + 0.3 * rand(); % Horizontal Dilution of Precision
    sensors.gps_satellites = 8 + round(4 * rand()); % Number of satellites
else
    sensors.gps = [NaN; NaN; NaN];
    sensors.gps_health = false;
    sensors.gps_hdop = 99.9;
    sensors.gps_satellites = 0;
end

%% BAROMETER SENSOR (20 Hz)
% Based on atmospheric pressure model

% True altitude (negative z in NED)
altitude_msl = -pos_ned(3); % Convert from NED down to MSL up

% ISA atmospheric model for pressure
pressure_true = calculate_atmospheric_pressure(altitude_msl);

% Add barometer noise and bias
baro_noise = params.Baro.sigma_z * randn(1,1);
baro_bias = 0.1 * sin(t / 100); % Slow temperature-dependent bias

% Convert pressure back to altitude measurement
altitude_measured = pressure_to_altitude(pressure_true) + baro_noise + baro_bias;

sensors.baro = -altitude_measured; % Convert back to NED (down positive)
sensors.baro_pressure = pressure_true; % Raw pressure for diagnostics

%% MAGNETOMETER SENSOR (20 Hz)
% Based on Earth's magnetic field model

% Earth's magnetic field in NED (location-specific: Espoo, Finland)
mag_earth_ned = params.mag_NED; % [North; East; Down] in µT

% Transform to body frame
mag_body_true = R_ned_to_body * mag_earth_ned;

% Calculate expected heading from magnetometer
heading_true = atan2(mag_body_true(2), mag_body_true(1));

% Add magnetometer noise and interference
mag_noise = params.Mag.sigma_rad * randn(1,1);

% Add interference from motors (speed-dependent)
motor_interference = calculate_motor_interference(sensors.accel, t);

% Add hard and soft iron effects (simplified)
iron_distortion = calculate_iron_effects(mag_body_true);

sensors.mag = heading_true + mag_noise + motor_interference + iron_distortion;
sensors.mag = wrapToPi(sensors.mag); % Wrap to [-π, π]

% Raw magnetometer vector (for advanced algorithms)
sensors.mag_vector = mag_body_true + 0.5 * randn(3,1) + iron_distortion * [1; 1; 1];

%% SENSOR HEALTH MONITORING
sensors.sensor_health = struct();
sensors.sensor_health.imu = true; % IMU rarely fails completely
sensors.sensor_health.gps = sensors.gps_health;
sensors.sensor_health.baro = check_baro_health(sensors.baro);
sensors.sensor_health.mag = check_mag_health(sensors.mag_vector);

%% ADDITIONAL SENSORS (Optional - for advanced implementations)
% These can be added for more comprehensive sensor fusion

% OPTICAL FLOW (if available)
if isfield(params, 'optical_flow_enabled') && params.optical_flow_enabled
    sensors.optical_flow = generate_optical_flow(vel_ned, pos_ned(3), att_ned);
end

% LIDAR ALTIMETER (if available)
if isfield(params, 'lidar_enabled') && params.lidar_enabled
    sensors.lidar_altitude = generate_lidar_altitude(pos_ned(3));
end

% AIR SPEED SENSOR (if available)
if isfield(params, 'airspeed_enabled') && params.airspeed_enabled
    sensors.airspeed = generate_airspeed(vel_ned, params);
end

end

%% HELPER FUNCTIONS

function R = rotation_matrix_ned_to_body(roll, pitch, yaw)
% Rotation matrix from NED to body frame

Rx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
Ry = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
Rz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];

R = Rx * Ry * Rz;

end

function accel_ned = calculate_true_acceleration(x_true, params, t)
% Calculate true acceleration in NED frame
% This would typically come from the physics simulation

% Check if this is the initialization phase (t near zero)
if t < 0.1
    % During initialization, use only gravity with minimal noise
    accel_ned = [0; 0; 0]; % No acceleration during initialization
else
    % After initialization, add small accelerations for realistic sensor modeling
    % In full implementation, this would be the output of the dynamics model
    
    % Small random accelerations for realistic sensor testing (reduced magnitude)
    accel_ned = 0.2 * randn(3,1);
    
    % Add some systematic motion if needed (reduced magnitude)
    accel_ned = accel_ned + [0.05*sin(t/10); 0.05*cos(t/15); 0.02*sin(t/20)];
end

end

function omega_true = calculate_true_angular_rates(x_true, t)
% Calculate true angular rates in body frame
% Simplified model for sensor testing

% Check if this is the initialization phase (t near zero)
if t < 0.1
    % During initialization, use zero angular rates
    omega_true = [0; 0; 0]; % No rotation during initialization
else
    % After initialization, add small angular rates for realistic flight
    % Reduced magnitude for more stable behavior
    omega_true = [0.01*sin(t/8); 0.01*cos(t/12); 0.005*sin(t/15)]; % rad/s
    
    % Apply stricter limits
    max_rate = deg2rad(45); % Reduced from 100 deg/s to 45 deg/s
    omega_true = max(min(omega_true, max_rate), -max_rate);
end

end

function available = is_gps_available(pos_ned, t)
% Model GPS availability including realistic dropouts and interference

% During initialization, ensure GPS is always available
if t < 0.1
    available = true;
    return;
end

% IMPROVED: Much more reliable GPS for outdoor flight simulation
% Basic availability (99% of the time for outdoor conditions)
available = rand() > 0.01; % Increased from 98% to 99% reliability

% Altitude-based degradation (more realistic for outdoor flight)
altitude = -pos_ned(3);
if altitude < 1 % Very low altitude (ground effect)
    available = available && (rand() > 0.05); % 5% additional dropout (reduced from 10%)
elseif altitude < 5 % Low altitude
    available = available && (rand() > 0.02); % 2% additional dropout (reduced from 5%)
end

% Reduced periodic interference (much less aggressive)
if mod(t, 120) < 0.5 % 0.5 second every 120 seconds (much less frequent)
    available = available && (rand() > 0.1); % 10% additional dropout (reduced from 15%)
end

end

function multipath_error = calculate_multipath_error(pos_ned, t)
% Calculate GPS multipath error (simplified model)

% Multipath depends on environment and satellite geometry
% Simplified sinusoidal model with time-varying component
multipath_magnitude = 0.5; % meters

multipath_error = multipath_magnitude * [
    0.3 * sin(t / 25 + pos_ned(1) / 100);
    0.3 * cos(t / 30 + pos_ned(2) / 100);
    0.2 * sin(t / 35 + pos_ned(3) / 50)
];

end

function atm_error = calculate_atmospheric_gps_error(pos_ned)
% Calculate atmospheric GPS errors (ionosphere/troposphere)

% Simplified model - typically 1-5 meters
altitude = -pos_ned(3);
atm_error_magnitude = 1.0 + 2.0 * exp(-altitude / 10000); % Decreases with altitude

atm_error = atm_error_magnitude * [
    0.2 * randn();
    0.2 * randn();
    0.3 * randn()
];

end

function pressure = calculate_atmospheric_pressure(altitude_msl)
% ISA atmospheric model for pressure calculation

% Standard atmosphere at sea level
P0 = 101325; % Pa
T0 = 288.15; % K
L = 0.0065;  % K/m (temperature lapse rate)
g = 9.80665; % m/s²
M = 0.0289644; % kg/mol (molar mass of air)
R = 8.31432; % J/(mol·K) (gas constant)

% ISA pressure formula
if altitude_msl < 11000 % Troposphere
    T = T0 - L * altitude_msl;
    pressure = P0 * (T / T0)^(g * M / (R * L));
else % Stratosphere (simplified)
    pressure = P0 * 0.2233 * exp(-g * M * (altitude_msl - 11000) / (R * 216.65));
end

end

function altitude = pressure_to_altitude(pressure)
% Convert pressure to altitude using ISA model

P0 = 101325; % Pa
T0 = 288.15; % K
L = 0.0065;  % K/m
g = 9.80665; % m/s²
M = 0.0289644; % kg/mol
R = 8.31432; % J/(mol·K)

% Inverse ISA formula
altitude = (T0 / L) * (1 - (pressure / P0)^(R * L / (g * M)));

end

function interference = calculate_motor_interference(accel, t)
% Calculate magnetometer interference from motors

% Motor interference is proportional to motor speed (approximated by thrust)
thrust_proxy = norm(accel); % Rough approximation

% Interference is time-varying due to motor RPM variations
interference = 0.05 * thrust_proxy * sin(2 * pi * 50 * t); % 50 Hz motor noise

% Limit interference
interference = max(min(interference, deg2rad(10)), deg2rad(-10));

end

function distortion = calculate_iron_effects(mag_vector)
% Calculate hard and soft iron magnetic distortions

% Hard iron effects (constant bias)
hard_iron_bias = deg2rad([2; -1; 0.5]); % Degrees converted to radians

% Soft iron effects (scaling and rotation - simplified)
soft_iron_scale = [0.98; 1.02; 0.99]; % Scale factors

% Calculate heading distortion from combined effects
mag_magnitude = norm(mag_vector);
if mag_magnitude > 0
    heading_distortion = atan2(hard_iron_bias(2), hard_iron_bias(1)) * 0.1;
else
    heading_distortion = 0;
end

distortion = heading_distortion;

end

function health = check_baro_health(baro_measurement)
% Check barometer health based on measurement validity

% Simple health check - reject if measurement is unrealistic
health = isfinite(baro_measurement) && abs(baro_measurement) < 1000;

end

function health = check_mag_health(mag_vector)
% Check magnetometer health based on magnetic field strength

expected_magnitude = 50; % µT (typical Earth's magnetic field)
measured_magnitude = norm(mag_vector);

% Health check: magnetic field magnitude should be reasonable
health = (measured_magnitude > 20) && (measured_magnitude < 80);

end

function optical_flow = generate_optical_flow(vel_ned, altitude, att)
% Generate optical flow sensor measurements (pixels/second)

if altitude > 0.1 % Avoid division by zero
    % Optical flow is proportional to horizontal velocity and inversely to altitude
    flow_scale = 100; % pixels per (m/s)/m
    
    % Convert horizontal velocity to optical flow
    optical_flow = [
        flow_scale * vel_ned(1) / altitude + 2 * randn();
        flow_scale * vel_ned(2) / altitude + 2 * randn()
    ];
else
    optical_flow = [0; 0];
end

end

function lidar_alt = generate_lidar_altitude(altitude_ned)
% Generate LIDAR altimeter measurement

true_altitude = -altitude_ned; % Convert NED down to altitude up

% Add LIDAR noise (typically very accurate)
lidar_noise = 0.05 * randn(); % 5cm standard deviation

lidar_alt = true_altitude + lidar_noise;

% LIDAR health check (reasonable altitude)
if lidar_alt < 0 || lidar_alt > 100
    lidar_alt = NaN; % Invalid measurement
end

end

function airspeed = generate_airspeed(vel_ned, params)
% Generate airspeed sensor measurement

% Calculate airspeed magnitude
true_airspeed = norm(vel_ned);

% Add airspeed sensor noise
airspeed_noise = 0.5 * randn(); % 0.5 m/s standard deviation

airspeed = true_airspeed + airspeed_noise;

% Ensure non-negative
airspeed = max(airspeed, 0);

end

