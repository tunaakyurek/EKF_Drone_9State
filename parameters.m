%% parameters.m - Drone EKF Simulation Parameters
% Select drone profile: 'QAV250' or 'S500'
profile = 'QAV250'; % Change to 'S500' for the other drone

%% 1. General Simulation & Timing Parameters
params.solver_type = 'Fixed-step';
params.Ts.physics = 0.002;   % Physics sim rate (500 Hz)
params.Ts.IMU    = 0.005;    % IMU sample time (200 Hz)
params.Ts.GPS    = 0.05;      % GPS sample time (20 Hz)
params.Ts.Baro   = 0.05;     % Barometer sample time (20 Hz)
params.Ts.Mag    = 0.05;     % Magnetometer sample time (20 Hz)
params.sim_duration = 60;   % seconds

%% 2. Location-Specific Parameters (Espoo, Finland)
params.g = [0; 0; -9.819];   % Gravity (m/s^2)
params.mag_NED = [15.16; 2.65; 49.78]; % Magnetic field (uT)

%% 3. Drone-Specific Physical Parameters
switch profile
    case 'QAV250'
        params.arm_length = 0.125;      % meters
        params.prop_radius = 0.0635;    % meters
        params.CT = 1.2e-6;             % Thrust coefficient
        params.CQ = 2.1e-8;             % Torque coefficient
        params.Q_vel = 0.4;             % EKF process noise (velocity, increased for turns)
        params.Q_att = 0.02;            % EKF process noise (attitude, increased for turns)
        params.mass = 0.5;              % kg
        params.I = diag([0.0023, 0.0023, 0.004]); % kg*m^2
    case 'S500'
        params.arm_length = 0.25;       % meters
        params.prop_radius = 0.127;     % meters
        params.CT = 9.5e-6;             % Thrust coefficient
        params.CQ = 1.8e-7;             % Torque coefficient
        params.Q_vel = 0.4;             % EKF process noise (velocity, increased for turns)
        params.Q_att = 0.01;            % EKF process noise (attitude, increased for turns)
        params.mass = 1.2;              % kg
        params.I = diag([0.011, 0.011, 0.021]); % kg*m^2
    otherwise
        error('Unknown profile!');
end

%% 4. Sensor Noise & Error Parameters (Pixhawk 6C & M10 GPS)
params.IMU.accel_noise_density = 6.9e-4;   % (m/s^2)/sqrt(Hz)
params.IMU.accel_bias_instab  = 0.008;     % m/s^2
params.IMU.gyro_noise_density = 4.9e-5;    % (rad/s)/sqrt(Hz)
params.IMU.gyro_bias_instab   = 8.7e-5;    % rad/s
params.GPS.sigma_xy = 1.5;                % meters (horizontal)
params.GPS.sigma_z  = 2.5;                % meters (vertical)
params.Baro.sigma_z = 0.6;                % meters
params.Mag.sigma_deg = 3.0;               % degrees
params.Mag.sigma_rad = params.Mag.sigma_deg * pi/180; % radians

%% 5. EKF Tuning Parameters (Q & R Matrices) - EXTREMELY OPTIMIZED for Enhanced Performance
% IMPROVED: Base Q matrix optimized for extremely robust tracking and GPS outage scenarios
% Position: Extremely low uncertainty for extremely smooth tracking
% Velocity: Extremely low uncertainty for extremely stable response
% Attitude: Extremely increased for extremely good stability and responsiveness
% Use moderate process noise values that allow filter to converge without
% becoming over-confident.  These are typical for a small quadrotor.
params.Q = diag([0.05 0.05 0.05, 0.2 0.2 0.2, 0.1 0.1 0.1]);

% ENHANCED: Adaptive noise scaling parameters - extremely conservative
params.adaptive_noise = true;  % Enable adaptive noise scaling
params.turn_threshold = 0.2;   % Extremely reduced for extremely early turn detection (rad/s)
params.noise_scale_turn = 1.1; % Extremely reduced scaling during turns for extremely good stability
params.noise_scale_normal = 0.5; % Extremely reduced normal flight scaling for extremely smooth response
params.gps_outage_max_scale = 1.5; % Extremely reduced maximum scaling during GPS outage

% OPTIMIZED: Measurement noise matrices for robust sensor fusion
% GPS: Slightly more conservative for better rejection of outliers
params.R_gps = diag([2.0^2, 2.0^2, 3.0^2]); % INCREASED for robustness
params.R_baro = params.R_gps(3,3); % Equal weight with GPS-z
params.R_mag = (6 * pi/180)^2; % INCREASED: 6 degrees for mag interference rejection

% IMPROVED: Innovation gate thresholds for robust measurement acceptance
params.innovation_gate_gps = 25.0;   % Allow larger position innovations (m)
params.innovation_gate_baro = 10.0;  % Baro innovation threshold (m)
params.innovation_gate_mag  = deg2rad(60); % Magnetometer innovation threshold (rad)
params.innovation_gate_imu  = 30.0;  % IMU innovation threshold (m/s²)
params.max_angular_rate = deg2rad(120); % Maximum allowed angular rate

%% 6. Cross-Coupling Parameters
params.drag_coeff = 0.1;      % Aerodynamic drag coefficient
params.gyro_momentum = 0.1;   % Rotor angular momentum (simplified)
params.coriolis_enabled = true; % Enable Coriolis effects

%% 7. Safety Limits (used by cascaded_control_system)
params.safety.max_velocity_mps = 18;  % Allow up to 18 m/s horizontal before limiting
params.safety.initial_grace_sec = 2.0; % Grace period for EKF convergence
