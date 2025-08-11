function [x, P] = ukf9_step(x, P, imu, z, params, dt, sensor_type)
% ukf9_step - Unscented Kalman Filter for 9-state model with IMU-driven motion

n = 9;
% UKF scaling parameters (more numerically robust spread)
alpha = 0.3;   % broader spread → better conditioning
beta  = 2;     % optimal for Gaussian
kappa = 0;
lambda = alpha^2*(n + kappa) - n;
c = n + lambda;
Wm = [lambda/c, repmat(1/(2*c), 1, 2*n)];
Wc = Wm; Wc(1) = Wc(1) + (1 - alpha^2 + beta);

% Sigma points
% Ensure P is symmetric positive definite before factorization
P = (P + P')/2;
jitter = 1e-12;
S = [];
for attempt = 1:6
    M = (c)*(P + jitter*eye(n)); M = (M + M')/2;
    try
        S = chol(M, 'lower');
        break;
    catch
        jitter = jitter * 10;
    end
end
if isempty(S)
    % Eigenvalue fallback to construct a valid square-root factor
    [U,D] = eig(M);
    d = real(diag(D));
    d(d < 1e-12) = 1e-12;
    S = U * diag(sqrt(d));
end
X = zeros(n, 2*n+1);
X(:,1) = x;
for i=1:n
    X(:,i+1)   = x + S(:,i);
    X(:,i+1+n) = x - S(:,i);
end

% Process function: f(x, imu)
X_pred = zeros(size(X));
for i=1:size(X,2)
    xd = X(:,i) + drone_dynamics_imu(0, X(:,i), imu, params) * dt;
    xd(7:9) = wrapToPi(xd(7:9));
    X_pred(:,i) = xd;
end

% Predicted mean and covariance
x_pred = X_pred * Wm';
P_pred = zeros(n);
for i=1:2*n+1
    dx = X_pred(:,i) - x_pred;
    % ensure small-angle consistency for attitude residual
    dx(7:9) = atan2(sin(dx(7:9)), cos(dx(7:9)));
    P_pred = P_pred + Wc(i) * (dx * dx');
end
Q = params.Q * dt;
P_pred = (P_pred + P_pred')/2 + Q;

switch sensor_type
    case 'IMU'
        % Condition P to be SPD
        P = (P_pred + P_pred')/2 + 1e-12*eye(n);
        x = x_pred; 
        return;
    case 'GPS'
        h = @(x) x(1:3);
        R = params.R_gps;
        m = 3;
    case 'Baro'
        h = @(x) -x(3);
        R = params.R_baro;
        m = 1;
    case 'Mag'
        h = @(x) x(9);
        R = params.R_mag;
        m = 1;
    otherwise
        error('Unknown sensor type');
end

% Transform sigma points
Z = zeros(m, 2*n+1);
for i=1:2*n+1
    Z(:,i) = h(X_pred(:,i));
end
z_pred = Z * Wm';
Pzz = zeros(m);
Pxz = zeros(n, m);
for i=1:2*n+1
    dz = Z(:,i) - z_pred; if m==1, dz = dz(:); end
    dx = X_pred(:,i) - x_pred;
    dx(7:9) = atan2(sin(dx(7:9)), cos(dx(7:9)));
    Pzz = Pzz + Wc(i) * (dz * dz');
    Pxz = Pxz + Wc(i) * (dx * dz');
end
Pzz = Pzz + R + 1e-9*eye(m);
K = Pxz / Pzz;
x = x_pred + K * (z - z_pred);
P = P_pred - K * Pzz * K';
% Symmetrize and add small jitter to maintain SPD
P = (P + P')/2 + 1e-12*eye(n);
x(7:9) = wrapToPi(x(7:9));

end


