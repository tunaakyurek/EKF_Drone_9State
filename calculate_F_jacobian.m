function F = calculate_F_jacobian(m, u, dt)
    % Calculates the process model Jacobian F with improved angular coupling
    % m: state vector [p, v, angles]'
    % u: input vector [thrust; tau_phi; tau_theta; tau_psi]'
    
    phi = m(7); theta = m(8); psi = m(9);
    thrust = u(1);
    tau = u(2:4);

    I3 = eye(3);
    Z3 = zeros(3);

    % Compute partial derivatives numerically with improved stability
    epsilon = 1e-8; % Smaller epsilon for better numerical accuracy
    f_body = [0; 0; thrust];

    % d(R*f)/d(angles) - improved numerical differentiation
    R0 = rotation_matrix(phi, theta, psi);
    dRa_dphi   = (rotation_matrix(phi+epsilon, theta, psi) * f_body - R0 * f_body) / epsilon;
    dRa_dtheta = (rotation_matrix(phi, theta+epsilon, psi) * f_body - R0 * f_body) / epsilon;
    dRa_dpsi   = (rotation_matrix(phi, theta, psi+epsilon) * f_body - R0 * f_body) / epsilon;
    dRa_dangles = [dRa_dphi, dRa_dtheta, dRa_dpsi];

    % Improved d(J*omega)/d(angles) calculation
    % This accounts for the coupling between angular rates and attitude
    dJ_dangles = zeros(3,3);
    
    % Add cross-coupling terms for angular dynamics
    if abs(cos(theta)) > 1e-6
        % d(omega)/d(phi) - effect of roll on angular rates
        E_phi_plus = [1, sin(phi+epsilon)*tan(theta), cos(phi+epsilon)*tan(theta);
                      0, cos(phi+epsilon),           -sin(phi+epsilon);
                      0, sin(phi+epsilon)/cos(theta), cos(phi+epsilon)/cos(theta)];
        E_phi_minus = [1, sin(phi-epsilon)*tan(theta), cos(phi-epsilon)*tan(theta);
                       0, cos(phi-epsilon),           -sin(phi-epsilon);
                       0, sin(phi-epsilon)/cos(theta), cos(phi-epsilon)/cos(theta)];
        
        % d(omega)/d(theta) - effect of pitch on angular rates
        E_theta_plus = [1, sin(phi)*tan(theta+epsilon), cos(phi)*tan(theta+epsilon);
                        0, cos(phi),                   -sin(phi);
                        0, sin(phi)/cos(theta+epsilon), cos(phi)/cos(theta+epsilon)];
        E_theta_minus = [1, sin(phi)*tan(theta-epsilon), cos(phi)*tan(theta-epsilon);
                         0, cos(phi),                   -sin(phi);
                         0, sin(phi)/cos(theta-epsilon), cos(phi)/cos(theta-epsilon)];
        
        % Numerical derivatives of angular rates with respect to angles
        I_inv = diag(1./[0.0023, 0.0023, 0.004]); % Inverse inertia matrix
        
        domega_dphi = (E_phi_plus * I_inv * tau - E_phi_minus * I_inv * tau) / (2*epsilon);
        domega_dtheta = (E_theta_plus * I_inv * tau - E_theta_minus * I_inv * tau) / (2*epsilon);
        domega_dpsi = zeros(3,1); % Yaw has minimal effect on angular rates in this model
        
        dJ_dangles = [domega_dphi, domega_dtheta, domega_dpsi];
    end

    % Add velocity-dependent terms for cross-coupling
    vel = m(4:6);
    if norm(vel) > 0.1 % Only add coupling when moving
        % d(acc)/d(vel) - drag effects
        drag_coeff = 0.1;
        dacc_dvel = -drag_coeff * eye(3);
    else
        dacc_dvel = zeros(3);
    end

    F = [
        I3, dt*I3, Z3;
        Z3, I3 + dt*dacc_dvel, dt * dRa_dangles;
        Z3, Z3,    I3 + dt * dJ_dangles
    ];
    
    % Ensure numerical stability of the Jacobian
    [U, S, V] = svd(F);
    S = max(S, 1e-12); % Prevent singular values from becoming too small
    F = U * S * V';
end