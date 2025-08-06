function R = rotation_matrix(phi, theta, psi)
    % Calculates the ZYX Body-to-World rotation matrix (yaw-pitch-roll)
    % phi: roll (x-axis rotation)
    % theta: pitch (y-axis rotation) 
    % psi: yaw (z-axis rotation)
    
    % Safety check for numerical stability
    if abs(cos(theta)) < 1e-6
        warning('Near-singularity detected in rotation matrix (cos(theta) near zero)');
        % Use small angle approximation for pitch
        theta = sign(theta) * 1e-6;
    end
    
    cph = cos(phi); sph = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);

    R = [
        cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
        sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
        -sth,    cth*sph,               cth*cph
    ];
    
    % Ensure orthogonality (numerical stability)
    [U, ~, V] = svd(R);
    R = U * V';
end