function w = wind_ned(x,wind_steady,wind_gusts)
    % compute wind data in NED
    phi     = x(7); % roll angle
    theta   = x(8); % pitch angle
    psi     = x(9); % yaw angle
    R_body_to_NED = rotation_matrix(phi,theta,psi)';
    w = wind_steady + R_body_to_NED*wind_gusts;
end