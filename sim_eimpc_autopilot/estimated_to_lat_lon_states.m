function [x_lat_m,x_lon_m] = estimated_to_lat_lon_states(xhat)
    % pn       = xhat(1);  % inertial North position
    % pe       = xhat(2);  % inertial East position
    h        = xhat(3);  % altitude
    Va       = xhat(4);  % airspeed
    alpha    = xhat(5);  % angle of attack
    beta     = xhat(6);  % side slip angle
    phi      = xhat(7);  % roll angle
    theta    = xhat(8);  % pitch angle
    % chi      = xhat(9);  % course angle
    p        = xhat(10); % body frame roll rate
    q        = xhat(11); % body frame pitch rate
    r        = xhat(12); % body frame yaw rate
    % Vg       = xhat(13); % ground speed
    wn       = xhat(14); % wind North
    we       = xhat(15); % wind East
    psi      = xhat(16); % heading

    R = angle2dcm(psi,theta,phi);  % NED to body
    wind_body = R*[wn;we;0];
    Var = Va * [cos(alpha)*cos(beta); sin(beta); sin(alpha)*cos(beta)];
    Vgb = Var + wind_body;
    u = Vgb(1);
    v = Vgb(2);
    w = Vgb(3);

    x_lat_m = [v p r phi psi]';
    x_lon_m = [u w q theta -h]';
end