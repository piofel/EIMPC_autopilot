% wind_gusts: vector of gusts along body axes
% wind_steady: vector of steady wind along NED axes
% x: dynamics state vector
function [Va, alpha, beta] = airdata(x,wind_steady,wind_gusts)
    u       = x(4); % velocity along body x-axis
    v       = x(5); % velocity along body y-axis
    w       = x(6); % velocity along body z-axis
    phi     = x(7); % roll angle
    theta   = x(8); % pitch angle
    psi     = x(9); % yaw angle

    R = rotation_matrix(phi,theta,psi);  % NED to body

    % compute wind data in body frame
    Vw = wind_gusts + R*wind_steady;
    
    % compute air data
    Var = [u-Vw(1); v-Vw(2); w-Vw(3)];
    ur = Var(1);
    vr = Var(2);
    wr = Var(3);
    Va = norm(Var);
    alpha = atan(wr/ur);
    beta = asin(vr/Va);
end