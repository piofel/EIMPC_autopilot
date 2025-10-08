function y = autopilot(xhat,commandSignals,autopilotParams,piInitFlag)

    % pn       = xhat(1);  % inertial North position
    % pe       = xhat(2);  % inertial East position
    h        = xhat(3);  % altitude
    Va       = xhat(4);  % airspeed
    % alpha    = xhat(5);  % angle of attack
    beta     = xhat(6);  % side slip angle
    phi      = xhat(7);  % roll angle
    theta    = xhat(8);  % pitch angle
    chi      = xhat(9);  % course angle
    p        = xhat(10); % body frame roll rate
    q        = xhat(11); % body frame pitch rate
    % r        = xhat(12); % body frame yaw rate
    % Vg       = xhat(13); % ground speed
    % wn       = xhat(14); % wind North
    % we       = xhat(15); % wind East
    % psi      = xhat(16); % heading
    % bx       = xhat(17); % x-gyro bias
    % by       = xhat(18); % y-gyro bias
    % bz       = xhat(19); % z-gyro bias

    Va_c     = commandSignals(1);  % commanded airspeed (m/s)
    h_c      = commandSignals(2);  % commanded altitude (m)
    chi_c    = commandSignals(3);  % commanded course (rad)

    %----------------------------------------------------------
    % lateral autopilot
    chi_ref = wrap(chi_c, chi);
    phi_c   = course_with_roll(chi_ref,chi,piInitFlag,autopilotParams);
    delta_r = sideslip_with_rudder(beta, piInitFlag, autopilotParams);
    delta_a = roll_with_aileron(phi_c, phi, p, autopilotParams);

    %----------------------------------------------------------
    % longitudinal autopilot
    % h_ref = saturate(h_c, h-altitude_zone, h+altitude_zone);  % from Beard&McLain - seems harmful
    h_ref = h_c;
    altitude_state_machine_on = autopilotParams(33);
    if altitude_state_machine_on == 0
        delta_t = airspeed_with_throttle(Va_c,Va,piInitFlag,autopilotParams);
        theta_c = altitude_with_pitch(h_ref, h, piInitFlag, autopilotParams);
    else
        altitude_zone = autopilotParams(14);
        h_take_off = autopilotParams(31);
        theta_take_off = autopilotParams(32);
        if h >= h_c+altitude_zone  % descend zone
            delta_t = 0.3;
            theta_c = airspeed_with_pitch(Va_c,Va,piInitFlag,autopilotParams);
        elseif h < h_c+altitude_zone && h >= h_c-altitude_zone  % altitude hold zone
            delta_t = airspeed_with_throttle(Va_c,Va,piInitFlag,autopilotParams);
            theta_c = altitude_with_pitch(h_ref, h, piInitFlag, autopilotParams);
        elseif h < h_c-altitude_zone && h >= h_take_off  % climb zone
            delta_t = 1;
            theta_c = airspeed_with_pitch(Va_c,Va,piInitFlag,autopilotParams);
        elseif h < h_take_off  % take-off zone
            delta_t = 1;
            theta_c = theta_take_off;
        else
            error('Unknown state')
        end
    end
    delta_e = pitch_with_elevator(theta_c, theta, q, autopilotParams);
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c;...              % theta
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
end

% wraps chi_c, so that it is within +-pi of chi
function chi_c = wrap(chi_c, chi)
    while chi_c-chi > pi
        chi_c = chi_c - 2*pi;
    end
    while chi_c-chi < -pi
        chi_c = chi_c + 2*pi;
    end
end

function output = saturate(input, low_limit, up_limit)
    if low_limit > up_limit
        error("Lower limit must be less than the upper limit!");
    end
    if input <= low_limit
        output = low_limit;
    elseif input >= up_limit
        output = up_limit;
    else
        output = input;
    end
end

function u = pi_controller(y_c,y,init_flag,kp,ki,low_limit,up_limit,Ts)
    persistent integrator;
    persistent error_d1;
    if init_flag==1 % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0; % _d1 means delayed by one time step
    end
    error = y_c - y; % compute the current error
    integrator = integrator + (Ts/2)*(error + error_d1); % update integr.
    error_d1 = error; % update the error for next time through the loop
    u_unsat = kp*error + ki*integrator;
    u = saturate(u_unsat,low_limit,up_limit);
    if ki~=0
        % integrator anti−windup
        integrator = integrator + Ts/ki * (u - u_unsat);
    end
end

%--------------------------------------------------------------------
% course_with_roll
%   - regulate heading using the roll command
%--------------------------------------------------------------------
function phi_c_sat = ...
    course_with_roll(chi_c, chi, flag, autopilotParams)
    kp = autopilotParams(3);  % course_kp
    ki = autopilotParams(4);  % course_ki
    Ts = autopilotParams(20);
    limit = autopilotParams(26);  % phi_max
    phi_c_sat = pi_controller(chi_c,chi,flag,kp,ki,-limit,limit,Ts);
end

%--------------------------------------------------------------------
% roll_with_aileron
%   - regulate roll using aileron
%--------------------------------------------------------------------
function delta_a = roll_with_aileron(phi_c, phi, p, autopilotParams)
    kp = autopilotParams(1);  % roll_kp
    kd = autopilotParams(2);  % roll_kd
    delta_a_max = autopilotParams(23);
    delta_a = kp*(phi_c-phi) - kd*p;
    delta_a = saturate(delta_a,-delta_a_max,delta_a_max);
end

%--------------------------------------------------------------------
% coordinated_turn_hold
%   - sideslip with rudder
%--------------------------------------------------------------------
function delta_r = sideslip_with_rudder(beta, flag, autopilotParams)
    kp = autopilotParams(5);  % sideslip_kp
    ki = autopilotParams(6);  % sideslip_ki
    Ts = autopilotParams(20);
    delta_r_max = autopilotParams(24);
    delta_r = pi_controller(0,beta,flag,kp,ki,-delta_r_max,delta_r_max,Ts);
end

%--------------------------------------------------------------------
% pitch_with_elevator
%   - regulate pitch using elevator
%--------------------------------------------------------------------
function delta_e = pitch_with_elevator(theta_c,theta,q,autopilotParams)
    pitch_kp = autopilotParams(9);  % pitch_kp
    pitch_kd = autopilotParams(10);  % pitch_kd
    delta_e_max = autopilotParams(22);
    delta_e = pitch_kp*(theta_c-theta) - pitch_kd*q;
    delta_e = saturate(delta_e,-delta_e_max,delta_e_max);
end

%--------------------------------------------------------------------
% altitude_with_pitch
%   - regulate altitude using pitch angle
%--------------------------------------------------------------------
function theta_c_sat = altitude_with_pitch(h_c,h,flag,autopilotParams)
    kp = autopilotParams(12);  % altitude_kp
    ki = autopilotParams(13);  % altitude_ki
    Ts = autopilotParams(20);
    limit = autopilotParams(27);  % theta_max
    theta_c_sat = pi_controller(h_c,h,flag,kp,ki,-limit,limit,Ts);
end

%--------------------------------------------------------------------
% airspeed_with_throttle
%   - regulate airspeed using throttle
%--------------------------------------------------------------------
function delta_t_sat = airspeed_with_throttle(Va_c,Va,flag,autopilotParams)
    kp = autopilotParams(15);  % airspeed_throttle_kp
    ki = autopilotParams(16);  % airspeed_throttle_ki
    delta_t_trim = autopilotParams(19);  % throttle trim value
    Ts = autopilotParams(20);
    delta_t_max = autopilotParams(25);
    low_limit = -delta_t_trim;
    up_limit = delta_t_max - delta_t_trim;
    delta_t_sat = delta_t_trim + ....
        pi_controller(Va_c,Va,flag,kp,ki,low_limit,up_limit,Ts);
end

%--------------------------------------------------------------------
% airspeed_with_pitch
%   - regulate airspeed using pitch
%--------------------------------------------------------------------
function theta_c_sat = airspeed_with_pitch(Va_c,Va,flag,autopilotParams)
    kp_V2 = autopilotParams(17);  % airspeed_pitch_kp
    ki_V2 = autopilotParams(18);  % airspeed_pitch_ki
    Ts = autopilotParams(20);
    limit = autopilotParams(27);  % theta_max
    theta_c_sat = pi_controller(Va_c,Va,flag,kp_V2,ki_V2,-limit,limit,Ts);
end