% forcesMoments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       Force   - forces
%       Torque  - moments
%       Va      - airspeed
%       alpha   - angle of attack
%       beta    - sideslip angle
%       w_ned   - wind vector in the inertial frame
%

function out = forces_moments(x, delta, Va, alpha, beta, uav_parameters)
    phi     = x(7); % roll angle
    theta   = x(8); % pitch angle
    % psi     = x(9); % yaw angle
    p       = x(10); % body frame roll rate
    q       = x(11); % body frame pitch rate
    r       = x(12); % body frame yaw rate

    delta_e = delta(1); % elevator deflection
    delta_a = delta(2); % aileron deflection
    delta_r = delta(3); % rudder deflection
    delta_t = delta(4); % propeller thrust (pulse-width-modulation command)

    mass = uav_parameters(10);

    gravity = uav_parameters(15);
    S_wing = uav_parameters(16);
    b = uav_parameters(17);
    c = uav_parameters(18);
    S_prop = uav_parameters(19);
    rho = uav_parameters(20);
    e = uav_parameters(21);
    AR = uav_parameters(22);
    C_L_0 = uav_parameters(23);
    % C_D_0 = uav_parameters(24);
    C_m_0 = uav_parameters(25);
    C_L_alpha = uav_parameters(26);
    % C_D_alpha = uav_parameters(27);
    C_m_alpha = uav_parameters(28);
    C_L_q = uav_parameters(29);
    C_D_q = uav_parameters(30);
    C_m_q = uav_parameters(31);
    C_L_delta_e = uav_parameters(32);
    C_D_delta_e = uav_parameters(33);
    C_m_delta_e = uav_parameters(34);
    M = uav_parameters(35);
    alpha0 = uav_parameters(36);
    % epsilon = uav_parameters(37);
    C_D_p = uav_parameters(38);
    C_Y_0 = uav_parameters(39);
    C_ell_0 = uav_parameters(40);
    C_n_0 = uav_parameters(41);
    C_Y_beta = uav_parameters(42);
    C_ell_beta = uav_parameters(43);
    C_n_beta = uav_parameters(44);
    C_Y_p = uav_parameters(45);
    C_ell_p = uav_parameters(46);
    C_n_p = uav_parameters(47);
    C_Y_r = uav_parameters(48);
    C_ell_r = uav_parameters(49);
    C_n_r = uav_parameters(50);
    C_Y_delta_a = uav_parameters(51);
    C_ell_delta_a = uav_parameters(52);
    C_n_delta_a = uav_parameters(53);
    C_Y_delta_r = uav_parameters(54);
    C_ell_delta_r = uav_parameters(55);
    C_n_delta_r = uav_parameters(56);
    C_prop = uav_parameters(57);
    k_motor = uav_parameters(58);
    k_Omega = uav_parameters(59);
    k_T_p = uav_parameters(60);
    
    % compute external forces and torques on aircraft
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);
    aeroFactor = 0.5 * rho * Va^2 * S_wing;
    C_D = calc_C_D(C_L_0,C_L_alpha,C_D_p,e,AR,alpha);
    C_L = calc_C_L(C_L_0,C_L_alpha,M,alpha0,alpha);
    C_X = -C_D*cosalpha + C_L*sinalpha;
    C_X_q = -C_D_q*cosalpha + C_L_q*sinalpha;
    C_X_delta_e = -C_D_delta_e*cosalpha + C_L_delta_e*sinalpha;
    C_Z = -C_D*sinalpha - C_L*cosalpha;
    C_Z_q = -C_D_q*sinalpha - C_L_q*cosalpha;
    C_Z_delta_e = -C_D_delta_e*sinalpha - C_L_delta_e*cosalpha;
    mg = mass*gravity;
    dbVa = 2*Va;
    Force = NaN(3,1);
    Force(1) = -mg*sin(theta) + ...
        aeroFactor*(C_X+C_X_q*q*c/dbVa+C_X_delta_e*delta_e) + ...
        0.5*rho*S_prop*C_prop*((k_motor*delta_t)^2-Va^2);
    Force(2) = mg*cos(theta)*sin(phi) + ...
        aeroFactor*(C_Y_0+C_Y_beta*beta+ ...
        (C_Y_p*p+C_Y_r*r)*b/dbVa+C_Y_delta_a*delta_a+ ... 
        C_Y_delta_r*delta_r);
    Force(3) = mg*cos(theta)*cos(phi) + ...
        aeroFactor*(C_Z+C_Z_q*q*c/dbVa+C_Z_delta_e*delta_e);
    Torque = NaN(3,1);
    Torque(1) = aeroFactor*b*(C_ell_0+C_ell_beta*beta+...
        (C_ell_p*p+C_ell_r*r)*b/dbVa+C_ell_delta_a*delta_a+...
        C_ell_delta_r*delta_r) - ...
        k_T_p*(k_Omega*delta_t)^2;
    Torque(2) = aeroFactor*c*(C_m_0+C_m_alpha*alpha+...
        C_m_q*q*c/dbVa+C_m_delta_e*delta_e);   
    Torque(3) = aeroFactor*b*(C_n_0+C_n_beta*beta+...
        (C_n_p*p+C_n_r*r)*b/dbVa+C_n_delta_a*delta_a+C_n_delta_r*delta_r);
    out = [Force; Torque];
end

function C_L = calc_C_L(C_L_0,C_L_alpha,M,alpha0,alpha)
    e1 = exp(-M*(alpha-alpha0));
    e2 = exp(M*(alpha+alpha0));
    sigma = (1+e1+e2)/((1+e1)*(1+e2));
    C_L = (1-sigma)*(C_L_0+C_L_alpha*alpha) + ...
        sigma*2*sign(alpha)*cos(alpha)*(sin(alpha))^2;
end

function C_D = calc_C_D(C_L_0,C_L_alpha,C_D_p,e,AR,alpha)
    C_D = C_D_p + (C_L_0+C_L_alpha*alpha)^2/(pi*e*AR);
end
