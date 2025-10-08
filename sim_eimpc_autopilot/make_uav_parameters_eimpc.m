function uavParams = make_uav_parameters_eimpc()
    % https://en.wikipedia.org/wiki/AAI_Aerosonde
    % physical parameters of airframe
    gravity = 9.81;
    mass = 13.5;
    Jx   = 0.824;
    Jy   = 1.135;
    Jz   = 1.759;
    Jxz  = 0.120;
    S_wing        = 0.55;
    b             = 2.90;
    c             = 0.19;
    S_prop        = 0.2027;
    rho           = 1.2682;
    e             = 0.9;
    AR            = b^2/S_wing;
    
    % Gamma parameters from uavbook page 36
    Gamma  = Jx*Jz-Jxz^2;
    Gamma1 = (Jxz*(Jx-Jy+Jz))/Gamma;
    Gamma2 = (Jz*(Jz-Jy)+Jxz*Jxz)/Gamma;
    Gamma3 = Jz/Gamma;
    Gamma4 = Jxz/Gamma;
    Gamma5 = (Jz-Jx)/Jy;
    Gamma6 = Jxz/Jy;
    Gamma7 = (Jx*(Jx-Jy)+Jxz*Jxz)/Gamma;
    Gamma8 = Jx/Gamma;
    
    % aerodynamic coefficients from Appendix E
    C_L_0         = 0.28;
    C_D_0         = 0.03;
    C_m_0         = -0.02338;
    C_L_alpha     = 3.45;
    C_D_alpha     = 0.03;
    C_m_alpha     = -0.38;
    C_L_q         = 0.0;
    C_D_q         = 0.0;
    C_m_q         = -3.6;
    C_L_delta_e   = -0.36;
    C_D_delta_e   = 0.0;
    C_m_delta_e   = -0.5;
    M             = 50;
    alpha0        = 0.4712;
    epsilon       = 0.1592;
    C_D_p         = 0.0437;
    C_Y_0         = 0.0;
    C_ell_0       = 0.0;
    C_n_0         = 0.0;
    C_Y_beta      = -0.98;
    C_ell_beta    = -0.12;
    C_n_beta      = 0.25;
    C_Y_p         = 0.0;
    C_ell_p       = -0.26;
    C_n_p         = 0.022;
    C_Y_r         = 0.0;
    C_ell_r       = 0.14;
    C_n_r         = -0.35;
    C_Y_delta_a   = 0.0;
    C_ell_delta_a = 0.08;
    C_n_delta_a   = 0.06;
    C_Y_delta_r   = 0.19;
    C_ell_delta_r = 0.105;
    C_n_delta_r   = -0.032;
    
    % Parameters for propulsion thrust and torque models
    % D_prop = 0.508;     % prop diameter in m
    
    % Motor parameters
    % K_V = 145;                    % from datasheet RPM/V
    % KQ = (1/K_V)*60/(2*pi);   % KQ in N-m/A, V-s/rad
    % R_motor = 0.042;              % ohms
    % i0 = 1.5;                     % no-load (zero-torque) current (A)
    
    % Inputs
    % ncells = 12;
    % V_max = 3.7*ncells;       % max voltage for specified number of battery cells
    
    % Coeffiecients from prop_data fit
    % C_Q2 = -0.01664;
    % C_Q1 = 0.004970;
    % C_Q0 = 0.005230;
    % C_T2 = -0.1079;
    % C_T1 = -0.06044;
    % C_T0 = 0.09357;

    % Coeffiecients from Appendix E
    C_prop = 1;
    k_motor = 80;
    k_Omega = 0;
    k_T_p = 0;
    
    % mavParams object
    uavParams = NaN(60,1);
    uavParams(1) = Gamma1;
    uavParams(2) = Gamma2;
    uavParams(3) = Gamma3;
    uavParams(4) = Gamma4;
    uavParams(5) = Gamma5;
    uavParams(6) = Gamma6;
    uavParams(7) = Gamma7;
    uavParams(8) = Gamma8;
    uavParams(9) = Gamma;
    uavParams(10) = mass;
    uavParams(11) = Jx;
    uavParams(12) = Jy;
    uavParams(13) = Jz;
    uavParams(14) = Jxz;
    uavParams(15) = gravity;
    uavParams(16) = S_wing;
    uavParams(17) = b;
    uavParams(18) = c;
    uavParams(19) = S_prop;
    uavParams(20) = rho;
    uavParams(21) = e;
    uavParams(22) = AR;
    uavParams(23) = C_L_0;
    uavParams(24) = C_D_0;
    uavParams(25) = C_m_0;
    uavParams(26) = C_L_alpha;
    uavParams(27) = C_D_alpha;
    uavParams(28) = C_m_alpha;
    uavParams(29) = C_L_q;
    uavParams(30) = C_D_q;
    uavParams(31) = C_m_q;
    uavParams(32) = C_L_delta_e;
    uavParams(33) = C_D_delta_e;
    uavParams(34) = C_m_delta_e;
    uavParams(35) = M;
    uavParams(36) = alpha0;
    uavParams(37) = epsilon;
    uavParams(38) = C_D_p;
    uavParams(39) = C_Y_0;
    uavParams(40) = C_ell_0;
    uavParams(41) = C_n_0;
    uavParams(42) = C_Y_beta;
    uavParams(43) = C_ell_beta;
    uavParams(44) = C_n_beta;
    uavParams(45) = C_Y_p;
    uavParams(46) = C_ell_p;
    uavParams(47) = C_n_p;
    uavParams(48) = C_Y_r;
    uavParams(49) = C_ell_r;
    uavParams(50) = C_n_r;
    uavParams(51) = C_Y_delta_a;
    uavParams(52) = C_ell_delta_a;
    uavParams(53) = C_n_delta_a;
    uavParams(54) = C_Y_delta_r;
    uavParams(55) = C_ell_delta_r;
    uavParams(56) = C_n_delta_r;
    uavParams(57) = C_prop;
    uavParams(58) = k_motor;
    uavParams(59) = k_Omega;
    uavParams(60) = k_T_p;
end