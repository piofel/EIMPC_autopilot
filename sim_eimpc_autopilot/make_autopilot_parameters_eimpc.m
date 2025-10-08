weight_elevator = 28e6;  % https://se.mathworks.com/help/mpc/ug/tuning-weights.html
weight_aileron = 6e4;
weight_rudder = 6e4;
weight_throttle = 8e6;

%---------autopilot parameters vector---------------
autopilotParams = NaN(128,1);

% autopilotParams(1:18) are spare

% autopilotParams(19:30) common autpilot parameters
autopilotParams(19) = u_trim(4);  % throttle trim value
autopilotParams(20) = Ts;
autopilotParams(21) = NaN;  % obsolete - formerly autopilotType
autopilotParams(22) = deg2rad(45); % delta_e_max
autopilotParams(23) = deg2rad(45); % delta_a_max
autopilotParams(24) = deg2rad(45); % delta_r_max
autopilotParams(25) = 1;  % delta_t_max
autopilotParams(26) = NaN;  % obsolete - formerly phi_max
autopilotParams(27) = NaN;  % obsolete - formerly theta_max
autopilotParams(28) = u_trim(1);  % elevator trim value
autopilotParams(29) = u_trim(2);  % aileron trim value
autopilotParams(30) = u_trim(3);  % rudder trim value

% autopilotParams(31:50) MPC autpilot parameters
autopilotParams(31) = 60;  % Np / prediction horizon for lateral autopiot
autopilotParams(32) = 6;  % Nc / control horizon for lateral autopiot
autopilotParams(33) = 60;  % Np / prediction horizon for longitudinal autopiot
autopilotParams(34) = 6;  % Nc / control horizon for longitudinal autopiot
autopilotParams(35) = NaN;  % obsolete - formerly weight_chi
autopilotParams(36) = NaN;  % obsolete - formerly weight_beta
autopilotParams(37) = weight_elevator;  % control weight on elevator
autopilotParams(38) = weight_aileron;  % control weight on aileron
autopilotParams(39) = weight_rudder;  % control weight on rudder
autopilotParams(40) = weight_throttle;  % control weight on throttle
autopilotParams(41) = NaN;  % redundant
autopilotParams(42) = 3;  % optimizer ID (1...3)

% autopilotParams(46:90) state estimator parameters
autopilotParams(46) = 1.8*Ts; % 0.4;  % state estimator delay
autopilotParams(51:69) = 6e-3 * [ones(1,4) repmat(1e-4,1,8) ones(1,3) repmat(1e-4,1,4)]';  % state estimator noise power (variation)
autopilotParams(71:89) = 3*N_sim + [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19]';  % state estimator noise seeds