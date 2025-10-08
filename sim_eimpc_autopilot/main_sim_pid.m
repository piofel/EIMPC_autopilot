clc
close all
clearvars

Ts = 0.01;
stopTime = 100;
N_sim = 1;
disp_ne_track = 1;

make_initial_state
make_nominal_flight_parameters_eimpc  % same as in the EIMPC simulations
uavParams = make_uav_parameters_eimpc();  % same as in the EIMPC simulations
make_wind_parameters
uav_trim
compute_uav_tf_model

%----------roll loop-------------
e_phi_max = deg2rad(90);
zeta_phi = 1;
phi_max = deg2rad(90);
delta_a_max = deg2rad(45);
wn_phi = sqrt(abs(a_phi2)*delta_a_max/e_phi_max);
roll_kp = delta_a_max * sign(a_phi2)/e_phi_max;
roll_kd = (2*zeta_phi*wn_phi-a_phi1)/a_phi2;
clearvars e_phi_max zeta_phi a_phi1 a_phi2

%----------course loop-------------
zeta_chi = 50;
W_chi = 4000;
wn_chi = wn_phi / W_chi;
course_kp = 2*zeta_chi*wn_chi*Va0/gravity;
course_ki = wn_chi^2*Va0/gravity;
clearvars wn_phi zeta_chi W_chi wn_chi

%----------sideslip loop-------------
zeta_beta = Inf;
e_beta_max = deg2rad(90);
delta_r_max = deg2rad(45);
sideslip_kp = delta_r_max*sign(a_beta2)/e_beta_max;
sideslip_ki = ((a_beta1+a_beta2*sideslip_kp)/(2*zeta_beta))^2/a_beta2;
clearvars e_beta_max zeta_beta a_beta1 a_beta2

%----------yaw damper-------------
yaw_damper_kp = NaN;  % TODO
yaw_damper_tau_r = NaN;  % TODO

%----------pitch loop-------------
e_theta_max = deg2rad(10);
zeta_theta = 1;
theta_max = deg2rad(30);
delta_e_max = deg2rad(45);
wn_theta = sqrt(a_theta2+delta_e_max*abs(a_theta3)/e_theta_max);
pitch_kp = delta_e_max*sign(a_theta3)/e_theta_max;
pitch_kd = (2*zeta_theta*wn_theta-a_theta1)/a_theta3;
K_theta_DC = pitch_kp*a_theta3/(a_theta2+pitch_kp*a_theta3);
clearvars e_theta_max zeta_theta a_theta1 a_theta2 a_theta3

%----------altitude loop-------------
W_h = 80;
zeta_h = 1;
altitude_zone = 15;
wn_h = wn_theta/W_h;
altitude_kp = 2*zeta_h*wn_h/(K_theta_DC*Va0);
altitude_ki = wn_h^2/(K_theta_DC*Va0);
clearvars W_h zeta_h wn_h

%---------airspeed hold using throttle---------------
wn_V = pi/100;
zeta_V = 100;
airspeed_throttle_kp = (2*zeta_V*wn_V-a_V1)/a_V2;
airspeed_throttle_ki = wn_V^2/a_V2;
clearvars wn_V zeta_V a_V2

%---------airspeed hold using pitch---------------
zeta_V2 = 1;
W_V2 = 10;
wn_V2 = wn_theta/W_V2;
airspeed_pitch_kp = (a_V1-2*zeta_V2*wn_V2)/(-K_theta_DC*gravity);  % sign changed wrt to (Beard&McLain)
airspeed_pitch_ki = wn_V2^2/(K_theta_DC*gravity);  % sign changed wrt to (Beard&McLain)
clearvars wn_theta W_V2 zeta_V2 a_V1 wn_V2 gravity

%---------autopilot parameters vector---------------
autopilotParams = NaN(128,1);

% autopilotParams(1:18) - PID autpilot parameters
autopilotParams(1) = roll_kp;
autopilotParams(2) = roll_kd;
autopilotParams(3) = course_kp;
autopilotParams(4) = course_ki;
autopilotParams(5) = sideslip_kp;
autopilotParams(6) = sideslip_ki;
autopilotParams(7) = yaw_damper_kp;
autopilotParams(8) = yaw_damper_tau_r;
autopilotParams(9) = pitch_kp;
autopilotParams(10) = pitch_kd;
autopilotParams(11) = K_theta_DC;
autopilotParams(12) = altitude_kp;
autopilotParams(13) = altitude_ki;
autopilotParams(14) = altitude_zone;
autopilotParams(15) = airspeed_throttle_kp;
autopilotParams(16) = airspeed_throttle_ki;
autopilotParams(17) = airspeed_pitch_kp;
autopilotParams(18) = airspeed_pitch_ki;

% autopilotParams(19:30) - common autpilot parameters
autopilotParams(19) = u_trim(4);  % throttle trim value
autopilotParams(20) = Ts;
autopilotParams(21) = NaN;  % obsolete - formerly autopilotType
autopilotParams(22) = delta_e_max;
autopilotParams(23) = delta_a_max;
autopilotParams(24) = delta_r_max;
autopilotParams(25) = 1;  % delta_t_max
autopilotParams(26) = phi_max;
autopilotParams(27) = theta_max;
autopilotParams(28) = u_trim(1);  % elevator trim value
autopilotParams(29) = u_trim(2);  % aileron trim value
autopilotParams(30) = u_trim(3);  % rudder trim value

% autopilotParams(31:35) - altitude-control state machine
autopilotParams(31) = 10;  % h_take_off
autopilotParams(32) = theta_max;  % theta_take_off
autopilotParams(33) = 0;  % altitude state machine on/off (0/1)

% autopilotParams(61:99) - state estimator parameters
autopilotParams(61:79) = 6e-3 * [ones(1,4) repmat(1e-4,1,8) ones(1,3) repmat(1e-4,1,4)]';  % state estimator noise power (variation)
autopilotParams(80) = 0.01; % state estimator delay
autopilotParams(81:99) = 3*N_sim + [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19]';  % state estimator noise seeds

clearvars roll_kp roll_kd course_kp course_ki sideslip_kp sideslip_ki
clearvars yaw_damper_tau_r yaw_damper_kp pitch_kp pitch_kd K_theta_DC
clearvars altitude_kp altitude_ki altitude_zone airspeed_pitch_ki
clearvars airspeed_throttle_kp airspeed_throttle_ki airspeed_pitch_kp
clearvars delta_e_max delta_a_max delta_r_max phi_max theta_max
clearvars alpha_lpf alpha1_lpf lpf_a lpf_a1

mdl = 'uav_sim_pid';
open_system(mdl)
simIn = Simulink.SimulationInput(mdl);
simIn = setModelParameter(simIn,"StopTime",num2str(stopTime));

quality_criteria = NaN(N_sim,8);

u_log_index =   1;
pn_log_index =  2;
pe_log_index =  3;
h_log_index =   4;
chi_log_index = 5;
V_log_index =   6;

tic
for i=1:N_sim

    % C_ell_delta_a = 0.08 + randn*0.02;
    % uavParams(52) = C_ell_delta_a;  % for C_m_alpha robustness test

    % uavParams(28) = -0.38 * 5;   % for C_m_alpha robustness test

    gusts_seeds_str = strcat('[', strjoin(string(gusts_seeds(:,i))), ']');
    set_param([mdl,'/Wind/Gusts Band-Limited White Noise'], 'Seed', gusts_seeds_str);

    simout = sim(simIn,"UseFastRestart","on");

    t = simout.logsout{V_log_index}.Values.Time;
    y = simout.logsout{V_log_index}.Values.Data(1,:,:);
    yc = simout.logsout{V_log_index}.Values.Data(2,:,:);
    L_Va = tracking_quality_criterion_01(t,t,y,yc);
    T_stl_Va = settling_time(t,t,y,yc,2,10);

    t = simout.logsout{h_log_index}.Values.Time;
    y = simout.logsout{h_log_index}.Values.Data(1,:,:);
    yc = simout.logsout{h_log_index}.Values.Data(2,:,:);
    L_h = tracking_quality_criterion_01(t,t,y,yc);
    T_stl_h = settling_time(t,t,y,yc,2,10);

    t = simout.logsout{chi_log_index}.Values.Time;
    y = simout.logsout{chi_log_index}.Values.Data(1,:,:);
    yc = simout.logsout{chi_log_index}.Values.Data(2,:,:);
    L_chi = tracking_quality_criterion_01(t,t,y,yc);
    T_stl_chi = settling_time(t,t,y,yc,5,10);

    t = simout.logsout{u_log_index}.Values.Time;
    u = simout.logsout{u_log_index}.Values.Data;
    [E_t,E_ear] = control_effort_criterion_airplane_01(t,u);

    if disp_ne_track
        pn_samples = simout.logsout{pn_log_index}.Values.Data;
        pe_samples = simout.logsout{pe_log_index}.Values.Data;
        plot(pe_samples,pn_samples,'LineWidth',2)
        xlabel('East position, m')
        ylabel('North position, m')
        grid on
    end

    quality_criteria(i,:) = [   L_Va
                                L_h
                                L_chi
                                T_stl_Va
                                T_stl_h
                                T_stl_chi
                                E_t
                                E_ear ];

    disp(strcat('Finished simulation #',num2str(i)));
end
toc

disp('Control quality metrics:');
Criterion = ["L_Va";"L_h";"L_chi";"T_stl_Va";"T_stl_h";"T_stl_chi";"E_t";"E_ear"];
if N_sim==1
    Value = quality_criteria';
    tab = table(Criterion,Value);
else
    Max = max(quality_criteria)';
    Mean = mean(quality_criteria)';
    Std_dev = std(quality_criteria)';
    CI95T = Mean + 1.96 * Std_dev;
    CI95B = Mean - 1.96 * Std_dev;
    tab = table(Criterion,Max,Mean,Std_dev,CI95T,CI95B);
end
disp(tab);

clear Criterion Value Max Std_dev CI95T CI95B disp_ne_track
