clc
close all
clearvars

Ts = 0.1;  % sampling time
stopTime = 100;
N_sim = 1;
disp_ne_track = 1;
make_C_m_alpha_test = 0;
make_C_ell_delta_a_test = 0;
make_mass_test = 0;

mdl = 'sim_eimpc';
open_system(mdl)
simIn = Simulink.SimulationInput(mdl);
simIn = setModelParameter(simIn,"StopTime",num2str(stopTime));

u_log_index =   1;
h_log_index =   4;
chi_log_index = 5;
V_log_index =   6;

quality_criteria = NaN(N_sim,8);

make_initial_state
make_nominal_flight_parameters_eimpc
uavParams = make_uav_parameters_eimpc();
make_wind_parameters

tic
for i=1:N_sim
    if make_C_m_alpha_test
        uavParams(28) = -0.38 * 5;   % for C_m_alpha robustness test
    end
    if make_C_ell_delta_a_test
        C_ell_delta_a = 0.08 + randn*0.02;
        uavParams(52) = C_ell_delta_a;  % for C_ell_delta_a robustness test
    end
    if make_mass_test
        mass = 13.5 + randn;
        Jx   = 0.824 + 0.25*randn;
        Jy   = 1.135 + 0.25*randn;
        Jz   = 1.759 + 0.25*randn;
        Jxz  = 0.120 + 0.025*randn;
        uavParams(10) = mass;
        uavParams(11) = Jx;
        uavParams(12) = Jy;
        uavParams(13) = Jz;
        uavParams(14) = Jxz;
    end

    uav_trim_va_h_chi
    compute_uav_ss_model_augmented
    make_autopilot_parameters_eimpc

    if make_C_m_alpha_test || make_C_ell_delta_a_test || make_mass_test
        uavParams = make_uav_parameters_eimpc();
    end

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
        pn_log_index =  2;
        pe_log_index =  3;
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

clear y yc t i h_log_index V_log_index chi_log_index pn_log_index
clear pe_log_index mdl simIn u pn_samples pe_samples u_log_index
clear gusts_seeds_str
clear Criterion Value Max Std_dev CI95T CI95B
