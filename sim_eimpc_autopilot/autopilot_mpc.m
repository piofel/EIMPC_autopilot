function [y,x_m_lat,x_m_lon] = autopilot_mpc(xhat,x_m_lat_prev,x_m_lon_prev,...
    delta_prev,commandSignals,A_lat,B_lat,C_lat,A_lon,B_lon,C_lon,autopilotParams)

    [x_lat,x_lon,x_m_lat,x_m_lon] = estimated_to_augmented_states(xhat,x_m_lat_prev,x_m_lon_prev);

    Va_c = commandSignals(1);
    h_c = commandSignals(2);
    chi_c = commandSignals(3);
    beta_c = 0;
    command_lat = chi_c;
    command_lon = [Va_c h_c]';

    delta_lat_prev = delta_prev(2:3);
    delta_lon_prev = [delta_prev(1); delta_prev(4)];
    Deltadelta_lat = ...
        lateral_autopilot(x_lat,delta_lat_prev,command_lat,A_lat,B_lat,...
        C_lat,autopilotParams);
    Deltadelta_lon = ...
        longitudinal_autopilot(x_lon,delta_lon_prev,command_lon,A_lon,...
        B_lon,C_lon,autopilotParams);
    delta = NaN(4,1);
    delta(1) = delta_prev(1) + Deltadelta_lon(1);
    delta(2:3) = delta_prev(2:3) + Deltadelta_lat;
    delta(4) = delta_prev(4) + Deltadelta_lon(2);

    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        beta_c;...               % beta
        0;...                    % phi
        0;...                    % theta
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];

    y = [delta; x_command];
end

function Deltadelta_lat = lateral_autopilot(x_lat,delta_lat_prev,command_lat,...
        A_lat,B_lat,C_lat,autopilotParams)
    control_weights_lat = ...
        [autopilotParams(38) autopilotParams(39)];  % u_lat_c = [delta_a delta_r]'
    delta_a_max = autopilotParams(23);
    delta_r_max = autopilotParams(24);
    u_max_lat = [delta_a_max; delta_r_max];
    u_min_lat = -u_max_lat;
    Np = autopilotParams(31);
    Nc = autopilotParams(32);
    optimizer_id = autopilotParams(42);
    Deltadelta_lat = mpc_linear_01(x_lat,delta_lat_prev,command_lat,...
        control_weights_lat,u_min_lat,u_max_lat,A_lat,B_lat,C_lat,...
        Np,Nc,optimizer_id);
end

function Deltadelta_lon = longitudinal_autopilot(x_lon,delta_lon_prev,...
        command_lon, A_lon,B_lon,C_lon,autopilotParams)
    control_weights_lon = ...
        [autopilotParams(37) autopilotParams(40)];  % u_lon_c = [delta_e delta_t]'
    delta_e_max = autopilotParams(22);
    delta_t_max = autopilotParams(25);
    u_max_lon = [delta_e_max; delta_t_max];
    u_min_lon = [-delta_e_max; 0];
    Np = autopilotParams(33);
    Nc = autopilotParams(34);
    optimizer_id = autopilotParams(42);
    Deltadelta_lon = mpc_linear_01(x_lon,delta_lon_prev,command_lon,...
        control_weights_lon,u_min_lon,u_max_lon,A_lon,B_lon,C_lon,...
        Np,Nc,optimizer_id);
end

function [x_lat,x_lon,x_m_lat,x_m_lon] = estimated_to_augmented_states(xhat,x_m_lat_prev,x_m_lon_prev)
    h        = xhat(3);  % altitude
    Va       = xhat(4);  % airspeed
    chi      = xhat(9);  % course angle
    [x_m_lat,x_m_lon] = estimated_to_lat_lon_states(xhat);
    x_lat = ...
    [
        x_m_lat - x_m_lat_prev;
        chi
    ];
    x_lon = ...
    [
        x_m_lon - x_m_lon_prev;
        [Va h]'
    ];
end