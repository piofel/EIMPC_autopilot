% State-space MAV models

% Global variables used:
% x_trim is the trimmed state,
% u_trim is the trimmed input

% Continuous models
[A_c,B_c,C_c,D_c] = linmod('uav_trim_va_h_chi_sim',x_trim,u_trim);

E1 = [...
0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;...
0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;...
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;...
0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;...
0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;...
];
E2 = [...
0, 1, 0, 0;...
0, 0, 1, 0;...
];
% x_m_lat = [v p r phi psi]'
A_c_lat = E1 * A_c * E1';
% u_lat = [delta_a delta_r]'
B_c_lat = E1 * B_c * E2';

E3 = [...
0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;...
0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;...
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;...
0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;...
0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
];
E4 = [...
1, 0, 0, 0;...
0, 0, 0, 1;...
];
% x_m_lon = [u w q theta pd]'
A_c_lon = E3 * A_c * E3';
% u_lon = [delta_e delta_t]'
B_c_lon = E3 * B_c * E4';

% y_lat = chi
C_m_lat = [C_c(3,5) C_c(3,10) C_c(3,12) C_c(3,7) C_c(3,9)];
% y_lon = [Va h]'
C_m_lon = [C_c(1:2,4) C_c(1:2,6) C_c(1:2,11) C_c(1:2,8)  C_c(1:2,3)];

clearvars E1 E2 E3 E4 gamma A_c B_c C_c D_c

% Discrete models
A_m_lat = state_matrix_continuous_to_discrete(A_c_lat, Ts);
A_m_lon = state_matrix_continuous_to_discrete(A_c_lon, Ts);
B_m_lat = input_matrix_continuous_to_discrete(A_c_lat, B_c_lat, Ts);
B_m_lon = input_matrix_continuous_to_discrete(A_c_lon, B_c_lon, Ts);

clearvars A_c_lat A_c_lon B_c_lat B_c_lon

% Augmented design models
[A_lat, B_lat, C_lat] = augmented_model(A_m_lat,B_m_lat,C_m_lat);
[A_lon, B_lon, C_lon] = augmented_model(A_m_lon,B_m_lon,C_m_lon);

clearvars A_m_lat B_m_lat C_m_lat A_m_lon B_m_lon C_m_lon

% Local functions

function [A, B, C] = augmented_model(Ad,Bd,Cd)
    % Algorithm described in Wang p. 6    
    [m1, ~] = size(Cd);
    [n1, n_in] = size(Bd);
    A = eye(n1+m1, n1+m1);
    A(1:n1, 1:n1) = Ad;
    A(n1+1:n1+m1, 1:n1) = Cd*Ad;
    B = zeros(n1+m1, n_in);
    B(1:n1, :) = Bd;
    B(n1+1:n1+m1, :) = Cd*Bd;
    C = zeros(m1, n1+m1);
    C(:, n1+1:n1+m1) = eye(m1, m1);
end

function A = state_matrix_continuous_to_discrete(A_c, Ts)
    A = expm(Ts*A_c);
end

function B = input_matrix_continuous_to_discrete(A_c, B_c, Ts)
    order = 60;  % Nd
    n = size(A_c,1);
    Q = eye(n);
    for i=2:order
        Q = Q + (Ts*A_c)^(i-1) / factorial(i);
    end
    Q = Ts*Q;
    B = Q*B_c;
end