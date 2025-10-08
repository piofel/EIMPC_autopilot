An UAV simulator with a model predictive control (MPC) autopilot
================================================================

Use `main_sim_eimpc.m` to run the main EIMPC simulation. Make sure that the scopes in the `Visualization` block in the `sim_eimpc.slx` Simulink model are opened.

Run `main_sim_pid.m` for the PID-based autopilot simulation. Make sure to open the scopes in the `uav_sim_pid.slx` Simulink model.

Requires Simulink 2024a or newer.

UAV parameters can be found in `make_uav_parameters_eimpc.m`
