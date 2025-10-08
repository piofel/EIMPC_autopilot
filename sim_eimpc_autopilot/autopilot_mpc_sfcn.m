function autopilot_mpc_sfcn(block)
    setup(block);
%endfunction

function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  % Override the output port properties.
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';

  % Register the parameters.
  block.NumDialogPrms     = 7;
  block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable',...
      'Nontunable','Nontunable','Nontunable','Nontunable'};
  
  % Set up the continuous states.
  block.NumContStates = 0;

  block.InputPort(1).Dimensions        = 19;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(2).Dimensions        = 3;
  block.InputPort(2).DirectFeedthrough = false;

  block.OutputPort(1).Dimensions       = 16;

  % Register the sample times.
  %  [positive_num offset] : Discrete sample time
  Ts = block.DialogPrm(1).Data(20);
  block.SampleTimes = [Ts 0];

  block.SimStateCompliance = 'DefaultSimState';

  % Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Update',                  @Update);

%endfunction

function DoPostPropSetup(block)
  block.NumDworks = 6;
  
  block.Dwork(1).Name            = 'step_no';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 7;      % uint32
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

  block.Dwork(2).Name            = 'state_estimator_ready';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 8;      % boolean
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;

  block.Dwork(3).Name            = 'x_lat_m';
  block.Dwork(3).Dimensions      = 5;
  block.Dwork(3).DatatypeID      = 0;  % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;

  block.Dwork(4).Name            = 'x_lon_m';
  block.Dwork(4).Dimensions      = 5;
  block.Dwork(4).DatatypeID      = 0;  % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;

  block.Dwork(5).Name            = 'states_initialized';
  block.Dwork(5).Dimensions      = 1;
  block.Dwork(5).DatatypeID      = 8;      % boolean
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;

  block.Dwork(6).Name            = 'delta';
  block.Dwork(6).Dimensions      = 4;
  block.Dwork(6).DatatypeID      = 0;  % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;

%endfunction

function InitConditions(block)

    autopilotParams = block.DialogPrm(1).Data;
    u_trim(1) = autopilotParams(28);  % elevator trim value
    u_trim(2) = autopilotParams(29);  % aileron trim value
    u_trim(3) = autopilotParams(30);  % rudder trim value
    u_trim(4) = autopilotParams(19);  % throttle trim value

    block.Dwork(1).Data = uint32(0);  % step_no
    block.Dwork(2).Data = false;  % state_estimator_ready
    block.Dwork(3).Data = NaN(5,1);  % x_lat_m
    block.Dwork(4).Data = NaN(5,1);  % x_lon_m
    block.Dwork(5).Data = false;  % states_initialized
    block.Dwork(6).Data = u_trim;  % delta

%endfunction

function Output(block)
    delta_prev = block.Dwork(6).Data;
    commandSignals = block.InputPort(2).Data;
    stateEstimatorReady = block.Dwork(2).Data;
    if stateEstimatorReady
        xhat = block.InputPort(1).Data;
        states_initialized = block.Dwork(5).Data;
        if states_initialized == false
            [x_lat_m,x_lon_m] = estimated_to_lat_lon_states(xhat);
            block.Dwork(3).Data = x_lat_m;
            block.Dwork(4).Data = x_lon_m;
            block.Dwork(5).Data = true;  % states_initialized
        end
        x_lat_m_prev = block.Dwork(3).Data;
        x_lon_m_prev = block.Dwork(4).Data;
        A_lat = block.DialogPrm(2).Data;
        B_lat = block.DialogPrm(3).Data;
        C_lat = block.DialogPrm(4).Data;
        A_lon = block.DialogPrm(5).Data;
        B_lon = block.DialogPrm(6).Data;
        C_lon = block.DialogPrm(7).Data;
        autopilotParams = block.DialogPrm(1).Data;
        [y,x_lat_m,x_lon_m] = autopilot_mpc(xhat,x_lat_m_prev,x_lon_m_prev,...
            delta_prev,commandSignals,...
            A_lat,B_lat,C_lat,A_lon,B_lon,C_lon,autopilotParams);
        block.Dwork(3).Data = x_lat_m;
        block.Dwork(4).Data = x_lon_m;
    else
        x_command = zeros(12,1);
        x_command(4) = commandSignals(1);  % Va_c
        x_command(3) = commandSignals(2);  % h_c
        x_command(9) = commandSignals(3);  % chi_c
        y = [delta_prev; x_command];
    end
    block.Dwork(6).Data = y(1:4);  % delta
    block.OutputPort(1).Data = y;
%endfunction

function Update(block)
  
  block.Dwork(1).Data = block.Dwork(1).Data + 1;
  if block.Dwork(1).Data > 1
      block.Dwork(2).Data = true;
  end
  
%endfunction


