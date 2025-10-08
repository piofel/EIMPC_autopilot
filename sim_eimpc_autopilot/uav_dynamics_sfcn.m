function uav_dynamics_sfcn(block)
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

  % Other port setup
  block.InputPort(1).Dimensions        = 6;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(2).Dimensions        = 60;
  block.InputPort(2).DirectFeedthrough = false;
  block.OutputPort(1).Dimensions       = 12;

  % Register the parameters.
  block.NumDialogPrms     = 1;
  block.DialogPrmsTunable = {'Nontunable'};
  
  % Set up the continuous states.
  block.NumContStates = 12;

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  block.SampleTimes = [0 0];

  block.SimStateCompliance = 'DefaultSimState';

  % Register methods
 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  block.RegBlockMethod('Outputs', @Outputs);
  block.RegBlockMethod('Derivatives', @Derivatives);

%endfunction

function InitializeConditions(block)
    block.ContStates.Data = block.DialogPrm(1).Data;
%endfunction

function Outputs(block)
  block.OutputPort(1).Data = block.ContStates.Data;
%endfunction

function Derivatives(block)
    mavParameters = block.InputPort(2).Data;
    forces = block.InputPort(1).Data(1:3);
    moments = block.InputPort(1).Data(4:6);
    block.Derivatives.Data = ...
        uav_dynamics_gen(mavParameters, block.ContStates.Data, forces, moments);
        
%endfunction
