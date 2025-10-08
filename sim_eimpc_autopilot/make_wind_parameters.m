windStatic = [-3;2;0]; % zeros(3,1); % [20;-8;2]; % 
windsigma = [3;3;3]; % zeros(3,1); % 
windL = [533 533 533]';
gustsNoisePower = 0.1;
gustsNoiseCorrelationTime = 0.5;
gusts_seeds = [ 1:N_sim;
                1+N_sim:N_sim+N_sim;
                1+N_sim+N_sim:N_sim+N_sim+N_sim];