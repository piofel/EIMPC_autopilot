function [Et,Eear] = control_effort_criterion_airplane_01(t,u)
    % t = time samples of control signal
    % u = control signal samples
    % each control sample is [delta_e delta_a delta_r delta_t]
    N = length(t);
    delta_t = u(:,4);
    delta_ear = u(:,1:3);
    Et = 0;
    Eear = 0;
    for i=2:(N-1)
        dt = t(i+1) - t(i);
        Et = Et + dt*delta_t(i)^2;
        Eear = Eear + dt*(delta_ear(i,:)*delta_ear(i,:)');
    end
    T = t(length(t)) - t(2) + t(1);
    Et = Et/T;
    Eear = Eear/T;
end