function L = tracking_quality_criterion_01(t,tc,y,yc)
    % t = time samples of controlled signal
    % tc = time samples of commanded (setpoint) signal
    % y = samples of controlled signal
    % yc = samples of commanded (setpoint) signal
    N = length(t);
    L = 0;
    for i=2:(N-1)
        if t(i) ~= tc(i)
            error('Time instants mismatch!');
        end
        dt = t(i+1) - t(i);
        e = yc(i) - y(i);
        L = L + dt*e^2;
    end
    T = t(length(t)) - t(2) + t(1);
    L = L/T;
end