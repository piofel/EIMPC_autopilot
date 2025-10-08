function Tstl = settling_time(t,tc,y,yc,e_tol,t_tol)
    % t = time samples of controlled signal
    % tc = time samples of commanded (setpoint) signal
    % y = samples of controlled signal
    % yc = samples of commanded (setpoint) signal
    % e_tol = controlled signal tolerance
    % t_tol = minimum time for the signal to be in tolerance
    N = length(t);
    timer = 0;
    Tstl = Inf;
    tol_entered = false;
    for i=1:(N-1)
        if t(i) ~= tc(i)
            error('Time instants mismatch!');
        end
        dt = t(i+1) - t(i);
        e = yc(i) - y(i);
        if abs(e) <= e_tol
            if ~tol_entered
                tol_entered = true;
                Tstl_candidate = t(i);
            end
            if timer >= t_tol
                Tstl = Tstl_candidate;
                break
            else
                timer = timer + dt;
            end
        else
            timer = 0;
            tol_entered = false;
        end
    end
end