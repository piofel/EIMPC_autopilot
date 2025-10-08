function Deltau = mpc_linear_01(X_hat,u_prev,r,control_weights,u_min,u_max,A,B,C,...
        Np,Nc,optimizer_id)
    % r - command vector / reference
    [PhiT_Phi, PhiT_F, PhiT_BarRs] = mpcgain(A,B,C,Np,Nc);
    [~, m] = size(B); % Number of inputs
    BarR = zeros(m*Nc, m*Nc);
    BarR(1:m, 1:m) = diag(control_weights);
    for i = 2:Nc
        BarR((i-1)*m+1:i*m, (i-1)*m+1:i*m) = diag(control_weights);
    end
    H = PhiT_Phi + BarR; % Hessian before inverting
    nf = PhiT_BarRs*r - PhiT_F*X_hat;  % -f
    f = -nf;
    switch optimizer_id
        case 1  % unconstrained solution
            DeltaU = H \ nf;
        case 2  % Hildreth’s algorithm
            [M1, N1] = control_constraints(u_min,u_max,Nc,m,u_prev);
            DeltaU = qp_hildreth(H, f, M1, N1);
        case 3  % Matlab's quadprog
            [M1, N1] = control_constraints(u_min,u_max,Nc,m,u_prev);
            DeltaU = quadprog(H, f, M1, N1);
        otherwise
            error("Bad optimizer ID.");
    end
    Deltau = first_control_increment(Nc, m, DeltaU);
end

function [M1, N1] = control_constraints(u_min, u_max, Nc, m, u_prev)
    % u_max - control upper constraint in the form of column vector
    % u_min - control lower constraint in the form of column vector
    M1dw = zeros(m*Nc, m*Nc);
    M1dw(1:m, 1:m) = eye(m);
    for i = 2:Nc
        M1dw((i-1)*m+1:i*m, 1:m) = eye(m);
    end
    for j = 2:Nc
        for k = 2:Nc
            if j >= k
                M1dw((j-1)*m+1:j*m, (k-1)*m+1:k*m) = eye(m);
            end
        end
    end
    M1up = -M1dw;
    M1 = [M1up; M1dw];
    C1 = eye(m);
    for i = 2:Nc
        C1((i-1)*m+1:i*m, 1:m) = eye(m);
    end  
    N1 = [-C1*u_min + C1*u_prev;
           C1*u_max - C1*u_prev];
end

function Deltau = first_control_increment(Nc, m, DeltaU)
    Deltau = zeros(m, m*Nc);
    Deltau(1:m, 1:m) = eye(m);
    for i = 2:Nc
        Deltau(:, (i-1)*m+1:i*m) = zeros(m);
    end
    Deltau = Deltau * DeltaU;
end

function [PhiT_Phi, PhiT_F, PhiT_BarRs] = mpcgain(A, B, C, Np, Nc)
    % A, B, C are augmented discrete state-space model
    % Algorithm described in Wang p. 13
    [q, ~] = size(C); % Number of outputs
    [~, m] = size(B); % Number of inputs
    h(1:q, :) = C;
    F(1:q, :) = C*A;
    for kk = 2:Np
        h((kk-1)*q+1:kk*q, :) = h((kk-2)*q+1:(kk-1)*q, :) * A;
        F((kk-1)*q+1:kk*q, :) = F((kk-2)*q+1:(kk-1)*q, :) * A;
    end
    v = h*B;
    Phi = zeros(q*Np, m*Nc);
    Phi(:, 1:m) = v;
    for i = 2:Nc
        Phi(:, (i-1)*m+1:i*m) = [zeros((i-1)*q, m); v(1:(Np-i+1)*q, 1:m)]; % Toeplitz matrix
    end
    BarRs(1:q, :) = eye(q);
    for kkk = 2:Np
        BarRs((kkk-1)*q+1:kkk*q, :) = eye(q);
    end
    PhiT_Phi = Phi' * Phi;
    PhiT_F = Phi' * F;
    PhiT_BarRs = Phi' * BarRs;
end

function eta = qp_hildreth(H, f, A_cons, b)    
    max_iterations = 1e4;
    tolerance = 1e-10;
    P = A_cons*(H\A_cons');
    d = (A_cons*(H\f) + b);
    [n, m] = size(d);
    x_ini = zeros(n, m);
    lambda = x_ini;
    for km = 1:max_iterations
        % Find the elements in the solution vector one by one
        % km could be larger if the Lagranger multiplier has a slow
        % convergence rate.
        lambda_p = lambda;
        for i = 1:n
            w = P(i,:) * lambda-P(i,i) * lambda(i,1);
            w = w + d(i,1);
            la = -w/P(i,i);
            lambda(i,1) = max(0,la);
        end
        al = (lambda-lambda_p)' * (lambda-lambda_p);
        if (al < tolerance)
            break;
        end
        if km == max_iterations
            warning('Hildreth QP algorithm tolerance not satisfied.');
        end
    end
    eta = -H\f - H\A_cons'*lambda;
end