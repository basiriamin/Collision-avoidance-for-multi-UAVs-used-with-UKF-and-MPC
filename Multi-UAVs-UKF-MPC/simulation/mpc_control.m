%%  ALLAH

function [t,x,u] = mpc_control(runningcosts, terminalcosts, ...
              constraints, terminalconstraints, ...
              linearconstraints, system, ...
              mpciterations, N, T, tmeasure, xmeasure, u0)

warning off all
t = [];
x = [];
u = [];
% Start of the NMPC iteration
mpciter = 0;
while(mpciter < mpciterations)
    disp(['iter = ',num2str(mpciter+1),' / ',num2str(mpciterations)])
    % Step (1) of the NMPC algorithm:
    %   Obtain new initial value
    [t0, x0] = measureInitialValue ( tmeasure, xmeasure );
    % Step (2) of the NMPC algorithm:
    %   Solve the optimal control problem
    u_new = solveOptimalControlProblem (runningcosts, terminalcosts, constraints, ...
        terminalconstraints, linearconstraints, system, N, t0, x0, u0, T);
    %   Store closed loop data
    t = [ t; tmeasure ];
    x = [ x; xmeasure ];
    u = [ u, u_new(:,1) ];
    %   Prepare restart
    u0 = shiftHorizon(u_new);
    %   Apply control to process
    [tmeasure, xmeasure] = applyControl(system, T, t0, x0, u_new);
    mpciter = mpciter+1;
end
end

function [t0, x0] = measureInitialValue ( tmeasure, xmeasure )
    t0 = tmeasure;
    x0 = xmeasure;
end

function [tapplied, xapplied] = applyControl(system, T, t0, x0, u)
    xapplied = dynamic(system, T, t0, x0, u(:,1));
    tapplied = t0+T;
end

function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end

function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, constraints, terminalconstraints, ...
    linearconstraints, system, N, t0, x0, u0, T)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u0);

    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    for k=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+k*T,x(k,:),u0(:,k));
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end

    % Solve optimization problem
    options = optimoptions('fmincon','Display','iter');
    fun = @(u) costfunction(runningcosts, terminalcosts, system, N, T, t0, x0, u);
    nonlcon = @(u) nonlinearconstraints(constraints, terminalconstraints, system, N, T, t0, x0, u);
    [u, V, exitflag, output] = fmincon(fun, u0, A, b, Aeq, beq, lb, ub, nonlcon,options);
end

function cost = costfunction(runningcosts, terminalcosts, system, N, T, t0, x0, u)
    cost = 0;
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u);
    for k=1:N
        cost = cost+runningcosts(t0+k*T, x(k,:), u(:,k));
    end
    cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:));
end

function [c,ceq] = nonlinearconstraints(constraints, ...
    terminalconstraints, system, N, T, t0, x0, u)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u);
    c = [];
    ceq = [];
    for k=1:N
        [cnew, ceqnew] = constraints(t0+k*T,x(k,:),u(:,k));
        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    [cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:));
    c = [c cnew];
    ceq = [ceq ceqnew];
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u)
    x(1,:) = x0;
    for k=1:N
        x(k+1,:) = dynamic(system, T, t0, x(k,:), u(:,k));
    end
end

function [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, x0, u)
    x = system(t0, x0, u, T);
    x_intermediate = [x0; x];
    t_intermediate = [t0, t0+T];
end