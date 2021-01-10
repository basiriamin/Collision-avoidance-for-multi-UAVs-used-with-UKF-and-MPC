%%  ALLAH

function [t,x,u] = uav_model_ekf
%%  global parameters
global T mpciterations

%%  inputs
mpciterations = 20;
N             = 11;
T             = 6;
tmeasure      = 0.0;
xmeasure      = [-1500,50,0,-1500,270,0,-1500,400,0];
u0            = 0*ones(9,N);

%%  solve mpc problem
[t,x,u] = mpc_control(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system, ...
         mpciterations, N, T, tmeasure, xmeasure, u0);
end

function dx = system(t,x,u,T)
global x_pre
x_pre = x;
V = 100;
y_psi_1 = atan2(x(2),x(1));
y_psi_2 = atan2(x(5),x(4));
y_psi_3 = atan2(x(8),x(7));
dx(1) = V*cos(y_psi_1) + u(1);
dx(2) = V*sin(y_psi_1) + u(2);
dx(3) = u(3);
dx(4) = V*cos(y_psi_2) + u(4);
dx(5) = V*sin(y_psi_2) + u(5);
dx(6) = u(6);
dx(7) = V*cos(y_psi_3) + u(7);
dx(8) = V*sin(y_psi_3) + u(8);
dx(9) = u(9);
end

function cost = runningcosts(t, x, u)
%   inputs
    h_pi = 10;
    zeta_ob = 400;
    zeta_t = 1;
    a = 0.5;
    b = 0.5;
    n = 3;
    Rs = 50;
    Gamma = eye(9)/1e6;
    %   call obstacle function
    [x_obs,y_obs,z_obs] = obstacle_function(t);
    %   call target function
    [xt,yt,zt] = target_function;
    %   estimate states with ekf
    q = 0.1;
    r = 0.1;
    Q = q^2*eye(9);
    R = r^2;
    f = @(x)(x);
    h = @(x)(x(1));
    z = h(ones(9,1));
    P = eye(9);
    xhat = ekf(f,reshape(x,[],1),P,h,z,Q,R);
    %   distance to the obstacles
    d1_x = xhat(1) - x_obs;
    d1_y = xhat(2) - y_obs;
    d1_z = xhat(3) - z_obs;
    d2_x = xhat(4) - x_obs;
    d2_y = xhat(5) - y_obs;
    d2_z = xhat(6) - z_obs;
    d3_x = xhat(7) - x_obs;
    d3_y = xhat(8) - y_obs;
    d3_z = xhat(9) - z_obs;
    d1 = sqrt(d1_x.^2+d1_y.^2+d1_z.^2);
    d2 = sqrt(d2_x.^2+d2_y.^2+d2_z.^2);
    d3 = sqrt(d3_x.^2+d3_y.^2+d3_z.^2);
    do(1) = min(d1);
    do(2) = min(d2);
    do(3) = min(d3);
    dt(1) = sqrt((x(1)-xt)^2+(x(2)-yt)^2+(x(3)-zt)^2);
    dt(2) = sqrt((x(4)-xt)^2+(x(5)-yt)^2+(x(6)-zt)^2);
    dt(3) = sqrt((x(7)-xt)^2+(x(8)-yt)^2+(x(9)-zt)^2);
    %   J0
    Jo = 0;
    for tau = 1:h_pi
        for i = 1:n
            if do(i) < Rs
                pe = 1;
            else
                pe = 0;
            end
        end
        Jo = Jo + pe;
    end
    Jo = zeta_ob*Jo;
    %   Jt
    Jt1 = 0;
    for i = 1:n
        for tau = 1:h_pi
            Jt1 = Jt1 + zeta_t*dt(i);
        end
    end
    Jt2 = 0;
    for i = 1:n
        for j = 1:n
            if j ~= i
                for tau = 1:h_pi
                    Jt2 = Jt2 + abs(dt(i) - dt(j));
                end
            end
        end
    end
    Jt = a*Jt1 + b*Jt2;
    %   Ju
    Ju = u'*Gamma*u;
    %   total cost
    cost = Jo + Jt + Ju;
end

function cost = terminalcosts(t, x)
    cost = 0.0;
end

function [c,ceq] = constraints(t, x, u)
    global T x_pre
    dx = system(t,x,u,T);
    c = [];
    ceq = x - x_pre - dx*T;
end

function [c,ceq] = terminalconstraints(t, x)
    c = [];
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [];
    ub  = [];
end