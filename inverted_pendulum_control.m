%%
% Digital Control Systems - Inverted Pendulum Control
% System parameters
M = 1;      % Mass (kg)
L = 0.842;  % Pendulum length (m)
F = 1;      % Friction coefficient
g = 9.8093; % Gravity acceleration (m/s^2)

% State variables:
% x = [s; ds; phi; dphi]
% where:
% s = cart position
% ds = cart velocity
% phi = pendulum angle
% dphi = pendulum angular velocity
% u = control input (force applied to cart)

%% A: Continuous-time system modeling
% Linearized system matrices
A = [0 1 0 0;
     0 -F/M 0 0;
     0 0 0 1;
     0 F/(M*L) g/L 0];
B = [0; 1/M; 0; -1/(M*L)];
C = [1 0 0 0;
     0 0 1 0];
D = [0; 0];

% State-space representation
sys = ss(A, B, C, D);

%% B1-B2: Continuous-time state feedback controller design
% Choose desired closed-loop poles
desired_poles = [-1+1i; -1-1i; -0.6+1i; -0.6-1i];
K = place(A, B, desired_poles);

% Closed-loop system
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);

% Simulation initial conditions
x0 = [0; 1; 0; 0]; % Initial state
t = 0:0.01:10;     % Time range

% Simulate closed-loop system
[y, t, x] = initial(sys_cl, x0, t);

% Calculate control input
u = -K*x';

% Plot results
figure;
subplot(2,1,1);
plot(t, y);
title('B1-B2: Continuous-Time Linear System Response');
xlabel('Time (s)');
ylabel('Output');
legend('s(t)', '\phi(t)');

subplot(2,1,2);
plot(t, u);
title('Control Input');
xlabel('Time (s)');
ylabel('u(t)');

%% B3: Simulate the non-linear system with the same controller K
tspan = [0 10];

% Simulation using ODE45
[t, x] = ode45(@(t, x) inverted_pendulum_dynamics(t, x, K), tspan, x0);

% Calculate control input for nonlinear system
u_nonlinear = zeros(length(t), 1);
for i = 1:length(t)
    u_nonlinear(i) = -K * x(i,:)';
end

% Plot results
figure;
subplot(3, 1, 1);
plot(t, x(:, 1));
title('B3: Nonlinear System - Position s(t)');
xlabel('Time (s)');
ylabel('Position (m)');

subplot(3, 1, 2);
plot(t, x(:, 3));
title('Angle \phi(t)');
xlabel('Time (s)');
ylabel('Angle (rad)');

subplot(3, 1, 3);
plot(t, u_nonlinear);
title('Control Input u(t)');
xlabel('Time (s)');
ylabel('Force (N)');

%% B4-B5: Effect of sampling time on system stability
Ts_stable = 0.1;   % Stable sampling time
Ts_unstable = 1.0; % Potentially unstable sampling time

% Discretize the closed-loop system with different sampling times
sysd_stable = c2d(sys_cl, Ts_stable);
sysd_unstable = c2d(sys_cl, Ts_unstable);

% Simulate discrete-time systems
t_stable = 0:Ts_stable:10;
t_unstable = 0:Ts_unstable:10;
[y_stable, t_stable] = initial(sysd_stable, x0, t_stable);
[y_unstable, t_unstable] = initial(sysd_unstable, x0, t_unstable);

% Plot results
figure;
subplot(2,1,1);
stairs(t_stable, y_stable);
title('B4: Discrete-Time System with T_s = 0.1s');
xlabel('Time (s)');
ylabel('Output');
legend('s(t)', '\phi(t)');

subplot(2,1,2);
stairs(t_unstable, y_unstable);
title('B5: Discrete-Time System with T_s = 1.0s');
xlabel('Time (s)');
ylabel('Output');
legend('s(t)', '\phi(t)');

%% B6: Calculate discrete-time state matrices
% Discretize the open-loop system with stable sampling time
sysd = c2d(ss(A, B, C, D), Ts_stable);
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

% Display discrete-time matrices
disp('B6: Discrete-time state matrices:');
disp('Ad = ');
disp(Ad);
disp('Bd = ');
disp(Bd);

%% B7: Design discrete-time state feedback controller
% Choose desired discrete-time poles
% Map continuous poles to discrete domain
desired_poles_d = exp(Ts_stable * desired_poles);
Kd = place(Ad, Bd, desired_poles_d);

% Display discrete controller gain
disp('B7: Discrete-time controller gain:');
disp('Kd = ');
disp(Kd);

%% B8: Correct implementation of discrete-time control for nonlinear system
% Simulation parameters
T_total = 10;
N_steps = T_total / Ts_stable;
x_discrete = zeros(4, N_steps+1);
x_discrete(:,1) = x0;
u_discrete = zeros(1, N_steps);

% Discrete-time simulation using zero-order hold
for k = 1:N_steps
    % Calculate control input
    u_discrete(k) = -Kd * x_discrete(:, k);
    
    % Simulate one step of continuous system with constant input
    [~, x_temp] = ode45(@(t, x) inverted_pendulum_dynamics_zoh(t, x, u_discrete(k)), ...
                        [0 Ts_stable], x_discrete(:, k));
    
    % Update state for next step
    x_discrete(:, k+1) = x_temp(end, :)';
end

% Plot results
t_discrete = 0:Ts_stable:T_total;
figure;
subplot(2,1,1);
plot(t_discrete, x_discrete(1,:), 'b', t_discrete, x_discrete(3,:), 'r');
title('B8: Discrete-Time Control of Nonlinear System');
xlabel('Time (s)');
ylabel('State');
legend('s(t)', '\phi(t)');

subplot(2,1,2);
stairs(t_discrete(1:end-1), u_discrete);
title('Discrete Control Input u(k)');
xlabel('Time (s)');
ylabel('u(k)');

%% B9-B10: LQR optimal control design
% Initial condition for LQR test
x0_lqr = [-0.2; 0; 0.1; 0];

% LQR weights
Q = diag([100, 1, 100, 1]); % Higher penalty on position and angle
R = 1;                      % Control input weight

% Calculate LQR optimal gain
K_lqr = dlqr(Ad, Bd, Q, R);

% Display LQR gain
disp('B9: LQR controller gain:');
disp('K_lqr = ');
disp(K_lqr);

% Simulation parameters
x_lqr = zeros(4, N_steps+1);
x_lqr(:,1) = x0_lqr;
u_lqr = zeros(1, N_steps);

% Discrete-time simulation with LQR control
for k = 1:N_steps
    % Calculate LQR control input
    u_lqr(k) = -K_lqr * x_lqr(:, k);
    
    % Simulate one step of continuous system with constant input
    [~, x_temp] = ode45(@(t, x) inverted_pendulum_dynamics_zoh(t, x, u_lqr(k)), ...
                        [0 Ts_stable], x_lqr(:, k));
    
    % Update state for next step
    x_lqr(:, k+1) = x_temp(end, :)';
end

% Plot results
figure;
subplot(2,1,1);
plot(t_discrete, x_lqr(1,:), 'b', t_discrete, x_lqr(3,:), 'r');
title('B10: LQR Control of Nonlinear System');
xlabel('Time (s)');
ylabel('State');
legend('s(t)', '\phi(t)');

subplot(2,1,2);
stairs(t_discrete(1:end-1), u_lqr);
title('LQR Control Input u(k)');
xlabel('Time (s)');
ylabel('u(k)');

%% B11: Compare pole placement and LQR controllers
figure;
subplot(2,2,1);
plot(t_discrete, x_discrete(1,:));
title('Pole Placement: Position s(t)');
xlabel('Time (s)');
ylabel('Position (m)');

subplot(2,2,2);
plot(t_discrete, x_discrete(3,:));
title('Pole Placement: Angle \phi(t)');
xlabel('Time (s)');
ylabel('Angle (rad)');

subplot(2,2,3);
plot(t_discrete, x_lqr(1,:));
title('LQR: Position s(t)');
xlabel('Time (s)');
ylabel('Position (m)');

subplot(2,2,4);
plot(t_discrete, x_lqr(3,:));
title('LQR: Angle \phi(t)');
xlabel('Time (s)');
ylabel('Angle (rad)');

%% C: System analysis - Reachability and Observability
% Reachability matrix
reachability_matrix = ctrb(A, B);

% Check reachability
if rank(reachability_matrix) == size(A, 1)
    disp('C1: The pair (A, B) is reachable.');
else
    disp('C1: The pair (A, B) is not reachable.');
end

% Observability matrix
observability_matrix = obsv(A, C);

% Check observability
if rank(observability_matrix) == size(A, 1)
    disp('C2: The pair (A, C) is observable.');
else
    disp('C2: The pair (A, C) is not observable.');
end

% Discrete-time reachability and observability
reachability_matrix_d = ctrb(Ad, Bd);
if rank(reachability_matrix_d) == size(Ad, 1)
    disp('C3: The discrete-time pair (Ad, Bd) is reachable.');
else
    disp('C3: The discrete-time pair (Ad, Bd) is not reachable.');
end

observability_matrix_d = obsv(Ad, Cd);
if rank(observability_matrix_d) == size(Ad, 1)
    disp('C4: The discrete-time pair (Ad, Cd) is observable.');
else
    disp('C4: The discrete-time pair (Ad, Cd) is not observable.');
end

%% Functions

% Continuous-time nonlinear inverted pendulum dynamics
function dx = inverted_pendulum_dynamics(t, x, K)
    % System parameters
    M = 1;
    L = 0.842;
    F = 1;
    g = 9.8093;

    % State variables
    s = x(1);
    ds = x(2);
    phi = x(3);
    dphi = x(4);

    % Control input
    u = -K * x;

    % State equations
    dx = [ds;
          (u - F*ds) / M;
          dphi;
          (g*sin(phi))/L - (1/L)*((u - F*ds)/M)*cos(phi)];
end

% Nonlinear inverted pendulum dynamics with zero-order hold input
function dx = inverted_pendulum_dynamics_zoh(t, x, u)
    % System parameters
    M = 1;
    L = 0.842;
    F = 1;
    g = 9.8093;

    % State variables
    s = x(1);
    ds = x(2);
    phi = x(3);
    dphi = x(4);

    % State equations with constant input u
    dx = [ds;
          (u - F*ds) / M;
          dphi;
          (g*sin(phi))/L - (1/L)*((u - F*ds)/M)*cos(phi)];
end 