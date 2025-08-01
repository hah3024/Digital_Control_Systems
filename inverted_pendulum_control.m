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
title('Continuous-Time Linear System Response');
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
x0 = [0; 1; 0; 0];
% Simulation using ODE45
[t, x] = ode45(@(t, x) inverted_pendulum_dynamics(t, x, K), tspan, x0);

% Calculate control input for nonlinear system
u_nonlinear = zeros(length(t), 1);
for i = 1:length(t)
    u_nonlinear(i) = -K * x(i,:)';
end

% Plot results
figure;

% Subplot 1: Position s(t)
subplot(3, 1, 1); % 3 rows, 1 column, first subplot
plot(t, x(:, 1), 'b', 'LineWidth', 1.5); % Plot position s(t) in blue
title('Nonlinear System - Position s(t)');
xlabel('Time (s)');
ylabel('Position (m)');
grid on; % Add grid
legend('Position s(t)'); % Add legend

% Subplot 2: Angle phi(t)
subplot(3, 1, 2); % 3 rows, 1 column, second subplot
plot(t, x(:, 3), 'r', 'LineWidth', 1.5); % Plot angle phi(t) in red
title('Angle \phi(t)');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on; % Add grid
legend('Angle \phi(t)'); % Add legend

% Subplot 3: Control Input u(t)
subplot(3, 1, 3); % 3 rows, 1 column, third subplot
plot(t, u_nonlinear, 'k', 'LineWidth', 1.5); % Plot control input u(t) in black
title('Control Input u(t)');
xlabel('Time (s)');
ylabel('Force (N)');
grid on; % Add grid
legend('Control Input u(t)'); % Add legend
% figure;
% 
% % Subplot 1: Position s(t) and Angle phi(t)
% subplot(2, 1, 1); % 2 rows, 1 column, first subplot
% plot(t, x(:, 1), 'b', 'LineWidth', 1.5); % Plot position s(t) in blue
% hold on;
% plot(t, x(:, 3), 'r', 'LineWidth', 1.5); % Plot angle phi(t) in red
% title('B3: Nonlinear System - Position s(t) and Angle \phi(t)');
% xlabel('Time (s)');
% ylabel('Output');
% legend('Position s(t)', 'Angle \phi(t)');
% grid on;
% 
% % Subplot 2: Control Input u(t)
% subplot(2, 1, 2); % 2 rows, 1 column, second subplot
% plot(t, u_nonlinear, 'k', 'LineWidth', 1.5); % Plot control input u(t) in black
% title('Control Input u(t)');
% xlabel('Time (s)');
% ylabel('Force (N)');
% grid on;

%% B4: Effect of sampling time on linear system stability
% Define sampling times to analyze
sampling_times_B4 = [0.1, 0.5, 1];

% Create a new figure for each sampling time
for i = 1:length(sampling_times_B4)
    Ts = sampling_times_B4(i);
    
    % Discretize the closed-loop linear system
    sysd = c2d(sys_cl, Ts);
    
    % Simulate discrete-time linear system
    t_discrete = 0:Ts:10;
    [y_discrete, t_discrete, x_discrete] = initial(sysd, x0, t_discrete);
    
    % Calculate control input
    u_discrete = zeros(1, length(t_discrete)-1);
    for k = 1:length(t_discrete)-1
        u_discrete(k) = -K * x_discrete(k,:)';
    end
    
    % Create figure for this sampling time
    figure(5+i);
    
    % Plot position
    subplot(3,1,1);
    stairs(t_discrete, x_discrete(:,1), 'LineWidth', 1.5);
    title(['Linear System Position (s) - T_s = ', num2str(Ts), 's']);
    xlabel('Time (s)');
    ylabel('Position (m)');
    grid on;
    
    % Plot angle
    subplot(3,1,2);
    stairs(t_discrete, x_discrete(:,3), 'LineWidth', 1.5);
    title(['Linear System Angle (\phi) - T_s = ', num2str(Ts), 's']);
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    grid on;
    
    % Plot control input
    subplot(3,1,3);
    stairs(t_discrete(1:end-1), u_discrete, 'LineWidth', 1.5);
    title(['Linear System Control Input (u) - T_s = ', num2str(Ts), 's']);
    xlabel('Time (s)');
    ylabel('Force (N)');
    grid on;
    
    % Set overall title and save figure
    sgtitle(['Linear System Response - Sampling Time T_s = ', num2str(Ts), 's']);
    saveas(gcf, ['linear_sampling_', num2str(Ts*10), 's.png']);
end

%% B5: Effect of sampling time on nonlinear system performance
% Define sampling times to analyze
sampling_times = [0.1, 0.5, 1.0];

% Loop through each sampling time
for i = 1:length(sampling_times)
    Ts = sampling_times(i);
    
    % Discretize the system
    sysd = c2d(sys, Ts);
    Ad = sysd.A;
    Bd = sysd.B;
    
    % Design discrete controller (map continuous poles to discrete domain)
    desired_poles_d = exp(Ts * desired_poles);
    Kd = place(Ad, Bd, desired_poles_d);
    
    % Simulation parameters
    T_total = 10;
    N_steps = T_total / Ts;
    x_discrete = zeros(4, N_steps+1);
    x_discrete(:,1) = x0;
    u_discrete = zeros(1, N_steps);
    
    % Discrete-time simulation using zero-order hold
    for k = 1:N_steps
        % Calculate control input
        u_discrete(k) = -Kd * x_discrete(:, k);
        
        % Simulate one step of continuous system with constant input
        [~, x_temp] = ode45(@(t, x) inverted_pendulum_dynamics_zoh(t, x, u_discrete(k)), ...
                            [0 Ts], x_discrete(:, k));
        
        % Update state for next step
        x_discrete(:, k+1) = x_temp(end, :)';
    end
    
    % Time vector for plotting
    t_discrete = 0:Ts:T_total;
    
    % Create figure for this sampling time
    figure(i+1);
    
    % Plot position
    subplot(3,1,1);
    plot(t_discrete, x_discrete(1,:), 'LineWidth', 1.5);
    title(['Position (s) - T_s = ', num2str(Ts), 's']);
    xlabel('Time (s)');
    ylabel('Position (m)');
    grid on;
    
    % Plot angle
    subplot(3,1,2);
    plot(t_discrete, x_discrete(3,:), 'LineWidth', 1.5);
    title(['Pendulum Angle (\phi) - T_s = ', num2str(Ts), 's']);
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    grid on;
    
    % Plot control input
    subplot(3,1,3);
    stairs(t_discrete(1:end-1), u_discrete, 'LineWidth', 1.5);
    title(['Control Input (u) - T_s = ', num2str(Ts), 's']);
    xlabel('Time (s)');
    ylabel('Force (N)');
    grid on;
    % 
    % % Set overall title and save figure
    % sgtitle(['Discrete-Time Control Response - Sampling Time T_s = ', num2str(Ts), 's']);
    % saveas(gcf, ['sampling_', num2str(Ts*10), 's.png']);
end
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
% Define stable sampling time
Ts_stable = 0.1;

% Discretize the system
sysd = c2d(sys, Ts_stable);
Ad = sysd.A;
Bd = sysd.B;

% Design discrete controller (map continuous poles to discrete domain)
desired_poles_d = exp(Ts_stable * desired_poles);
Kd = place(Ad, Bd, desired_poles_d);

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
figure(10);

% Plot position
subplot(3,1,1);
plot(t_discrete, x_discrete(1,:), 'LineWidth', 1.5);
title('Cart Position (s) - Discrete-Time Control');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

% Plot angle
subplot(3,1,2);
plot(t_discrete, x_discrete(3,:), 'LineWidth', 1.5);
title('Pendulum Angle (\phi) - Discrete-Time Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Plot control input
subplot(3,1,3);
stairs(t_discrete(1:end-1), u_discrete, 'LineWidth', 1.5);
title('Discrete Control Input u(k)');
xlabel('Time (s)');
ylabel('Force (N)');
grid on;

% Set overall title and save figure
sgtitle('B8: Discrete-Time Control of Nonlinear System');
saveas(gcf, 'discrete_nonlinear.png');

%% B9-B10: LQR optimal control design
% Initial condition for LQR test
x0_lqr = [0; 1; 0.1; 0];

% LQR weights
Q = diag([100, 1, 100, 1]); % Higher penalty on position and angle
R = 1;                      % Control input weight

% Calculate LQR optimal gain for discrete system with Ts = 0.1
Ts_lqr = 0.1;
sysd_lqr = c2d(sys, Ts_lqr);
Ad_lqr = sysd_lqr.A;
Bd_lqr = sysd_lqr.B;
K_lqr = dlqr(Ad_lqr, Bd_lqr, Q, R);

% Simulation parameters
T_total = 10;
N_steps = T_total / Ts_lqr;
x_lqr = zeros(4, N_steps+1);
x_lqr(:,1) = x0_lqr;
u_lqr = zeros(1, N_steps);

% Discrete-time simulation with LQR control
for k = 1:N_steps
    % Calculate LQR control input
    u_lqr(k) = -K_lqr * x_lqr(:, k);
    
    % Simulate one step of continuous system with constant input
    [~, x_temp] = ode45(@(t, x) inverted_pendulum_dynamics_zoh(t, x, u_lqr(k)), ...
                        [0 Ts_lqr], x_lqr(:, k));
    
    % Update state for next step
    x_lqr(:, k+1) = x_temp(end, :)';
end

% Time vector for plotting
t_lqr = 0:Ts_lqr:T_total;

% Create figure for LQR control
figure(9);

% Plot position
subplot(3,1,1);
plot(t_lqr, x_lqr(1,:), 'LineWidth', 1.5);
title('Cart Position (s) - LQR Control');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

% Plot angle
subplot(3,1,2);
plot(t_lqr, x_lqr(3,:), 'LineWidth', 1.5);
title('Pendulum Angle (\phi) - LQR Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Plot control input
subplot(3,1,3);
stairs(t_lqr(1:end-1), u_lqr, 'LineWidth', 1.5);
title('Control Input (u) - LQR Control');
xlabel('Time (s)');
ylabel('Force (N)');
grid on;

% Set overall title and save figure
sgtitle('LQR Optimal Control Response');
saveas(gcf, 'lqr_control.png');

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

%% Nonlinear inverted pendulum dynamics with zero-order hold input
function dx = inverted_pendulum_dynamics_zoh(t, x, u)
    % System parameters
    M = 1;
    L = 0.842;
    F = 1;
    g = 9.8093;

    % State variables
    s = x(1);      % Position
    ds = x(2);     % Velocity
    phi = x(3);    % Angle
    dphi = x(4);   % Angular velocity

    % State equations with constant input u
    dx = [ds;
          (u - F*ds) / M;
          dphi;
          (g*sin(phi))/L - (1/L)*((u - F*ds)/M)*cos(phi)];
end
%% Helper function to simulate discrete-time control
function [t, y, u] = simulate_discrete_control(Ts, x0, K)
    % Define the time vector
    t = 0:Ts:10; % Simulation time from 0 to 10 seconds

    % Initialize state and output
    y = zeros(length(t), 4); % [s; ds; phi; dphi]
    y(1, :) = x0'; % Set initial state

    % Initialize control input
    u = zeros(length(t), 1); % Control input

    % Simulate the system using discrete-time control
    for k = 1:length(t)-1
        % Current state
        x = y(k, :)';

        % Compute control input using the discrete-time controller
        u(k) = -K * x; % State feedback control law

        % Simulate the nonlinear system for one time step
        [~, x_next] = ode45(@(t, x) inverted_pendulum_dynamics(t, x, u(k)), ...
                      [t(k), t(k+1)], x);

        % Update the state
        y(k+1, :) = x_next(end, :);
    end

    % Store the final control input
    u(end) = -K * y(end, :)';
end
