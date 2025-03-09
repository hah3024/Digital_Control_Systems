%%
M = 1; % 质量
L = 0.842; % 摆长
F = 1; % 摩擦系数
g = 9.8093; % 重力加速度

% 非线性系统方程
% x = [s; ds; phi; dphi]
% u = mu

% 线性化系统
A = [0 1 0 0;
     0 -F/M 0 0;
     0 0 0 1;
     0 F/(M*L) g/L 0];
B = [0; 1/M; 0; -1/(M*L)];
C = [1 0 0 0;
     0 0 1 0];
D = [0; 0];

% 状态空间表示
sys = ss(A, B, C, D);

% 设计状态反馈控制器
% 选择期望的闭环极点
desired_poles = [-1+1i; -1-1i; -0.6+1i; -0.6-1i];
K = place(A, B, desired_poles);

% 闭环系统
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);

% 仿真初始条件
x0 = [0; 1; 0; 0]; % 初始状态
t = 0:0.01:10; % 时间范围

% 仿真闭环系统
[y, t, x] = initial(sys_cl, x0, t);

% 计算控制输入
u = -K*x';

% 绘图
figure;
subplot(2,1,1);
plot(t, y);
title('Closed-Loop System Output');
xlabel('Time (s)');
ylabel('Output');
legend('s(t)', '\phi(t)');

subplot(2,1,2);
plot(t, u);
title('Control Input');
xlabel('Time (s)');
ylabel('u(t)');
%% Simulate the non-linear system use the same K B3
     tspan = [0 10]; % 从0到10秒

    % 仿真
    [t, x] = ode45(@(t, x) inverted_pendulum_dynamics(t, x, K), tspan, x0);

    % 绘图
    figure;
    subplot(2, 1, 1);
    plot(t, x(:, 1));
    title('Position s(t)');
    xlabel('Time (s)');
    ylabel('Position (m)');

    subplot(2, 1, 2);
    plot(t, x(:, 3));
    title('Angle \phi(t)');
    xlabel('Time (s)');
    ylabel('Angle (rad)');

%% B4-5采样时间影响
Ts_stable = 0.1; % 稳定的采样时间
Ts_unstable = 1; % 可能不稳定的采样时间

sysd_stable = c2d(ss(A_cl, B, C, D), Ts_stable);
sysd_unstable = c2d(ss(A_cl, B, C, D), Ts_unstable);

figure;
stairs(0:Ts_stable:10, initial(sysd_stable, x0, 0:Ts_stable:10));
title('稳定采样时间 T = 0.1s');

figure;
stairs(0:Ts_unstable:10, initial(sysd_unstable, x0, 0:Ts_unstable:10));
title('不稳定采样时间 T = 1s');

%% B6: 计算离散状态矩阵
sysd = c2d(ss(A, B, C, D), Ts_stable);
Ad = sysd.A;
Bd = sysd.B;
%% B7: 设计离散状态反馈控制器
%c conside the
desired_poles_d = [0.2; 0.1; 0.1+0.1j; 0.1-0.1j];
Kd = place(Ad, Bd, exp(Ts_stable*desired_poles_d));

%% B8: 应用离散控制到非线性系统
[t, x] = ode45(@(t, x) inverted_pendulum_dynamics_discrete(t, x, Kd, Ts_stable), [0 10], x0);
figure;
subplot(2,1,1);
plot(t, x(:,1));
title('离散时间控制: 位置 s(t)');

subplot(2,1,2);
plot(t, x(:,3));
title('离散时间控制: 角度 \phi(t)');
%% try use the brutel force to solve the problem
% 离散时间仿真
T_total = 10;         % 总仿真时间
N_steps = T_total / Ts_stable; % 计算离散时间步数
x_discrete = zeros(4, N_steps+1); % 存储状态
x_discrete(:,1) = x0;  % 初始状态
u_discrete = zeros(1, N_steps); % 存储控制输入

% 离散仿真
for k = 1:N_steps
    u_discrete(k) = -Kd * x_discrete(:, k);  % 计算控制输入
    x_discrete(:, k+1) = Ad * x_discrete(:, k) + Bd * u_discrete(k);  % 状态更新
end

% 绘图
t_discrete = 0:Ts_stable:T_total;
figure;
subplot(2,1,1);
plot(t_discrete, x_discrete(1,:), 'b', t_discrete, x_discrete(3,:), 'r');
title('离散时间控制: 位置 s(t) 和 角度 \phi(t)');
xlabel('时间 (s)');
ylabel('状态');
legend('s(t)', '\phi(t)');

subplot(2,1,2);
stairs(t_discrete(1:end-1), u_discrete);
title('离散时间控制: 控制输入 u(k)');
xlabel('时间 (s)');
ylabel('u(k)');


%% B9-B10: 设计 LQR 最优控制
Q = diag([100, 1, 100, 1]);
R = 1;
K_opt = dlqr(Ad, Bd, Q, R);

[t, x] = ode45(@(t, x) inverted_pendulum_dynamics_discrete(t, x, K_opt, Ts_stable), [0 10], x0);
figure;
subplot(2,1,1);
plot(t, x(:,1));
title('LQR 控制: 位置 s(t)');

subplot(2,1,2);
plot(t, x(:,3));
title('LQR 控制: 角度 \phi(t)');
%%
%% B9-B10: 设计 LQR 最优控制
x0 = [-0.2; 0; 0.1; 0];
Q = diag([100, 1, 100, 1]); % 惩罚 s 和 phi 比速度更大
R = 1; % 控制输入权重
K_opt = dlqr(Ad, Bd, Q, R); % 计算 LQR 最优增益

% 仿真参数
T_total = 10;         
N_steps = T_total / Ts_stable; % 计算离散时间步数
x_lqr = zeros(4, N_steps+1); % 存储状态
x_lqr(:,1) = x0;  % 初始状态
u_lqr = zeros(1, N_steps); % 存储控制输入

% 迭代计算 LQR 控制下的状态演化
for k = 1:N_steps
    u_lqr(k) = -K_opt * x_lqr(:, k);  % 计算控制输入
    x_lqr(:, k+1) = Ad * x_lqr(:, k) + Bd * u_lqr(k);  % 更新状态
end

% 绘图
t_discrete = 0:Ts_stable:T_total;
figure;
subplot(2,1,1);
plot(t_discrete, x_lqr(1,:), 'b', t_discrete, x_lqr(3,:), 'r');
title('LQR 控制: 位置 s(t) 和 角度 \phi(t)');
xlabel('时间 (s)');
ylabel('状态');
legend('s(t)', '\phi(t)');

subplot(2,1,2);
stairs(t_discrete(1:end-1), u_lqr);
title('LQR 控制: 控制输入 u(k)');
xlabel('时间 (s)');
ylabel('u(k)');

%%
reachability_matrix = ctrb(A, B);
% 检查可达性
if rank(reachability_matrix) == size(A, 1)
    disp('The pair (A, B) is reachable.');
else
    disp('The pair (A, B) is not reachable.');
end

% 可观测性矩阵
observability_matrix = obsv(A, C);

% 检查可观测性
if rank(observability_matrix) == size(A, 1)
    disp('The pair (A, C) is observable.');
else
    disp('The pair (A, C) is not observable.');
end
%% continuous inverted_pendulum_dynamics systems
function dx = inverted_pendulum_dynamics(t, x, K)
    % 系统参数
    M = 1;
    L = 0.842;
    F = 1;
    g = 9.8093;

    % 状态变量
    s = x(1);
    ds = x(2);
    phi = x(3);
    dphi = x(4);

    % 控制输入
    u = -K * x;

    % 状态方程
    dx = [ds;
          (u - F*ds) / M;
          dphi;
          (g*sin(phi))/L - 1/L*((u - F*ds) / M)*cos(phi)];
end
%% Discrete time for the inverted_pendulum (some mistake here)
function dx = inverted_pendulum_dynamics_discrete(t, x, Kd, Ts)
    % System parameters
    M = 1;
    L = 0.842;
    F = 1;
    g = 9.8093;

    % Control input
    u = -Kd * x;

    % State equations with zero-order hold input
    dx = [x(2);
          (u - F*x(2)) / M;
          x(4);
          (g*sin(x(3)))/L - 1/L*((u - F*x(2)) / M)*cos(x(3))];
end

