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
x0 = [0.1; 0; 1.2; 0]; % 初始状态
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
%% Simulate the non-linear system use the same K
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