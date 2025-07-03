clc;
clear;

% System parameters
m = 2;                % kg
g = 9.81;             % m/s^2
z_desired = 0.2;      % meters (20 cm)
T_sample = 1/400;     % seconds

% Continuous-time system
A = [0 1; 0 0];
B = [0; 1/m];
C = [1 0];
D = 0;
sys = ss(A, B, C, D);

% Discretize
sys_z = c2d(sys, T_sample, 'zoh');
Ad = sys_z.A;
Bd = sys_z.B;
Cd = sys_z.C;
Dd = sys_z.D;

disp(Ad);
disp(Bd);
disp(Cd);

% Check controllability and observability
Cm = [Bd Ad*Bd];
disp('Rank of controllability matrix:');
disp(rank(Cm));

Om = [Cd ; Cd*Ad];
disp('Rank of observability matrix:');
disp(rank(Om));

% Choose closed-loop poles inside unit circle
p1 = 0.999;
p2 = 0.998;

% Place poles and compute gain K
K = place(Ad, Bd, [p1 p2]);
disp('State feedback gain K:');
disp(K);

% Choose poles slightly faster than controller poles
p_obs = [0.97, 0.96];
L = place(Ad', Cd', p_obs)';
disp('Observer gain L is:');
disp(L);

% Compute feedforward gain N to track z_desired
N = inv(Cd * inv(eye(2) - Ad + Bd * K) * Bd);
disp('Feedforward gain N:');
disp(N);

% Simulation setup
n_steps = 9000;
x = [0; 0];               % Initial state: z = 0, vz = 0
z_history = zeros(1, n_steps);
u_history = zeros(1, n_steps);
T_history = zeros(1, n_steps); % Actual thrust applied

for k = 1:n_steps
    % Control law
    u = -K * x + N * z_desired;     % u = T - mg
    T =  u + m * g;                  % actual thrust

    % System update
    x = Ad * x + Bd * u;

    % Store
    z_history(k) = x(1);
    u_history(k) = u;
    T_history(k) = T;
end

% Plotting
t = (0:n_steps-1) * T_sample;

figure;
plot(t, z_history, 'b', 'LineWidth', 2);
yline(z_desired, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Altitude z (m)');
title('Quadcopter Altitude Response');
legend('z(t)', 'z_{desired}');

figure;
plot(t, T_history, 'k', 'LineWidth', 2);
yline(m * g, 'r--');
xlabel('Time (s)');
ylabel('Thrust T (N)');
title('Thrust Command vs Hover Thrust');
legend('T(t)', 'mg');
