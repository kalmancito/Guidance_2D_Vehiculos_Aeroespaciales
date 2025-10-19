clear; clc; close all

% --- base scenario ---
s.dt = 0.01; s.Tmax = 240;
s.N0 = 8; s.a_max = 100; s.use_actuator = true;
s.m_pos0=[0;0]; s.m_vel0=300*[cosd(5);sind(5)];

s.t_pos0=[6000;2000]; s.t_vel0=-250*[cosd(-10);sind(-10)];
s.t_maneuver_on=5; s.t_maneuver_off=25; s.t_maneuver_acc=30;

s.r_impact_threshold=20; s.min_Vc_for_tgo=5;

% optional fields with defaults
s.m_heading_err_deg = 5; % deg
% s.t_heading_err_deg
% s.bias_lambda_deg
% s.sigma_lambda_deg
% s.lambda_dot_tau;

% Control de impresión del resumen:
s.print_summary = true;     % <- pon false para silenciar

% --- single run ---
sim = pn_2d_sim(s);
fprintf('Miss = %.2f m at t=%.2f s (impact=%d)\n',...
    sim.miss_distance, sim.impact_time, sim.impact);

%% ---- Plots

% Trayectorias
figure;
plot(sim.m_pos(1,:), sim.m_pos(2,:), '-', 'LineWidth', 1.8); hold on;
plot(sim.t_pos(1,:), sim.t_pos(2,:), '--', 'LineWidth', 1.4);
plot(sim.m_pos(1,1), sim.m_pos(2,1), 'ko','MarkerFaceColor','k');
plot(sim.t_pos(1,1), sim.t_pos(2,1), 'ro','MarkerFaceColor','r');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
legend('Missile','Target','Missile init','Target init','Location','best');
title(sprintf('Trajectories (2-D) — N=%.2f, HE=%.1f° , a_{max,M}=%.1f m/s^2, a_T=%.1f m/s^2',...
    s.N0, s.m_heading_err_deg, s.a_max, s.t_maneuver_acc));

figure;
plot(sim.t, (s.N0 .* max(sim.Vc,0) .* sim.lambda_dot)/9.8, '-', 'LineWidth',1.2); hold on;
plot(sim.t, vecnorm(sim.a_act)/9.8, '--', 'LineWidth',1.2);
yline(s.a_max/9.8,'r:','Actuator limit');
grid on; xlabel('time [s]'); ylabel('lateral acc [Gs]');
legend('cmd scalar (N V_c \cdot \lambda\_dot)','|a_{act}|','actuator limit','Location','best');
title(sprintf('Commanded vs Applied Lateral Acceleration — N=%.2f, HE=%.1f° , a_{max,M}=%.1f m/s^2, a_T=%.1f m/s^2',...
    s.N0, s.m_heading_err_deg, s.a_max, s.t_maneuver_acc));
