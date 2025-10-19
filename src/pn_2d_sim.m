function sim = pn_2d_sim(scenario)
% PN_2D_SIM  Simulate 2-D Proportional Navigation guidance (with heading and seeker errors)
%
% Returns struct 'sim' with time histories and a text summary in sim.summary
%
% Optional fields:
%   m_heading_err_deg, t_heading_err_deg, bias_lambda_deg, sigma_lambda_deg, lambda_dot_tau

% -------------------------------------------------------------------------
dt = scenario.dt; Tmax = scenario.Tmax;
N0 = scenario.N0; a_max = scenario.a_max; use_act = scenario.use_actuator;
m_pos = scenario.m_pos0; m_vel = scenario.m_vel0;
t_pos = scenario.t_pos0; t_vel = scenario.t_vel0;

t_on  = scenario.t_maneuver_on;
t_off = scenario.t_maneuver_off;
a_tlat= scenario.t_maneuver_acc;
r_thr = scenario.r_impact_threshold;
minVc = scenario.min_Vc_for_tgo;

% optional fields with defaults
if isfield(scenario,'m_heading_err_deg'), m_head_err = deg2rad(scenario.m_heading_err_deg);
else, m_head_err = 0; end
if isfield(scenario,'t_heading_err_deg'), t_head_err = deg2rad(scenario.t_heading_err_deg);
else, t_head_err = 0; end
if isfield(scenario,'bias_lambda_deg'), bias_lambda = deg2rad(scenario.bias_lambda_deg);
else, bias_lambda = 0; end
if isfield(scenario,'sigma_lambda_deg'), sigma_lambda = deg2rad(scenario.sigma_lambda_deg);
else, sigma_lambda = 0; end
if isfield(scenario,'lambda_dot_tau'), tau_ldot = scenario.lambda_dot_tau;
else, tau_ldot = 0; end
% --------- lee print_summary con default = false ----------
if isfield(scenario,'print_summary'), print_summary = logical(scenario.print_summary);
else, print_summary = false; end


% apply heading errors
Rm = [cos(m_head_err) -sin(m_head_err); sin(m_head_err) cos(m_head_err)];
Rt = [cos(t_head_err) -sin(t_head_err); sin(t_head_err) cos(t_head_err)];
m_vel = Rm*m_vel;
t_vel = Rt*t_vel;

% -------------------------------------------------------------------------
tvec = 0:dt:Tmax; nt = numel(tvec);

% Preallocate
m_pos_hist = nan(2,nt); t_pos_hist = nan(2,nt);
m_vel_hist = nan(2,nt); t_vel_hist = nan(2,nt);
lambda = nan(1,nt); lambda_dot = nan(1,nt);
lambda_meas = nan(1,nt); lambda_dot_meas = nan(1,nt);
range = nan(1,nt);  Vc = nan(1,nt);
a_cmd = nan(2,nt);  a_act = nan(2,nt);
ZEM = nan(1,nt);    tgo = nan(1,nt);

m_pos_hist(:,1)=m_pos; m_vel_hist(:,1)=m_vel;
t_pos_hist(:,1)=t_pos; t_vel_hist(:,1)=t_vel;

impact=false; impact_time=NaN;
lambda_prev_meas = NaN; lambda_dot_filt = 0;

for k = 1:nt-1
    tnow = tvec(k);
    rel = t_pos - m_pos;
    R = norm(rel);
    if R < 1e-9, break; end
    range(k) = R;

    % true LOS and derivatives
    lambda_true = atan2(rel(2), rel(1));
    rel_v = t_vel - m_vel;
    lambda_dot_true = (rel(1)*rel_v(2)-rel(2)*rel_v(1))/(R^2);
    Vc(k) = -(rel.'*rel_v)/R;
    lambda(k)=lambda_true; lambda_dot(k)=lambda_dot_true;

    % measured LOS angle (bias + noise)
    e_k = bias_lambda + sigma_lambda*randn;
    lambda_meas(k) = atan2(sin(lambda_true + e_k), cos(lambda_true + e_k));

    % measured lambda-dot (finite diff + optional LPF)
    if isnan(lambda_prev_meas)
        lambda_dot_m = lambda_dot_true;
    else
        dphi = atan2(sin(lambda_meas(k) - lambda_prev_meas), cos(lambda_meas(k) - lambda_prev_meas));
        lambda_dot_m = dphi/dt;
    end
    lambda_prev_meas = lambda_meas(k);

    if tau_ldot > 0
        alpha = dt/(tau_ldot+dt);
        lambda_dot_filt = (1-alpha)*lambda_dot_filt + alpha*lambda_dot_m;
        lambda_dot_meas(k)=lambda_dot_filt;
    else
        lambda_dot_meas(k)=lambda_dot_m;
    end

    % time-to-go
    if Vc(k)>minVc, tgo(k)=R/Vc(k); else, tgo(k)=NaN; end

    % LOS direction (measured)
    u_los = [cos(lambda_meas(k)); sin(lambda_meas(k))];
    u_perp = [-u_los(2); u_los(1)];

    % Predicted ZEM (optional)
    if ~isnan(tgo(k))
        rel_pred = rel + rel_v*tgo(k);
        ZEM(k) = rel_pred.'*u_perp;
    else
        ZEM(k) = rel.'*u_perp;
    end

    % PN guidance (using measured lambda-dot)
    n_c = N0*max(Vc(k),0)*lambda_dot_meas(k);
    a_c = n_c*u_perp; a_cmd(:,k)=a_c;

    % actuator saturation
    nrm = norm(a_c);
    if use_act
        if nrm>a_max, a_act(:,k)=a_c*(a_max/nrm); else, a_act(:,k)=a_c; end
    else
        a_act(:,k)=a_c;
    end

    % target maneuver
    if tnow>=t_on && tnow<=t_off
        v = t_vel; vn=norm(v)+eps;
        u_tperp=[-v(2);v(1)]/vn;
        a_tcmd=a_tlat*u_tperp;
    else
        a_tcmd=[0;0];
    end

    % RK4 step
    [m_pos,m_vel,t_pos,t_vel] = rk4_step(m_pos,m_vel,a_act(:,k),...
                                         t_pos,t_vel,a_tcmd,dt);

    m_pos_hist(:,k+1)=m_pos; m_vel_hist(:,k+1)=m_vel;
    t_pos_hist(:,k+1)=t_pos; t_vel_hist(:,k+1)=t_vel;

    if norm(t_pos-m_pos)<=r_thr
        impact=true; impact_time=tvec(k+1);
        range(k+1)=norm(t_pos-m_pos); % ensure last range stored
        break;
    end
end

if impact
    idx = find(tvec<=impact_time+eps,1,'last');
else
    idx = nt;
end

% trim and output
fn = @(x)x(:,1:idx);
sim.t=tvec(1:idx);
sim.m_pos=fn(m_pos_hist); sim.t_pos=fn(t_pos_hist);
sim.m_vel=fn(m_vel_hist); sim.t_vel=fn(t_vel_hist);
sim.lambda=lambda(1:idx); sim.lambda_dot=lambda_dot(1:idx);
sim.lambda_meas=lambda_meas(1:idx); sim.lambda_dot_meas=lambda_dot_meas(1:idx);
sim.range=range(1:idx); sim.Vc=Vc(1:idx);
sim.a_cmd=a_cmd(:,1:idx); sim.a_act=a_act(:,1:idx);
sim.ZEM=ZEM(1:idx); sim.tgo=tgo(1:idx);
sim.impact=impact; sim.impact_time=impact_time;
if impact
    sim.miss_distance = norm(sim.t_pos(:,end)-sim.m_pos(:,end));
else
    sim.miss_distance = sim.range(end);
end

% -------------------------------------------------------------------------
% Textual summary (siempre se construye; imprimir es opcional)
% -------------------------------------------------------------------------
m_speed0 = norm(scenario.m_vel0);
t_speed0 = norm(scenario.t_vel0);
closing0 = -(scenario.t_vel0 - scenario.m_vel0)' * (scenario.t_pos0 - scenario.m_pos0) ...
           / norm(scenario.t_pos0 - scenario.m_pos0);

outcome = "MISS";
if impact, outcome = "HIT"; end

summary_txt = sprintf([ ...
    '--- 2D PN Encounter Summary ---\n' ...
    'Navigation constant N0     : %.2f\n' ...
    'Max accel (m/s²)           : %.1f\n' ...
    'dt / Tmax (s)              : %.3f / %.1f\n' ...
    'Impact threshold [m]       : %.2f\n' ...    
    'Min closing vel for tgo    : %.2f m/s\n' ...    
    '\nInitial conditions:\n' ...
    '  Missile pos [m]          : [%.0f, %.0f]\n' ...
    '  Target pos [m]           : [%.0f, %.0f]\n' ...
    '  Missile speed [m/s]      : %.1f\n' ...
    '  Target speed [m/s]       : %.1f\n' ...
    '  Initial closing vel [m/s]: %.1f\n' ...
    '\nErrors:\n' ...
    '  Missile heading err [deg]: %.2f\n' ...
    '  Target heading err [deg] : %.2f\n' ...
    '  Seeker bias [deg]        : %.2f\n' ...
    '  Seeker noise σ [deg]     : %.2f\n' ...
    '\nResult: %s\n' ...
    '  Miss distance [m]        : %.2f\n' ...
    '  Time to impact [s]       : %.2f\n' ...
    '-------------------------------\n'], ...
    N0, a_max, dt, Tmax, ...
    r_thr, ...    
    minVc, ...   
    scenario.m_pos0(1), scenario.m_pos0(2), ...
    scenario.t_pos0(1), scenario.t_pos0(2), ...
    m_speed0, t_speed0, closing0, ...
    rad2deg(m_head_err), rad2deg(t_head_err), ...
    rad2deg(bias_lambda), rad2deg(sigma_lambda), ...
    outcome, sim.miss_distance, sim.impact_time);

if print_summary
    fprintf('%s', summary_txt);
end
sim.summary = summary_txt;
end

% =========================== helper ======================================
function [m_pos,m_vel,t_pos,t_vel] = rk4_step(m_pos,m_vel,a_m,...
                                              t_pos,t_vel,a_t,dt)
m_k1_pos = m_vel;  m_k1_vel = a_m;
t_k1_pos = t_vel;  t_k1_vel = a_t;

m_k2_pos = m_vel + 0.5*dt*m_k1_vel; m_k2_vel = a_m;
t_k2_pos = t_vel + 0.5*dt*t_k1_vel; t_k2_vel = a_t;

m_k3_pos = m_vel + 0.5*dt*m_k2_vel; m_k3_vel = a_m;
t_k3_pos = t_vel + 0.5*dt*t_k2_vel; t_k3_vel = a_t;

m_k4_pos = m_vel + dt*m_k3_vel;     m_k4_vel = a_m;
t_k4_pos = t_vel + dt*t_k3_vel;     t_k4_vel = a_t;

m_pos = m_pos + (dt/6)*(m_k1_pos+2*m_k2_pos+2*m_k3_pos+m_k4_pos);
m_vel = m_vel + (dt/6)*(m_k1_vel+2*m_k2_vel+2*m_k3_vel+m_k4_vel);
t_pos = t_pos + (dt/6)*(t_k1_pos+2*t_k2_pos+2*t_k3_pos+t_k4_pos);
t_vel = t_vel + (dt/6)*(t_k1_vel+2*t_k2_vel+2*t_k3_vel+t_k4_vel);
end
