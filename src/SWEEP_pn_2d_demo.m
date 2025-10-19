clear; clc; close all

% --- base scenario MISSILE ---
s.dt = 0.01; s.Tmax = 240;
s.N0 = 8; s.a_max = 100; s.use_actuator = true;
s.m_pos0=[0;0]; s.m_vel0=300*[cosd(5);sind(5)];

% --- base scenario TARGET ---
s.t_pos0=[6000;0]; s.t_vel0=-250*[cosd(-10);sind(-10)];
s.t_maneuver_on=0; s.t_maneuver_off=25; s.t_maneuver_acc=-30;

% --- base scenario GENERAL ---
s.r_impact_threshold=20; s.min_Vc_for_tgo=5;
s.m_heading_err_deg = 0;
s.print_summary = false;

% --- parámetros del barrido ---
Ns               = [3, 4, 5, 6, 8];
HE_deg_vec       = [0, 20, 30];
amax_missile_vec = [50, 100];
aT_vec           = s.t_maneuver_acc;   % o [10 20 30] si quieres barrer también

% --- opciones de salida ---
SAVE_FIGS = false;  
OUTDIR = "fig";
if SAVE_FIGS && ~exist(OUTDIR,'dir')
    mkdir(OUTDIR);
end

% --- almacenamiento de resultados ---
nN  = numel(Ns); nHE = numel(HE_deg_vec);
nAM = numel(amax_missile_vec); nAT = numel(aT_vec);
rows = nN*nHE*nAM*nAT;

N_col=nan(rows,1); HE_col=nan(rows,1); AM_col=nan(rows,1);
AT_col=nan(rows,1); impact_col=false(rows,1);
miss_col=nan(rows,1); timp_col=nan(rows,1);
idx=0;

%% === BUCLE PRINCIPAL ===
for iAT=1:nAT
 for iA=1:nAM
  for iH=1:nHE
      % --- crea figura con 2 subplots (trayectorias + aceleraciones) ---
      f = figure('Name',sprintf('HE=%.1f°, a_{max,M}=%.0f',HE_deg_vec(iH),amax_missile_vec(iA)),...
                 'Position',[100 100 900 700]);
      tiledlayout(2,1,'TileSpacing','compact');

      % Subplot 1: Trayectorias
      nexttile; hold on;
      for iN=1:nN
          s.N0 = Ns(iN);
          s.a_max = amax_missile_vec(iA);
          s.m_heading_err_deg = HE_deg_vec(iH);
          s.t_maneuver_acc = aT_vec(iAT);

          sim = pn_2d_sim(s);
          plot(sim.m_pos(1,:), sim.m_pos(2,:), 'LineWidth', 1.5, ...
              'DisplayName', sprintf('N=%.1f', s.N0));

          % Guardar resultados en tabla
          idx=idx+1;
          N_col(idx)=s.N0; HE_col(idx)=s.m_heading_err_deg;
          AM_col(idx)=s.a_max; AT_col(idx)=s.t_maneuver_acc;
          impact_col(idx)=logical(sim.impact);
          miss_col(idx)=sim.miss_distance; timp_col(idx)=sim.impact_time;
      end
      % Añadir blanco (solo una vez)
      plot(sim.t_pos(1,:), sim.t_pos(2,:), '--k','LineWidth',1.0,'DisplayName','Target');
      plot(sim.t_pos(1,1), sim.t_pos(2,1),'ro','MarkerFaceColor','r');
      axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
      legend('Location','bestoutside');
      title(sprintf('Trajectories — HE=%.1f°, a_{max,M}=%.0f m/s², a_T=%.0f m/s²',...
          HE_deg_vec(iH), amax_missile_vec(iA), s.t_maneuver_acc));

      % Subplot 2: Aceleraciones laterales
      nexttile; hold on;
      for iN=1:nN
          s.N0 = Ns(iN);
          s.a_max = amax_missile_vec(iA);
          s.m_heading_err_deg = HE_deg_vec(iH);
          s.t_maneuver_acc = aT_vec(iAT);

          sim = pn_2d_sim(s);
          a_act_g = vecnorm(sim.a_act,2,1)/9.8;
          plot(sim.t, a_act_g, 'LineWidth',1.2, 'DisplayName', sprintf('N=%.1f',s.N0));
      end
      yline(s.a_max/9.8,'r:','Actuator limit');
      grid on; xlabel('time [s]'); ylabel('|a_{act}| [Gs]');
      legend('Location','bestoutside');
      title(sprintf('Lateral Accel — HE=%.1f°, a_{max,M}=%.0f m/s², a_T=%.0f m/s²',...
          HE_deg_vec(iH), amax_missile_vec(iA), s.t_maneuver_acc));

      % Título global de figura
      sgtitle(sprintf('Sweep in N for HE=%.1f°, a_{max,M}=%.0f m/s², a_T=%.0f m/s²',...
          HE_deg_vec(iH), amax_missile_vec(iA), s.t_maneuver_acc));

      % Guardar si está activo
      if SAVE_FIGS
          fn = sprintf('%s/pn2d_HE%.1f_aM%.0f_aT%.0f.png', ...
                       OUTDIR, HE_deg_vec(iH), amax_missile_vec(iA), s.t_maneuver_acc);
          exportgraphics(f, fn, 'Resolution', 150);
%           close(f); % cierra para no saturar
      end
  end
 end
end

%% === Tabla global de resultados ===
T = table(N_col, HE_col, AM_col, AT_col, impact_col, miss_col, timp_col, ...
    'VariableNames', {'N','HE_deg','a_max_missile','a_T_target','Impact','Miss_m','T_impact_s'});
disp(T);

%% === Heatmaps (Miss y T_impact) por a_max ===
for iA=1:nAM
    sub = T(T.a_max_missile==amax_missile_vec(iA),:);
    MissGrid = nan(numel(HE_deg_vec), numel(Ns));
    TimpGrid = nan(numel(HE_deg_vec), numel(Ns));
    for iH=1:numel(HE_deg_vec)
        for iN=1:numel(Ns)
            idxs = sub.N==Ns(iN) & sub.HE_deg==HE_deg_vec(iH);
            if any(idxs)
                MissGrid(iH,iN) = sub.Miss_m(idxs);
                TimpGrid(iH,iN) = sub.T_impact_s(idxs);
            end
        end
    end

    f = figure('Name',sprintf('Heatmaps a_{max,M}=%.0f',amax_missile_vec(iA)),...
           'Position',[200 200 900 400]);
    tiledlayout(1,2,'TileSpacing','compact');

    % Miss distance
    nexttile;
    imagesc(Ns, HE_deg_vec, MissGrid);
    colorbar; xlabel('N'); ylabel('HE [deg]');
    title(sprintf('Miss distance [m] — a_{max,M}=%.0f m/s²',amax_missile_vec(iA)));
    set(gca,'YDir','normal');

    % Impact time
    nexttile;
    imagesc(Ns, HE_deg_vec, TimpGrid);
    colorbar; xlabel('N'); ylabel('HE [deg]');
    title(sprintf('Impact time [s] — a_{max,M}=%.0f m/s²',amax_missile_vec(iA)));
    set(gca,'YDir','normal');

    if SAVE_FIGS
        fn = sprintf('%s/heatmap_aM%.0f.png', OUTDIR, amax_missile_vec(iA));
        exportgraphics(f, fn, 'Resolution', 150);
        close(f);
    end
end
