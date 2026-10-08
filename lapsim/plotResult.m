function fig = plotResult(res, visible)
%PLOTRESULT Speed, energy, power and energy-allocation plots of a lapsim result.
%   fig = plotResult(res) draws a lapsim() result in a new figure.
%   fig = plotResult(res, 'off') keeps it hidden, for export:
%       exportgraphics(plotResult(res, 'off'), 'baseline.png', 'Resolution', 160);
%
%   Pure presentation: it reads res and changes nothing but its own figure
%   (no global graphics defaults). Ported from the Chat branch's plotResult.
if nargin < 2
    visible = 'on';
end
ts = res.ts;  sm = res.summary;  p = res.params;  E = sm.energy;
MPH = 0.44704;                                       % [m/s per mph]
ink = [0.15 0.15 0.15];

fig = figure('Name', 'lapsim', 'Color', 'w', 'Visible', visible, 'Position', [100 100 1200 760]);
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Speed against distance, with the corner speed limit
nexttile;
n = numel(ts.s_lap);
plot(ts.x(1:n)/1000, ts.v(1:n)/MPH, 'LineWidth', 1.2); hold on;
plot(ts.x(1:n)/1000, min(ts.v_env, 2*max(ts.v))/MPH, ':', 'LineWidth', 1);
xlabel('Distance (km)'); ylabel('Speed (mph)'); grid on;
legend({'car', 'corner envelope'}, 'Location', 'best');
if sm.finished
    title(sprintf('%.2f min, %.1f mph average', sm.time_min, sm.v_avg/MPH));
else
    title(sprintf('DNF (%s) after %.0f m', strrep(sm.stop_reason, '_', ' '), sm.distance_m));
end

% Joulemeter energy against time, with the budget for the team goal
nexttile;
plot(ts.t/60, ts.E_batt/3600, 'LineWidth', 1.4); hold on;
yline(sm.energy_budget_Wh, '--', sprintf('%.0f mi/kWh goal', p.goal.target_mi_per_kWh));
xlabel('Time (min)'); ylabel('Energy at the joulemeter (Wh)'); grid on;
if sm.finished
    title(sprintf('%.1f mi/kWh, %.2f Wh', sm.mi_per_kWh, sm.E_Wh));
else
    title(sprintf('%.2f Wh used', sm.E_Wh));
end

% Terminal power
nexttile;
plot(ts.t(1:n)/60, ts.P_batt, 'LineWidth', 0.9);
xlabel('Time (min)'); ylabel('Terminal power (W)'); grid on;
title('Motor, controller and auxiliaries');

% Where the joulemeter energy went
nexttile;
vals = [E.rolling, E.aero, E.copper, E.motor_friction, E.drivetrain, E.controller, ...
        E.aux, E.brakes, E.coast_drag, E.grade, E.kinetic_gain] / 3600;   % [Wh]
barh(vals);
yticks(1:numel(vals));
yticklabels({'Rolling', 'Aero', 'Motor copper', 'Motor friction', 'Drivetrain', 'Controller', ...
             'Auxiliary', 'Brakes', 'Coast drag', 'Grade (net)', 'Kinetic at end'});
xlabel('Energy (Wh); aero, grade and kinetic are signed'); grid on;
set(gca, 'YDir', 'reverse');
title('Energy allocation');

sgtitle(sprintf('lapsim | %.1f kg | %s | feasible: %d | goal met: %d', res.derived.m, ...
    strrep(char(p.strategy.mode), '_', ' '), sm.feasible, sm.target_met), 'Color', ink);
% Explicit colours keep exports readable when MATLAB uses a dark theme
set(findall(fig, 'Type', 'axes'), 'Color', 'w', 'XColor', ink, 'YColor', ink, ...
    'GridColor', [0.65 0.65 0.65]);
set(findall(fig, 'Type', 'text'), 'Color', ink);
set(findall(fig, 'Type', 'constantline'), 'Color', [0.4 0.4 0.4]);
end
