function out = runBaseline(outputDirectory, runStudies)
%RUNBASELINE Reproducible baseline run with CSV and plot export, plus optional studies.
%   out = runBaseline                 baseline + studies, written to lapsim/results/latest
%   out = runBaseline(dir)            same, into the folder dir
%   out = runBaseline(dir, false)     baseline only (about 1 s instead of about a minute)
%
%   Writes summary.csv, trace.csv, energy.csv, baseline.png and a .mat file
%   holding the exact inputs; with studies it also writes sensitivity.csv,
%   strategies.csv and targets_*.csv. The results folder is ignored by git.
%   For your own inputs call lapsim(p) directly (see parameters.m).
%
%   Ported from the Chat branch's runLapSim; the model behind it is this one.
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(outputDirectory)
    outputDirectory = fullfile(here, 'results', 'latest');
end
if nargin < 2
    runStudies = true;
end
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end
MPH = 0.44704;                                        % [m/s per mph]

p = parameters();
out.params   = p;
out.baseline = lapsim(p);                             % prints the summary
sm = out.baseline.summary;
writetable(struct2table(rmfield(sm, 'energy')), fullfile(outputDirectory, 'summary.csv'));
writetable(struct2table(sm.energy), fullfile(outputDirectory, 'energy.csv'));
writetable(timeseriesTable(out.baseline.ts), fullfile(outputDirectory, 'trace.csv'));
fig = plotResult(out.baseline, 'off');
exportgraphics(fig, fullfile(outputDirectory, 'baseline.png'), 'Resolution', 160);
close(fig);

if runStudies
    fprintf('\nRunning sensitivity, strategy and target studies (about a minute)...\n');
    out.sensitivity = sensitivity(p);
    writetable(out.sensitivity, fullfile(outputDirectory, 'sensitivity.csv'));
    out.strategies = strategySweep(p, [16.6 17 17.5 18 19] * MPH, [0 1 2] * MPH);
    writetable(out.strategies, fullfile(outputDirectory, 'strategies.csv'));
    out.targets.P_aux = targetSweep(p, 'elec.P_aux', [0 2 5 10 20]);
    writetable(out.targets.P_aux, fullfile(outputDirectory, 'targets_P_aux.csv'));
    out.targets.C_rr = targetSweep(p, 'vehicle.C_rr', [0.002 0.003 0.004 0.0056 0.0075 0.009]);
    writetable(out.targets.C_rr, fullfile(outputDirectory, 'targets_C_rr.csv'));
    out.targets.m_vehicle = targetSweep(p, 'mass.m_vehicle', [20 33.6 50 70 83.6]);
    writetable(out.targets.m_vehicle, fullfile(outputDirectory, 'targets_m_vehicle.csv'));
    best = out.strategies(out.strategies.is_best_feasible, ...
        {'strategy_mode', 'speed_mps', 'band_mps', 'time_s', 'E_Wh', 'mi_per_kWh'});
    fprintf('Best feasible strategy on the grid (not a global optimum):\n');
    disp(best);
end
save(fullfile(outputDirectory, 'results.mat'), 'out');
fprintf('Wrote inputs and outputs to %s\n', outputDirectory);
end

function T = timeseriesTable(ts)
% One row per time step; the state vectors lose their initial row to match
n = numel(ts.s_lap);
T = table(ts.t(2:end), ts.x(2:end), ts.v(2:end), ts.E_batt(2:end)/3600, ...
    ts.s_lap(:), ts.drive(:), ts.T_motor(:), ts.rpm_motor(:), ts.I(:), ts.duty(:), ...
    ts.P_batt(:), ts.F_wheel(:), ts.F_brake(:), ts.F_trac_limit(:), ts.headwind(:), ...
    'VariableNames', {'t_s', 'x_m', 'v_mps', 'E_terminal_Wh', 's_lap_m', 'driving', ...
    'T_motor_Nm', 'rpm_motor', 'I_motor_A', 'duty', 'P_terminal_W', 'F_wheel_N', ...
    'F_brake_N', 'F_traction_limit_N', 'headwind_mps'});
assert(height(T) == n);
end
