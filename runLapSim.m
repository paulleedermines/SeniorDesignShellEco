function results = runLapSim(outputDirectory, runStudies)
%RUNLAPSIM Reproducible baseline, design studies, CSVs, and plot exports.
% Run from the repository root: results = runLapSim;
% Faster baseline only: results = runLapSim('results/baseline',false);
% For custom inputs call lapsim.simulate(p,track) directly (see README).
if nargin<1, outputDirectory=fullfile('results','latest'); end
if nargin<2, runStudies=true; end
if ~exist(outputDirectory,'dir'), mkdir(outputDirectory); end
p=lapsim.defaultParameters(); track=lapsim.exampleTrack();
results.baseline=lapsim.simulate(p,track);
s=results.baseline.summary;
fprintf('\nShell Eco-marathon LapSim (estimated inputs)\n');
fprintf('Total mass %.2f kg (car %.2f + driver %.2f). Replace with measurements.\n', ...
    s.total_mass_kg,p.vehicle.chassis_mass_kg,p.vehicle.driver_mass_kg);
fprintf('%.2f km | %.2f min | %.2f mph average | feasible %d\n', ...
    s.distance_m/1000,s.time_s/60,s.average_speed_mps/0.44704,s.feasible);
fprintf('%.2f Wh terminal | %.2f Wh including battery loss | %.1f mi/kWh\n', ...
    s.terminal_energy_Wh,s.battery_energy_Wh,s.mi_per_kWh);
fprintf('%.0f mi/kWh target: %.2f Wh allowed | margin %+.2f Wh | target met %d\n', ...
    p.race.target_mi_per_kWh,s.energy_budget_Wh,s.energy_margin_Wh,s.target_met);
writetable(struct2table(s),fullfile(outputDirectory,'summary.csv'));
writetable(results.baseline.trace,fullfile(outputDirectory,'trace.csv'));
writetable(track,fullfile(outputDirectory,'track.csv'));
writetable(struct2table(results.baseline.energy),fullfile(outputDirectory,'energy.csv'));
fig=lapsim.plotResult(results.baseline,'off');
exportgraphics(fig,fullfile(outputDirectory,'baseline.png'),'Resolution',160);
close(fig);
if runStudies
    fprintf('Running independent sensitivity and strategy cases...\n');
    results.sensitivity=lapsim.sensitivity(p,track);
    results.strategies=lapsim.strategySweep(p,track,[16.6 17 17.5 18 19]*0.44704,[0 1 2]*0.44704);
    results.rollingTargets=lapsim.targetSweep(p,track,'tires.crr',[.001 .002 .003 .004 .0056]);
    % Total-mass study holds driver mass at 50 kg; label this separate assumption.
    massStudy=p; massStudy.vehicle.driver_mass_kg=50;
    results.massTargets=lapsim.targetSweep(massStudy,track,'vehicle.chassis_mass_kg',[10 20 35 50 70 100]);
    writetable(results.sensitivity,fullfile(outputDirectory,'sensitivity.csv'));
    writetable(results.strategies,fullfile(outputDirectory,'strategies.csv'));
    writetable(results.rollingTargets,fullfile(outputDirectory,'rolling_targets.csv'));
    writetable(results.massTargets,fullfile(outputDirectory,'mass_targets_driver50kg.csv'));
    disp(results.strategies(results.strategies.is_best_feasible, ...
        {'strategy_mode','speed_mps','band_mps','terminal_energy_Wh','mi_per_kWh','target_met'}));
end
save(fullfile(outputDirectory,'results.mat'),'results','p','track');
fprintf('Wrote reproducible inputs and outputs to %s\n',outputDirectory);
end
