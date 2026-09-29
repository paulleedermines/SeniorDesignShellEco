function row = studyBaselineDelta(row, baseline)
%STUDYBASELINEDELTA Add comparison with the unmodified input vehicle.
row.baseline_terminal_energy_Wh = baseline.terminal_energy_Wh;
row.delta_terminal_energy_Wh = ...
    row.terminal_energy_Wh - baseline.terminal_energy_Wh;
row.delta_terminal_energy_percent = NaN;
if isfinite(baseline.terminal_energy_Wh) && baseline.terminal_energy_Wh ~= 0
    row.delta_terminal_energy_percent = ...
        100 * row.delta_terminal_energy_Wh / baseline.terminal_energy_Wh;
end
row.baseline_valid = baseline.valid;
row.baseline_feasible = baseline.feasible;
row.baseline_error_reason = baseline.error_reason;
end
