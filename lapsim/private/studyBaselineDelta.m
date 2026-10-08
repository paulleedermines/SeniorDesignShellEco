function row = studyBaselineDelta(row, baseline)
%STUDYBASELINEDELTA Add the comparison with the unmodified input vehicle.
%   Deltas are in joulemeter (terminal) Wh and are NaN unless both runs finished.
row.baseline_E_Wh = baseline.E_Wh;
row.delta_E_Wh    = row.E_Wh - baseline.E_Wh;
row.delta_E_pct   = NaN;
if isfinite(baseline.E_Wh) && baseline.E_Wh ~= 0
    row.delta_E_pct = 100 * row.delta_E_Wh / baseline.E_Wh;
end
row.baseline_time_s = baseline.time_s;
row.delta_time_s    = row.time_s - baseline.time_s;
row.baseline_feasible = baseline.feasible;
end
