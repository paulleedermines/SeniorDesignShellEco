function results = targetSweep(p, parameterPath, values)
%TARGETSWEEP Try absolute values of one parameter against the efficiency goal.
%   T = targetSweep(P, PATH, VALUES) replaces the scalar parameter at the
%   dot-separated PATH with each of VALUES in turn and reports, for each, the
%   finish time, joulemeter energy, mi/kWh, and whether p.goal.target_mi_per_kWh
%   is met (target_met, energy_budget_Wh, energy_margin_Wh):
%       T = targetSweep(p, 'vehicle.C_rr', [0.002 0.003 0.0056]);
%       T = targetSweep(p, 'elec.P_aux', [0 2 5 10 20]);
%
%   Rows keep the order supplied. Invalid values stay in the table with an
%   error_reason and are never clipped. Deltas are against the unchanged P.
%
%   Ported from the Chat branch's lapsim.targetSweep.
if ~(ischar(parameterPath) && isrow(parameterPath)) ...
        && ~(isstring(parameterPath) && isscalar(parameterPath))
    error('lapsim:InvalidStudyPath', 'parameterPath must be a scalar text path.');
end
if ~isnumeric(values) || ~isreal(values) || ~isvector(values) || isempty(values)
    error('lapsim:InvalidStudyValues', 'values must be a nonempty real numeric vector.');
end

baseline = studyCase(p);
rows = cell(numel(values), 1);
for k = 1:numel(values)
    row = studyCase(p, parameterPath, values(k), false);
    row = studyBaselineDelta(row, baseline);
    row.parameter_path = string(parameterPath);
    row.requested_value = values(k);
    rows{k} = row;
end
results = struct2table(vertcat(rows{:}));
results = movevars(results, {'parameter_path', 'requested_value', 'parameter_value'}, ...
    'Before', 1);
end
