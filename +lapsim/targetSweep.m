function results = targetSweep(p, track, parameterPath, values)
%TARGETSWEEP Evaluate absolute design values against the efficiency target.
%   T = lapsim.targetSweep(P, TRACK, PATH, VALUES) independently replaces one
%   scalar parameter for each value. PATH is a dot-separated field name, for
%   example 'aero.cd' or 'vehicle.chassis_mass_kg'. The target configured in P
%   is evaluated by lapsim.simulate and returned as target_met, together with
%   energy_budget_Wh, energy_margin_Wh, and all feasibility flags.
%     T = lapsim.targetSweep(p, track, 'tires.crr', [.001 .002 .003 .004]);
%
%   Results preserve the supplied order. Invalid candidates remain as rows
%   with an error_reason; values are never silently clipped to valid bounds.
%   Deltas use delivered battery-terminal Wh from the unchanged baseline.
if ~(ischar(parameterPath) && isrow(parameterPath)) && ...
        ~(isstring(parameterPath) && isscalar(parameterPath))
    error('lapsim:InvalidStudyPath', 'parameterPath must be a scalar text path.');
end
if ~isnumeric(values) || ~isreal(values) || ~isvector(values) || isempty(values)
    error('lapsim:InvalidStudyValues', 'values must be a nonempty real numeric vector.');
end
baseline = studyCase(p, track);
rows = cell(numel(values), 1);
for index = 1:numel(values)
    row = studyCase(p, track, parameterPath, values(index), false);
    row = studyBaselineDelta(row, baseline);
    row.parameter_path = string(parameterPath);
    row.requested_value = values(index);
    rows{index} = row;
end
results = struct2table(vertcat(rows{:}));
results = movevars(results, ...
    {'parameter_path', 'requested_value', 'parameter_value'}, 'Before', 1);
end
