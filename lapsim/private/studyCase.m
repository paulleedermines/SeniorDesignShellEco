function row = studyCase(p, parameterPath, requestedValue, isFactor)
%STUDYCASE Run one independent candidate and keep it even if the inputs are rejected.
%   ROW = STUDYCASE(P) runs P unchanged. ROW = STUDYCASE(P, PATH, VALUE, ISFACTOR)
%   replaces the scalar parameter at the dot-separated PATH (e.g. 'vehicle.C_rr')
%   with VALUE, or with VALUE times its current value when ISFACTOR is true.
%   Each call starts from the P it is given, so changes never compound.
%
%   Energy and efficiency are NaN for a run that does not finish, so a DNF can
%   never look like an efficient one.
row = struct( ...
    'parameter_value', NaN, ...
    'valid', false, ...
    'finished', false, ...
    'stop_reason', "", ...
    'time_s', NaN, ...
    'distance_m', NaN, ...
    'E_Wh', NaN, ...
    'E_chem_Wh', NaN, ...
    'km_per_kWh', NaN, ...
    'mi_per_kWh', NaN, ...
    'max_current_A', NaN, ...
    'max_duty', NaN, ...
    'n_corner_violations', NaN, ...
    'n_brake_steps', NaN, ...
    'physically_feasible', false, ...
    'within_capacity', false, ...
    'feasible', false, ...
    'target_met', false, ...
    'energy_budget_Wh', NaN, ...
    'energy_margin_Wh', NaN, ...
    'status', "invalid", ...
    'error_identifier', "", ...
    'error_reason', "");
try
    if nargin >= 2
        parts = strsplit(char(parameterPath), '.');
        currentValue = readParameter(p, parts);
        if nargin >= 4 && isFactor
            requestedValue = currentValue * requestedValue;
        end
        row.parameter_value = requestedValue;
        p = writeParameter(p, parts, requestedValue);
    end
    sm = lapsim(p, false).summary;
    row.valid                = true;
    row.finished             = sm.finished;
    row.stop_reason          = string(sm.stop_reason);
    row.time_s               = sm.time_s;
    row.distance_m           = sm.distance_m;
    row.max_current_A        = sm.max_current_A;
    row.max_duty             = sm.max_duty;
    row.n_corner_violations  = sm.n_corner_violations;
    row.n_brake_steps        = sm.n_brake_steps;
    row.physically_feasible  = sm.physically_feasible;
    row.within_capacity      = sm.within_capacity;
    row.feasible             = sm.feasible;
    row.target_met           = sm.target_met;
    row.energy_budget_Wh     = sm.energy_budget_Wh;
    row.energy_margin_Wh     = sm.energy_margin_Wh;
    if sm.finished
        row.E_Wh        = sm.E_Wh;
        row.E_chem_Wh   = sm.E_chem_Wh;
        row.km_per_kWh  = sm.km_per_kWh;
        row.mi_per_kWh  = sm.mi_per_kWh;
    end
    if row.feasible
        row.status = "feasible";
    else
        row.status = "infeasible";
    end
catch problem
    row.error_identifier = string(problem.identifier);
    row.error_reason     = string(problem.message);
end
end

function value = readParameter(p, parts)
value = p;
for index = 1:numel(parts)
    field = parts{index};
    if ~isvarname(field) || ~isstruct(value) || ~isscalar(value) || ~isfield(value, field)
        error('lapsim:UnknownStudyParameter', ...
            'Parameter path "%s" does not identify an existing field.', strjoin(parts, '.'));
    end
    value = value.(field);
end
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value)
    error('lapsim:InvalidStudyParameter', ...
        'Parameter "%s" must be a real numeric scalar.', strjoin(parts, '.'));
end
end

function p = writeParameter(p, parts, value)
% MATLAB structs have value semantics, so every candidate starts from the p it was given
if isscalar(parts)
    p.(parts{1}) = value;
else
    p.(parts{1}) = writeParameter(p.(parts{1}), parts(2:end), value);
end
end
