function results = sensitivity(p, parameterPaths, factors)
%SENSITIVITY One-parameter-at-a-time study: scale each parameter and re-run.
%   T = sensitivity(P) scales mass, C_d, A_f, C_rr, air density and the
%   drivetrain and controller efficiencies by 0.8, 1 and 1.2.
%   T = sensitivity(P, PATHS, FACTORS) takes a cell array of dot-separated
%   field names and a vector of multipliers:
%       T = sensitivity(p, {'vehicle.C_rr', 'vehicle.C_d'}, [0.8 1 1.2]);
%
%   Every row starts from the unchanged P, so factors never compound. Rows
%   whose inputs are rejected stay in the table with status "invalid" and the
%   error text. Compare energy only between rows that finished.
%
%   A multiplier cannot move a parameter that is zero (P_aux, R_batt, the coast
%   drag and the wind speed all default to 0): use targetSweep with absolute
%   values for those. Efficiencies above 1 are rejected, so factors above 1 on
%   eta_dt / eta_ctrl show up as invalid rows rather than being clipped.
%
%   Ported from the Chat branch's lapsim.sensitivity; rewritten for this model.
if nargin < 2 || isempty(parameterPaths)
    parameterPaths = {'mass.m_vehicle', 'mass.m_driver', 'vehicle.C_d', 'vehicle.A_f', ...
        'vehicle.C_rr', 'env.rho', 'drivetrain.eta', 'elec.eta_ctrl'};
end
if nargin < 3 || isempty(factors)
    factors = [0.8 1 1.2];
end
if ischar(parameterPaths) || (isstring(parameterPaths) && isscalar(parameterPaths))
    parameterPaths = {char(parameterPaths)};
elseif isstring(parameterPaths)
    parameterPaths = cellstr(parameterPaths);
end
if ~iscellstr(parameterPaths) || ~isvector(parameterPaths) %#ok<ISCLSTR>
    error('lapsim:InvalidStudyPaths', ...
        'parameterPaths must be a cell array of character vectors or a string vector.');
end
if ~isnumeric(factors) || ~isreal(factors) || ~isvector(factors)
    error('lapsim:InvalidStudyFactors', 'factors must be a real numeric vector.');
end

baseline = studyCase(p);
rows = cell(numel(parameterPaths) * numel(factors), 1);
index = 0;
for k = 1:numel(parameterPaths)
    path = parameterPaths{k};
    for j = 1:numel(factors)
        index = index + 1;
        row = studyCase(p, path, factors(j), true);
        row = studyBaselineDelta(row, baseline);
        row.parameter_path = string(path);
        row.factor = factors(j);
        rows{index} = row;
    end
end
results = struct2table(vertcat(rows{:}));
results = movevars(results, {'parameter_path', 'factor', 'parameter_value'}, 'Before', 1);
end
