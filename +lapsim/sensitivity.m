function results = sensitivity(p, track, parameterPaths, factors)
%SENSITIVITY Run independent, one-parameter-at-a-time vehicle studies.
%   T = lapsim.sensitivity(P, TRACK) varies chassis mass, driver mass, Cd,
%   frontal area, rolling resistance, and auxiliary power by 0.8, 1, and 1.2.
%   T = lapsim.sensitivity(P, TRACK, PATHS, FACTORS) accepts a cell array of
%   dot-separated field paths (or a string array) and a real numeric vector
%   of multipliers. Each row starts from the unchanged P; factors never
%   compound across runs. For example:
%     T = lapsim.sensitivity(p, track, {'aero.cd','tires.crr'}, [.8 1 1.2]);
%
%   Energy deltas compare battery-terminal delivered Wh with the original
%   vehicle and strategy. Failed validation is retained with valid=false,
%   status="invalid", and error_reason. Feasible=false also identifies valid
%   simulations that violate completion, time, capacity, or physics limits.
%   Compare energy only between feasible runs with the same mission distance.
if nargin < 3 || isempty(parameterPaths)
    parameterPaths = {'vehicle.chassis_mass_kg', 'vehicle.driver_mass_kg', ...
        'aero.cd', 'aero.frontal_area_m2', 'tires.crr', 'auxiliary.power_W'};
end
if nargin < 4 || isempty(factors)
    factors = [.8 1 1.2];
end
if ischar(parameterPaths)
    parameterPaths = {parameterPaths};
elseif isstring(parameterPaths)
    parameterPaths = cellstr(parameterPaths);
end
if ~iscellstr(parameterPaths) || ~isvector(parameterPaths)
    error('lapsim:InvalidStudyPaths', ...
        'parameterPaths must be a cell array of character vectors or a string vector.');
end
if ~isnumeric(factors) || ~isreal(factors) || ~isvector(factors)
    error('lapsim:InvalidStudyFactors', 'factors must be a real numeric vector.');
end
baseline = studyCase(p, track);
rows = cell(numel(parameterPaths) * numel(factors), 1);
index = 0;
for parameterIndex = 1:numel(parameterPaths)
    path = parameterPaths{parameterIndex};
    for factorIndex = 1:numel(factors)
        index = index + 1;
        row = studyCase(p, track, path, factors(factorIndex), true);
        row = studyBaselineDelta(row, baseline);
        row.parameter_path = string(path);
        row.factor = factors(factorIndex);
        rows{index} = row;
    end
end
results = struct2table(vertcat(rows{:}));
results = movevars(results, {'parameter_path', 'factor', 'parameter_value'}, 'Before', 1);
end
