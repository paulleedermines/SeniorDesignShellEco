function results = strategySweep(p, track, speeds_mps, bands_mps)
%STRATEGYSWEEP Compare a finite grid of cruise and pulse-coast strategies.
%   T = lapsim.strategySweep(P, TRACK, SPEEDS_MPS, BANDS_MPS) tests every
%   speed/band pair. A zero band selects constant-speed cruise. A positive
%   band selects pulse_coast with lower/upper speeds at speed +/- band/2.
%   SPEEDS_MPS defaults to the baseline cruise speed; BANDS_MPS defaults to 0.
%     T = lapsim.strategySweep(p, track, [6 7 8], [0 1 2]);
%
%   All other parameters remain at their input values for every simulation.
%   is_best_feasible marks the lowest delivered battery-terminal Wh among
%   feasible candidates in this grid; it does not imply a global optimum.
%   Ties select the first supplied candidate. If no candidate is feasible,
%   every is_best_feasible entry is false. Invalid cases retain error_reason.
if nargin < 3 || isempty(speeds_mps)
    speeds_mps = p.strategy.cruise_speed_mps;
end
if nargin < 4 || isempty(bands_mps)
    bands_mps = 0;
end
if ~isnumeric(speeds_mps) || ~isreal(speeds_mps) || ~isvector(speeds_mps)
    error('lapsim:InvalidStudySpeeds', 'speeds_mps must be a real numeric vector.');
end
if ~isnumeric(bands_mps) || ~isreal(bands_mps) || ~isvector(bands_mps)
    error('lapsim:InvalidStudyBands', 'bands_mps must be a real numeric vector.');
end
baseline = studyCase(p, track);
rows = cell(numel(speeds_mps) * numel(bands_mps), 1);
index = 0;
for speedIndex = 1:numel(speeds_mps)
    speed = speeds_mps(speedIndex);
    for bandIndex = 1:numel(bands_mps)
        band = bands_mps(bandIndex);
        index = index + 1;
        candidate = p;
        candidate.strategy.cruise_speed_mps = speed;
        candidate.strategy.pulse_min_speed_mps = speed - band / 2;
        candidate.strategy.pulse_max_speed_mps = speed + band / 2;
        if band == 0
            candidate.strategy.mode = 'cruise';
        else
            candidate.strategy.mode = 'pulse_coast';
        end
        row = studyCase(candidate, track);
        row = studyBaselineDelta(row, baseline);
        row.strategy_mode = string(candidate.strategy.mode);
        row.speed_mps = speed;
        row.band_mps = band;
        row.pulse_min_speed_mps = candidate.strategy.pulse_min_speed_mps;
        row.pulse_max_speed_mps = candidate.strategy.pulse_max_speed_mps;
        row.is_best_feasible = false;
        rows{index} = row;
    end
end
results = struct2table(vertcat(rows{:}));
results = removevars(results, 'parameter_value');
results = movevars(results, ...
    {'strategy_mode', 'speed_mps', 'band_mps', 'pulse_min_speed_mps', ...
    'pulse_max_speed_mps', 'is_best_feasible'}, 'Before', 1);
eligible = find(results.feasible & isfinite(results.terminal_energy_Wh));
if ~isempty(eligible)
    [~, minimumIndex] = min(results.terminal_energy_Wh(eligible));
    results.is_best_feasible(eligible(minimumIndex)) = true;
end
end
