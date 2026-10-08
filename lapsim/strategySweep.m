function results = strategySweep(p, speeds_mps, bands_mps)
%STRATEGYSWEEP Compare a grid of cruise and pulse-and-glide strategies.
%   T = strategySweep(P, SPEEDS, BANDS) runs every speed/band pair [m/s].
%   A band of 0 is constant-speed cruise at that speed. A positive band is
%   pulse-and-glide between speed - band/2 and speed + band/2. SPEEDS defaults
%   to P's cruise speed and BANDS to 0:
%       T = strategySweep(p, [7 7.5 8], [0 0.5 1]);
%
%   Everything else, including the pulse torque, stays as in P. is_best_feasible
%   marks the lowest joulemeter energy among the feasible candidates *on this
%   grid*. It is not a global optimum. Ties go to the first candidate; if none
%   is feasible, no row is marked.
%
%   Ported from the Chat branch's lapsim.strategySweep.
if nargin < 2 || isempty(speeds_mps)
    speeds_mps = p.strategy.v_cruise;
end
if nargin < 3 || isempty(bands_mps)
    bands_mps = 0;
end
if ~isnumeric(speeds_mps) || ~isreal(speeds_mps) || ~isvector(speeds_mps)
    error('lapsim:InvalidStudySpeeds', 'speeds must be a real numeric vector.');
end
if ~isnumeric(bands_mps) || ~isreal(bands_mps) || ~isvector(bands_mps)
    error('lapsim:InvalidStudyBands', 'bands must be a real numeric vector.');
end

baseline = studyCase(p);
rows = cell(numel(speeds_mps) * numel(bands_mps), 1);
index = 0;
for i = 1:numel(speeds_mps)
    speed = speeds_mps(i);
    for j = 1:numel(bands_mps)
        band = bands_mps(j);
        index = index + 1;
        q = p;
        q.strategy.v_cruise = speed;
        if band == 0
            q.strategy.mode = 'cruise';
        else
            q.strategy.mode = 'pulse_glide';
            q.strategy.v_lo = speed - band / 2;
            q.strategy.v_hi = speed + band / 2;
        end
        row = studyCase(q);
        row = studyBaselineDelta(row, baseline);
        row.strategy_mode = string(q.strategy.mode);
        row.speed_mps = speed;
        row.band_mps = band;
        row.v_lo_mps = speed - band / 2;
        row.v_hi_mps = speed + band / 2;
        row.is_best_feasible = false;
        rows{index} = row;
    end
end
results = struct2table(vertcat(rows{:}));
results = removevars(results, 'parameter_value');
results = movevars(results, {'strategy_mode', 'speed_mps', 'band_mps', 'v_lo_mps', ...
    'v_hi_mps', 'is_best_feasible'}, 'Before', 1);
eligible = find(results.feasible & isfinite(results.E_Wh));
if ~isempty(eligible)
    [~, best] = min(results.E_Wh(eligible));
    results.is_best_feasible(eligible(best)) = true;
end
end
