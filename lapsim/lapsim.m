function res = lapsim(p, verbose)
%LAPSIM  Lap simulator for the Mines Shell Eco-marathon battery-electric Prototype.
%
%   res = lapsim()           run with parameters() and print a summary
%   res = lapsim(p)          run with the parameter struct p (see parameters.m)
%   res = lapsim(p, false)   run without printing (for sweeps)
%
%   Simulates one attempt at Indianapolis (n_laps laps, standing start, no
%   stops) on the GPS track and reports finish time, energy at the
%   joulemeter, km/kWh and mi/kWh, and whether every operating point was
%   physically possible.
%
%   res.summary   headline numbers, limit checks and the energy balance
%   res.ts        time series, one row per time step (trimmed to the run)
%   res.track     the processed track
%   res.derived   quantities derived from p (masses, motor losses, limits)
%   res.params    the parameters that were used
%
%   MODEL (SI units throughout; AGENTS.md "Physics the new model must get right")
%   Mass        m    = m_vehicle + m_driver + ballast (tops the driver up to 50 kg)
%               m_eq = m + n_wheels*I_wheel/r^2  [+ J_rotor*GR^2/r^2 while coupled]
%   Road load   F    = C_rr*m*g*cos(th) + 0.5*rho*C_d*A_f*v^2 + m*g*sin(th),
%               th = atan(grade)
%   Motor       T_e = T + T_f + B*w,  I = T_e/kt,  V_eff = I*R + ke*w,
%               D = V_eff/V_batt,  P_elec = V_eff*I,  ke = kt.
%               T_f and B come from the 30 V no-load point:
%               T_f + B*w0 = kt*I0, split (1-frac_visc) : frac_visc.
%   Drivetrain  w = GR*v/r,  F_wheel = T*GR*eta_dt/r  (torque is commanded and
%               the force follows from it)
%   Coasting    Controller off, I = 0.
%               Freewheel: the motor decouples and spins down on its own
%               friction; re-engaging costs 0.5*J*(w^2 - w_rotor^2), taken
%               from the battery through eta_comm*eta_ctrl (motor copper loss
%               during the spin-up is not modelled, so this is a lower bound).
%               No freewheel: back-drive drag (T_f + B*w)*GR/(r*eta_dt).
%   Battery     P_batt = P_elec/(eta_comm*eta_ctrl) + P_aux, charged every step,
%               coasting included. No regeneration.
%   Strategy    Pulse at T_pulse once v <= v_lo, glide once v >= v_hi. Never
%               drive above min(v_hi, corner envelope). Partial torque is used
%               only to land the last step of a pulse exactly on v_hi, or to
%               hold a corner's speed limit. Approaching a corner (where the
%               envelope is a coast-down curve), the car lifts and coasts
%               unless a full pulse step still fits under the envelope. Brakes
%               are used only when coasting alone would exceed the envelope.
%   Limits      Applied after the strategy, as hard caps on motor torque:
%               motor current <= I_max, duty cycle D <= 1, wheel force <=
%               traction limit, motor speed <= n_max. Every step where a cap
%               binds is counted.
%   Traction    F_max = mu*f_r*m*g/(1 - mu*h_cg/l_wb): rear (driven) wheel,
%               static share f_r plus load transfer, with acceleration taken as
%               F/m (the legacy closed form, Shell_Track_Profile_Final.m L88).
%   Corners     v_lim = sqrt(a_y_max/|kappa|), a_y_max = min(mu*g, a_y_rollover,
%               a_y_comfort). The envelope is a backward pass of the coast-down
%               from every corner (held flat where coasting would speed up).
%   Integrator  Semi-implicit Euler (v first, then x with the new v). The race
%               ends by distance; the loop is bounded by the time limit and a
%               car that does not finish is reported as a DNF.

if nargin < 1 || isempty(p)
    p = parameters();
end
if nargin < 2
    verbose = true;
end

checkParams(p);
d   = derivedQuantities(p);
if p.elec.V_batt < p.elec.V_min || p.elec.P_aux > d.Pt_cap
    error('lapsim:auxInfeasible', ['The pack cannot even power the auxiliaries ' ...
        '(P_aux = %.1f W, terminal power cap %.1f W at %.1f V).'], ...
        p.elec.P_aux, d.Pt_cap, p.elec.V_batt);
end
trk = loadTrack(p, d);
trk.v_env = speedEnvelope(trk, p, d);
trk.coast_zone = trk.v_env < trk.v_lim;   % envelope set by a corner further on
[ts, summary] = simulate(p, d, trk);

res.summary = summary;
res.ts      = ts;
res.track   = trk;
res.derived = d;
res.params  = p;

if verbose
    printSummary(res);
end
end

%% =====================================================================
%  Parameter checks
%  =====================================================================
function checkParams(p)
blanks = findBlanks(p, 'p');
if ~isempty(blanks)
    error('lapsim:blankParameters', '%s', sprintf( ...
        ['%d parameter(s) in parameters.m are still BLANK (NaN). ' ...
         'Fill these in first:\n  %s'], numel(blanks), strjoin(blanks, [newline '  '])));
end

flags = {'p.drivetrain.freewheel', p.drivetrain.freewheel; ...
         'p.track.scale_to_official', p.track.scale_to_official};
for k = 1:size(flags, 1)
    val = flags{k, 2};
    if ~isscalar(val) || ~(islogical(val) || val == 0 || val == 1)
        error('lapsim:badFlag', '%s must be true or false.', flags{k, 1});
    end
end

positive = {'p.env.g', p.env.g; 'p.env.rho', p.env.rho; ...
            'p.mass.m_vehicle', p.mass.m_vehicle; 'p.mass.m_driver', p.mass.m_driver; ...
            'p.vehicle.h_cg', p.vehicle.h_cg; 'p.vehicle.l_wb', p.vehicle.l_wb; ...
            'p.vehicle.C_rr', p.vehicle.C_rr; 'p.vehicle.C_d', p.vehicle.C_d; ...
            'p.vehicle.A_f', p.vehicle.A_f; 'p.vehicle.mu', p.vehicle.mu; ...
            'p.wheel.r_tire', p.wheel.r_tire; 'p.wheel.r_rim', p.wheel.r_rim; ...
            'p.motor.R', p.motor.R; 'p.motor.kt', p.motor.kt; 'p.motor.ks', p.motor.ks; ...
            'p.motor.I0', p.motor.I0; 'p.motor.n0', p.motor.n0; ...
            'p.motor.I_max', p.motor.I_max; 'p.motor.n_max', p.motor.n_max; ...
            'p.drivetrain.GR', p.drivetrain.GR; 'p.elec.V_batt', p.elec.V_batt; ...
            'p.strategy.T_pulse', p.strategy.T_pulse; 'p.strategy.v_lo', p.strategy.v_lo; ...
            'p.track.a_y_rollover', p.track.a_y_rollover; ...
            'p.track.a_y_comfort', p.track.a_y_comfort; 'p.sim.dt', p.sim.dt; ...
            'p.vehicle.front_track', p.vehicle.front_track; ...
            'p.motor.T_max', p.motor.T_max; 'p.motor.P_shaft_max', p.motor.P_shaft_max; ...
            'p.elec.I_batt_max', p.elec.I_batt_max; 'p.elec.P_term_max', p.elec.P_term_max; ...
            'p.elec.capacity_Wh', p.elec.capacity_Wh; 'p.strategy.v_cruise', p.strategy.v_cruise; ...
            'p.goal.target_mi_per_kWh', p.goal.target_mi_per_kWh};
for k = 1:size(positive, 1)
    if ~(positive{k, 2} > 0)
        error('lapsim:badValue', '%s must be > 0.', positive{k, 1});
    end
end
nonNegative = {'p.wheel.m_tire', p.wheel.m_tire; 'p.wheel.m_rim', p.wheel.m_rim; ...
               'p.motor.J_rotor', p.motor.J_rotor; 'p.elec.P_aux', p.elec.P_aux; ...
               'p.env.wind_speed', p.env.wind_speed; 'p.drivetrain.coast_drag', p.drivetrain.coast_drag; ...
               'p.elec.R_batt', p.elec.R_batt; 'p.elec.V_min', p.elec.V_min; ...
               'p.vehicle.F_brake_max', p.vehicle.F_brake_max};
for k = 1:size(nonNegative, 1)
    if ~(nonNegative{k, 2} >= 0)
        error('lapsim:badValue', '%s must be >= 0.', nonNegative{k, 1});
    end
end
effs = {'p.motor.eta_comm', p.motor.eta_comm; 'p.drivetrain.eta', p.drivetrain.eta; ...
        'p.elec.eta_ctrl', p.elec.eta_ctrl};
for k = 1:size(effs, 1)
    if ~(effs{k, 2} > 0 && effs{k, 2} <= 1)
        error('lapsim:badValue', '%s must be in (0, 1].', effs{k, 1});
    end
end
if ~all(p.vehicle.corner_weights > 0)
    error('lapsim:badValue', 'p.vehicle.corner_weights must all be > 0.');
end
if ~(p.motor.frac_visc >= 0 && p.motor.frac_visc <= 1)
    error('lapsim:badValue', 'p.motor.frac_visc must be in [0, 1].');
end
if ~(p.strategy.v_hi > p.strategy.v_lo)
    error('lapsim:badValue', 'p.strategy.v_hi must be greater than p.strategy.v_lo.');
end
if ~(ischar(p.strategy.mode) || (isstring(p.strategy.mode) && isscalar(p.strategy.mode))) ...
        || ~any(strcmp(char(p.strategy.mode), {'pulse_glide', 'cruise'}))
    error('lapsim:badValue', 'p.strategy.mode must be ''pulse_glide'' or ''cruise''.');
end
if ~(p.vehicle.mu * p.vehicle.h_cg / p.vehicle.l_wb < 1)
    error('lapsim:badValue', 'mu*h_cg/l_wb must be < 1 for the traction limit to exist.');
end

% Rule checks: warn, so that out-of-rule designs can still be studied
if p.elec.V_batt > p.rules.V_max
    warning('lapsim:rule', 'V_batt = %.1f V exceeds the %.0f V rule limit.', ...
        p.elec.V_batt, p.rules.V_max);
end
if p.mass.m_vehicle > p.rules.m_vehicle_max
    warning('lapsim:rule', 'm_vehicle = %.1f kg exceeds the %.0f kg rule limit.', ...
        p.mass.m_vehicle, p.rules.m_vehicle_max);
end

% A rotor inertia above 0.01 kg*m^2 (100,000 g*cm^2) is almost certainly a
% unit slip: the datasheet gives g*cm^2, and 1 g*cm^2 = 1e-7 kg*m^2
if p.motor.J_rotor > 0.01
    warning('lapsim:rotorInertia', ['p.motor.J_rotor = %g kg*m^2 is implausibly large for ' ...
        'an EC 90 flat (about 4e-4). The datasheet gives g*cm^2; multiply by 1e-7.'], ...
        p.motor.J_rotor);
end

% kt and ke are the same constant in SI units; check the datasheet agrees
ke_from_ks = 60 / (2*pi*p.motor.ks);                 % [V*s/rad]
if abs(ke_from_ks - p.motor.kt) / p.motor.kt > 0.02
    warning('lapsim:ktke', ['kt = %.4f N*m/A but 60/(2*pi*ks) = %.4f V*s/rad. In SI units ' ...
        'these must be equal; check the datasheet values.'], p.motor.kt, ke_from_ks);
end
end

function list = findBlanks(s, prefix)
% Paths of every numeric field of struct s that contains NaN
list = {};
names = fieldnames(s);
for k = 1:numel(names)
    val  = s.(names{k});
    path = [prefix '.' names{k}];
    if isstruct(val)
        list = [list, findBlanks(val, path)]; %#ok<AGROW>
    elseif isnumeric(val) && any(isnan(val(:)))
        list{end+1} = path; %#ok<AGROW>
    end
end
end

%% =====================================================================
%  Derived quantities
%  =====================================================================
function d = derivedQuantities(p)
% Mass
d.ballast = max(0, p.rules.m_driver_min - p.mass.m_driver);       % [kg]
d.m       = p.mass.m_vehicle + p.mass.m_driver + d.ballast;        % [kg] total mass

% Rotating inertia, referred to the car as equivalent mass
d.r       = p.wheel.r_tire;                                        % [m] rolling radius
I_rim     = 0.5 * p.wheel.m_rim * p.wheel.r_rim^2;                 % [kg*m^2] solid disc (legacy L63)
I_tire    = 0.5 * p.wheel.m_tire * (p.wheel.r_tire^2 + p.wheel.r_rim^2); % [kg*m^2] annulus (legacy L64)
d.I_wheel = I_rim + I_tire;                                        % [kg*m^2] one wheel
d.m_eq_free    = d.m + p.vehicle.n_wheels * d.I_wheel / d.r^2;     % [kg] motor decoupled
d.m_rotor_eq   = p.motor.J_rotor * p.drivetrain.GR^2 / d.r^2;      % [kg]
d.m_eq_coupled = d.m_eq_free + d.m_rotor_eq;                       % [kg] motor coupled

% Motor
d.kt = p.motor.kt;                                 % [N*m/A]
d.ke = p.motor.kt;                                 % [V*s/rad] equal to kt in SI units
d.ke_from_ks = 60 / (2*pi*p.motor.ks);             % [V*s/rad] datasheet cross-check only
d.w0   = p.motor.n0 * 2*pi/60;                     % [rad/s] no-load speed at 30 V
d.T_nl = d.kt * p.motor.I0;                        % [N*m] total loss torque at w0
d.T_f  = (1 - p.motor.frac_visc) * d.T_nl;         % [N*m] Coulomb friction torque
d.B    = p.motor.frac_visc * d.T_nl / d.w0;        % [N*m*s/rad] viscous coefficient
d.w_max    = p.motor.n_max * 2*pi/60;              % [rad/s]
d.v_w_max  = d.w_max * d.r / p.drivetrain.GR;      % [m/s] car speed at max motor speed
d.eta_elec = p.motor.eta_comm * p.elec.eta_ctrl;   % [-] battery -> motor terminals

% Traction limit of the rear (driven) wheel, straight line (the simulation also
% shrinks mu by the friction circle in corners)
w = p.vehicle.corner_weights;
d.f_rear  = w(end) / sum(w);                                       % [-] static share on rear
d.F_trac  = p.vehicle.mu * d.f_rear * d.m * p.env.g ...
            / (1 - p.vehicle.mu * p.vehicle.h_cg / p.vehicle.l_wb); % [N]
d.F_brake_max = min(p.vehicle.F_brake_max, p.vehicle.mu * d.m * p.env.g);  % [N]

% Rollover: the support triangle is the two front contacts and the single rear
% wheel, so it narrows towards the rear. Half-width at the CG:
x_cg      = p.vehicle.l_wb * d.f_rear;                             % [m] CG behind the front axle
half_w    = 0.5 * p.vehicle.front_track * (1 - x_cg / p.vehicle.l_wb);  % [m]
d.a_y_roll = p.env.g * half_w / p.vehicle.h_cg;                    % [m/s^2] tips over at this

% Battery envelope at the terminals. Stable branch of P = (Voc - Rb*I)*I
Voc = p.elec.V_batt;  Rb = p.elec.R_batt;
if Voc < p.elec.V_min
    d.Ib_cap = 0;  d.Pt_cap = 0;
elseif Rb > 0
    d.Ib_cap = min([p.elec.I_batt_max, Voc / (2*Rb), (Voc - p.elec.V_min) / Rb]);   % [A]
    d.Pt_cap = min(p.elec.P_term_max, (Voc - Rb * d.Ib_cap) * d.Ib_cap);            % [W]
else
    d.Ib_cap = p.elec.I_batt_max;                                                   % [A]
    d.Pt_cap = min(p.elec.P_term_max, Voc * p.elec.I_batt_max);                     % [W]
end

% Corners and race
d.a_y_max    = min([p.vehicle.mu * p.env.g, d.a_y_roll, p.track.a_y_rollover, p.track.a_y_comfort]); % [m/s^2]
d.L_official = p.rules.d_total / p.rules.n_laps;                   % [m] official lap
d.E_budget_Wh = p.rules.d_total / 1609.344 / p.goal.target_mi_per_kWh * 1000;  % [Wh] for the goal
end

%% =====================================================================
%  Track: GPS CSV -> grade, curvature and corner speed limit on a 1 m grid
%  =====================================================================
function trk = loadTrack(p, d)
raw = readmatrix(p.track.csv_file, 'NumHeaderLines', 1);
lat = raw(:, 1);  lon = raw(:, 2);  alt = raw(:, 3);   % [deg], [deg], [m]

% 1. Local east/north projection about the mean latitude [m]
lat0 = mean(lat);  lon0 = mean(lon);  Re = p.track.R_earth;
E = Re * cosd(lat0) * deg2rad(lon - lon0);
N = Re * deg2rad(lat - lat0);

% 2. The trace is one closed lap: treat it as periodic, so the last point
%    joins the first (a 0.97 m segment, shorter than the usual 1.43 m).
seg = hypot(diff([E; E(1)]), diff([N; N(1)]));      % [m] includes last -> first
if any(seg <= 0)
    error('lapsim:track', 'Track file has repeated points.');
end
L_gps = sum(seg);                                   % [m] arc length from the raw points
s_raw = [0; cumsum(seg(1:end-1))];                  % [m] CSV arc length of each point

% 5. Start/finish line: the GPS point nearest the given coordinates
Es = Re * cosd(lat0) * deg2rad(p.track.start_lon - lon0);
Ns = Re * deg2rad(p.track.start_lat - lat0);
[snap, k0] = min(hypot(E - Es, N - Ns));
s0 = s_raw(k0);                                     % [m] CSV arc length of the start line

% Periodic interpolation along the lap, measured from the start line
s_ext   = [s_raw; L_gps];
alongLap = @(f, s) interp1(s_ext, [f; f(1)], mod(s0 + s, L_gps));

% 3. Curvature: resample at ~1 m, Gaussian smoothing with wrap-around
N1  = round(L_gps / p.track.ds_curv);
ds1 = L_gps / N1;                                   % [m] exact grid step
s1  = (0:N1-1)' * ds1;                              % [m] lap coordinate (0 = start line)
sig = p.track.sigma_curv / ds1;                     % [samples]
h   = ceil(4 * sig);
kern = exp(-0.5 * ((-h:h)' / sig).^2);
kern = kern / sum(kern);
xs = circSmooth(alongLap(E, s1), kern);
ys = circSmooth(alongLap(N, s1), kern);
dx  = (circshift(xs, -1) - circshift(xs, 1)) / (2*ds1);
dy  = (circshift(ys, -1) - circshift(ys, 1)) / (2*ds1);
ddx = (circshift(xs, -1) - 2*xs + circshift(xs, 1)) / ds1^2;
ddy = (circshift(ys, -1) - 2*ys + circshift(ys, 1)) / ds1^2;
kappa = (dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^1.5;   % [1/m] left turn > 0
psi   = atan2(dy, dx);                              % [rad] direction of travel, x = east, y = north
% Wind component along the direction of travel (positive = headwind). The wind
% blows FROM wind_from_deg, so its velocity points the opposite way.
w_east  = -p.env.wind_speed * sind(p.env.wind_from_deg);    % [m/s]
w_north = -p.env.wind_speed * cosd(p.env.wind_from_deg);    % [m/s]
headwind = -(w_east * cos(psi) + w_north * sin(psi));       % [m/s]
heading_total = sum(angle(exp(1i * diff([psi; psi(1)])))); % [rad] -2*pi for clockwise
if abs(abs(rad2deg(heading_total)) - 360) > 2
    error('lapsim:track', 'Total heading change is %.1f deg, not +/-360 deg.', ...
        rad2deg(heading_total));
end

% 4. Grade: resample at ~5 m, centred moving average with wrap-around
N5  = round(L_gps / p.track.ds_grade);
ds5 = L_gps / N5;                                   % [m]
s5  = (0:N5-1)' * ds5;
win = round(p.track.L_grade_avg / ds5);             % [samples]
if mod(win, 2) == 0
    error('lapsim:track', ['L_grade_avg must be an odd multiple of ds_grade so the ' ...
        'moving average is centred.']);
end
z5 = circSmooth(alongLap(alt, s5), ones(win, 1) / win);   % [m]
g5 = (circshift(z5, -1) - circshift(z5, 1)) / (2*ds5);    % [-] dz/ds
climb = sum(max(0, diff([z5; z5(1)])));                    % [m] climb per lap
grade = interp1([s5; L_gps], [g5; g5(1)], s1);
z     = interp1([s5; L_gps], [z5; z5(1)], s1);

% 7. Optionally shrink the GPS lap to the official lap length
if p.track.scale_to_official
    k = d.L_official / L_gps;
else
    k = 1;
end

trk.L     = L_gps * k;          % [m] lap length used for track lookups
trk.ds    = ds1 * k;            % [m] grid step
trk.s     = s1 * k;             % [m] lap coordinate, 0 = start/finish line
trk.z     = z;                  % [m] smoothed elevation
trk.grade = grade / k;          % [-] dz/ds
trk.kappa = kappa / k;          % [1/m] signed curvature, left > 0
trk.headwind = headwind;        % [m/s] wind along the direction of travel, headwind > 0

% 6. Corner speed limit
trk.v_lim = sqrt(d.a_y_max ./ abs(trk.kappa));     % [m/s] Inf on a true straight

% Diagnostics (README section 5 values: 3,849 m, -360 deg, ~6.3 m climb, ~+/-2 %)
[kmax, iR]           = max(abs(trk.kappa));
trk.info.L_gps       = L_gps;                       % [m]
trk.info.scale       = k;                           % [-]
trk.info.s0_csv      = s0;                          % [m] start line in CSV arc length
trk.info.start_snap  = snap;                        % [m] start coords to nearest GPS point
trk.info.heading_deg = rad2deg(heading_total);      % [deg]
trk.info.climb       = climb;                       % [m] per lap
trk.info.grade_range = [min(trk.grade), max(trk.grade)];  % [-]
trk.info.R_min       = 1 / kmax;                    % [m]
trk.info.s_R_min     = trk.s(iR);                   % [m]
end

function y = circSmooth(x, kern)
% Convolution of a periodic column x with an odd-length kernel, wrapping around
h  = (numel(kern) - 1) / 2;
xp = [x(end-h+1:end); x; x(1:h)];
y  = conv(xp, kern(:), 'valid');
end

%% =====================================================================
%  Road load and back-drive drag (shared by the envelope and the simulation)
%  =====================================================================
function [F_rr, F_aero, F_grade] = roadLoad(v, grade, hw, p, d)
% hw = headwind along the direction of travel [m/s]; negative for a tailwind.
% F_aero is signed: a tailwind faster than the car pushes it forwards.
th      = atan(grade);                                             % [rad]
v_air   = v + hw;                                                  % [m/s] airspeed
F_rr    = p.vehicle.C_rr * d.m * p.env.g * cos(th);                % [N]
F_aero  = 0.5 * p.env.rho * p.vehicle.C_d * p.vehicle.A_f * v_air * abs(v_air);   % [N]
F_grade = d.m * p.env.g * sin(th);                                 % [N]
end

function F_bd = backdriveForce(v, p, d)
% Drag at the wheel when coasting turns an unpowered motor (no freewheel)
w    = p.drivetrain.GR * v / d.r;                                  % [rad/s]
F_bd = (d.T_f + d.B * w) * p.drivetrain.GR / (d.r * p.drivetrain.eta);  % [N]
end

%% =====================================================================
%  Corner speed envelope: backward coast-down passes around the lap
%  =====================================================================
function v_env = speedEnvelope(trk, p, d)
freewheel = logical(p.drivetrain.freewheel);
if freewheel
    m_eq = d.m_eq_free;
else
    m_eq = d.m_eq_coupled;
end
n = numel(trk.v_lim);
v_env = trk.v_lim;
for pass = 1:2                      % two passes so limits carry across the start line
    for k = n:-1:1
        kn = mod(k, n) + 1;
        vn = v_env(kn);
        if isinf(vn)
            continue
        end
        [F_rr, F_aero, F_grade] = roadLoad(vn, trk.grade(k), trk.headwind(k), p, d);
        F = F_rr + F_aero + F_grade;
        if freewheel
            F = F + p.drivetrain.coast_drag;
        else
            F = F + backdriveForce(vn, p, d);
        end
        decel = max(F, 0) / m_eq;   % [m/s^2] held flat where coasting would speed up
        vc = sqrt(vn^2 + 2 * decel * trk.ds);
        if vc < v_env(k)
            v_env(k) = vc;
        end
    end
end
end

%% =====================================================================
%  Time-domain simulation of one attempt
%  =====================================================================
function [ts, sm] = simulate(p, d, trk)
V_TOL = 1e-9;       % [m/s] numerical tolerance for "speed has reached the top of the band"
VIOL_TOL = 1e-3;    % [m/s] numerical tolerance for corner-limit violations

dt      = p.sim.dt;
nSteps  = floor(p.rules.t_limit / dt + 1e-9);
d_total = p.rules.d_total;
L  = trk.L;   ds = trk.ds;

GR = p.drivetrain.GR;   eta_dt = p.drivetrain.eta;   r = d.r;
kF = GR * eta_dt / r;   % [N per N*m] wheel force per unit motor torque
kW = GR / r;            % [rad/s per m/s] motor speed per unit car speed
kt = d.kt;  ke = d.ke;  R = p.motor.R;  T_f = d.T_f;  B = d.B;  J = p.motor.J_rotor;
V_batt = p.elec.V_batt;   Rb = p.elec.R_batt;   eta_elec = d.eta_elec;
P_aux  = p.elec.P_aux;    v_w_max  = d.v_w_max;
E_cap_J = p.elec.capacity_Wh * 3600;              % [J] usable chemical energy
m_eq_c = d.m_eq_coupled;  m_eq_f = d.m_eq_free;
T_pulse = p.strategy.T_pulse;  v_lo = p.strategy.v_lo;  v_hi = p.strategy.v_hi;
cruise = strcmp(char(p.strategy.mode), 'cruise');
if cruise
    v_top = p.strategy.v_cruise;                  % [m/s] speed the cruise holds
else
    v_top = v_hi;                                 % [m/s] top of the pulse band
end
freewheel = logical(p.drivetrain.freewheel);
mu = p.vehicle.mu;  g = p.env.g;  m = d.m;  f_rear = d.f_rear;
h_cg = p.vehicle.h_cg;  l_wb = p.vehicle.l_wb;

% States (nSteps+1) and per-step values (nSteps)
t = zeros(nSteps+1, 1);  x = zeros(nSteps+1, 1);  v = zeros(nSteps+1, 1);
E_cum  = zeros(nSteps+1, 1);                      % [J] energy at the terminals (joulemeter) so far
E_chem = zeros(nSteps+1, 1);                      % [J] chemical energy drawn from the cells so far
z0 = zeros(nSteps, 1);
[T_m, w_m, I_m, V_eff, Duty, P_elec, P_batt, F_wheel, F_rr, F_aero, F_grade, ...
    F_bd, F_cdrag, F_brake, a, s_lap, v_lim, v_env, I_b, V_term, P_chem, F_trac_n, hw_log] = deal(z0);
[drive, coupled, hitI, hitV, hitTrac, hitSpeed] = deal(false(nSteps, 1));

E = struct('rolling', 0, 'aero', 0, 'grade', 0, 'controller', 0, 'copper', 0, ...
           'motor_friction', 0, 'drivetrain', 0, 'brakes', 0, 'coast_drag', 0, 'aux', 0, ...
           'pack', 0);

driving     = true;     % standing start: first pulse from rest
was_coupled = false;    % motor at rest
w_r         = 0;        % [rad/s] rotor speed (matters only while decoupled)
n_engage    = 0;
finished    = false;
capacity_hit = false;
nLast       = nSteps;

for n = 1:nSteps
    vn = v(n);  xn = x(n);
    s  = mod(xn, L);
    s_nx    = mod(xn + vn*dt, L);             % [m] predicted position one step ahead
    grd     = lookupLin(trk.grade, s, ds);
    hw      = lookupLin(trk.headwind, s, ds);
    kap     = lookupLin(trk.kappa, s, ds);
    venv_nx = lookupLin(trk.v_env, s_nx, ds);
    zone_nx = lookupAny(trk.coast_zone, s_nx, ds);
    [Frr, Fa, Fg] = roadLoad(vn, grd, hw, p, d);
    F_road = Frr + Fa + Fg;
    w_c    = kW * vn;                    % [rad/s] motor speed when coupled
    T_loss = T_f + B * w_c;              % [N*m] motor friction + windage at w_c

    % Tyre grip left for the driven/braked direction: the friction circle takes
    % the lateral share v^2*kappa/g out of mu
    cth  = cos(atan(grd));
    a_y  = vn^2 * abs(kap);                                          % [m/s^2]
    mu_l = sqrt(max(0, mu^2 - (a_y / (g * cth))^2));                 % [-]
    F_tr = mu_l * f_rear * m * g * cth / (1 - mu_l * h_cg / l_wb);   % [N] rear-wheel traction
    F_br_max = min(p.vehicle.F_brake_max, mu_l * m * g * cth);       % [N] total braking force

    % 1. Strategy
    if cruise
        T_cmd = Inf;                       % ask for whatever holds v_cruise
    else
        % pulse-and-glide hysteresis
        if driving && vn >= v_hi - V_TOL
            driving = false;
        elseif ~driving && vn <= v_lo
            driving = true;
        end
        T_cmd = driving * T_pulse;
    end
    % Never drive above the top of the band or the corner envelope.
    % T_tgt is the torque that lands exactly on v_tgt at the next step.
    v_tgt = min(v_top, venv_nx);
    T_tgt = (m_eq_c * (v_tgt - vn) / dt + F_road) / kF;
    if T_cmd > T_tgt                       % a full pulse step would overshoot
        if zone_nx && venv_nx < v_top
            T_cmd = 0;                     % approaching a corner: lift and coast
        else
            T_cmd = max(T_tgt, 0);         % land on the top speed, or hold the corner limit
        end
    end

    % 2. Hard limits, applied after the strategy
    P_extra = 0;                           % [W] terminal power the rotor spin-up would add
    if freewheel && ~was_coupled
        P_extra = max(0, 0.5 * J * (w_c^2 - w_r^2)) / eta_elec / dt;
    end
    [I_lim_m, I_lim_p] = currentLimits(w_c, T_loss, P_extra, p, d);
    T_lim_I   = kt * I_lim_m - T_loss;                     % motor current, torque, shaft power
    T_lim_V   = kt * I_lim_p - T_loss;                     % duty cycle <= 1 and the pack envelope
    T_lim_tr  = F_tr / kF;                                 % traction
    T_lim_spd = (m_eq_c * (v_w_max - vn) / dt + F_road) / kF;  % motor speed next step
    T = max(0, min([T_cmd, T_lim_I, T_lim_V, T_lim_tr, T_lim_spd]));
    if T_cmd > 0
        hitI(n)     = T_lim_I   < T_cmd;
        hitV(n)     = T_lim_V   < T_cmd;
        hitTrac(n)  = T_lim_tr  < T_cmd;
        hitSpeed(n) = T_lim_spd < T_cmd;
    end

    % 3. Motor, drivetrain and coasting
    E_spin = 0;                                            % [J] rotor spin-up this step
    Fcd = 0;                                               % [N] residual drag, freewheel coasting
    if T > 0
        cpl = true;   wm = w_c;
        I   = (T + T_loss) / kt;
        Ve  = I * R + ke * wm;
        Pel = Ve * I;
        Fw  = kF * T;   Fbd = 0;
        if freewheel && ~was_coupled
            E_spin   = max(0, 0.5 * J * (wm^2 - w_r^2));
            n_engage = n_engage + 1;
        end
    else
        I = 0;  Pel = 0;  Fw = 0;
        if freewheel
            cpl = false;  wm = w_r;  Fbd = 0;
            if vn > 0
                Fcd = p.drivetrain.coast_drag;
            end
        else
            cpl = true;   wm = w_c;  Fbd = backdriveForce(vn, p, d);
        end
        Ve = ke * wm;                                      % open-circuit back-EMF
    end
    if cpl
        m_eq = m_eq_c;
    else
        m_eq = m_eq_f;
    end

    % Brakes: only if coasting alone would leave the car above the envelope
    Fb = 0;
    if T == 0
        v_coast = vn - dt * (F_road + Fbd + Fcd) / m_eq;
        if v_coast > venv_nx
            Fb = min(m_eq * (v_coast - venv_nx) / dt, F_br_max);
        end
    end

    % 4. Semi-implicit Euler
    acc = (Fw - F_road - Fbd - Fcd - Fb) / m_eq;
    v(n+1) = max(0, vn + acc * dt);      % held at rest rather than rolling backwards
    x(n+1) = xn + v(n+1) * dt;
    t(n+1) = n * dt;

    % Rotor: rides with the wheel when coupled, spins down on its own otherwise
    P_spin = 0;
    if cpl
        w_r = kW * v(n+1);
    elseif w_r > 0 && J > 0
        w_old  = w_r;
        w_r    = max(0, w_old - (T_f + B * w_old) / J * dt);
        P_spin = 0.5 * J * (w_old^2 - w_r^2) / dt;   % rotor KE lost to its friction
    else
        w_r = 0;
    end
    was_coupled = cpl;

    % 5. Energy at the joulemeter (terminals) and where it goes [J]
    Pb = Pel / eta_elec + P_aux + E_spin / eta_elec / dt;  % [W] terminal power
    [Ib, Vt] = batteryPoint(V_batt, Rb, Pb);               % pack current and terminal voltage
    Pc = V_batt * Ib;                                      % [W] chemical power; Pc - Pb = Rb*Ib^2
    E_cum(n+1)  = E_cum(n)  + Pb * dt;
    E_chem(n+1) = E_chem(n) + Pc * dt;
    E.rolling    = E.rolling + Frr * vn * dt;
    E.aero       = E.aero    + Fa  * vn * dt;
    E.grade      = E.grade   + Fg  * vn * dt;
    E.controller = E.controller + (Pel + E_spin/dt) * (1/eta_elec - 1) * dt;
    E.copper     = E.copper  + I^2 * R * dt;
    E.brakes     = E.brakes  + Fb * vn * dt;
    E.coast_drag = E.coast_drag + Fcd * vn * dt;
    E.aux        = E.aux     + P_aux * dt;
    E.pack       = E.pack    + Rb * Ib^2 * dt;
    if T > 0
        E.motor_friction = E.motor_friction + T_loss * wm * dt;
        E.drivetrain     = E.drivetrain + (1 - eta_dt) * T * wm * dt;
    elseif cpl            % back-driven: motor loss plus drivetrain loss
        E.motor_friction = E.motor_friction + T_loss * wm * dt;
        E.drivetrain     = E.drivetrain + (Fbd * vn - T_loss * wm) * dt;
    else                  % decoupled rotor spinning down
        E.motor_friction = E.motor_friction + P_spin * dt;
    end

    % Log this step
    T_m(n) = T;  w_m(n) = wm;  I_m(n) = I;  V_eff(n) = Ve;
    Duty(n) = (T > 0) * Ve / Vt;
    P_elec(n) = Pel;  P_batt(n) = Pb;  F_wheel(n) = Fw;
    I_b(n) = Ib;  V_term(n) = Vt;  P_chem(n) = Pc;
    F_rr(n) = Frr;  F_aero(n) = Fa;  F_grade(n) = Fg;  F_bd(n) = Fbd;  F_cdrag(n) = Fcd;
    F_brake(n) = Fb;  F_trac_n(n) = F_tr;  hw_log(n) = hw;
    a(n) = acc;  s_lap(n) = s;  drive(n) = T > 0;  coupled(n) = cpl;
    v_lim(n) = lookupLin(trk.v_lim, s, ds);  v_env(n) = lookupLin(trk.v_env, s, ds);

    % 6. Finish by distance; interpolate inside the last step
    if x(n+1) >= d_total
        f = (d_total - xn) / (x(n+1) - xn);
        t_finish = t(n) + f * dt;
        E_finish = E_cum(n) + f * (E_cum(n+1) - E_cum(n));
        E_chem_finish = E_chem(n) + f * (E_chem(n+1) - E_chem(n));
        finished = true;
        nLast = n;
        break
    end
    % 7. Out of usable battery energy: the run ends here (DNF)
    if E_chem(n+1) >= E_cap_J
        capacity_hit = true;
        nLast = n;
        break
    end
end

% Trim to the simulated length
iS = 1:nLast+1;   iP = 1:nLast;
ts.t = t(iS);  ts.x = x(iS);  ts.v = v(iS);  ts.E_batt = E_cum(iS);  ts.E_chem = E_chem(iS);
ts.s_lap = s_lap(iP);  ts.a = a(iP);  ts.drive = drive(iP);  ts.coupled = coupled(iP);
ts.T_motor = T_m(iP);  ts.omega_motor = w_m(iP);  ts.rpm_motor = w_m(iP) * 60/(2*pi);
ts.I = I_m(iP);  ts.V_eff = V_eff(iP);  ts.duty = Duty(iP);
ts.P_elec = P_elec(iP);  ts.P_batt = P_batt(iP);  ts.P_chem = P_chem(iP);
ts.I_batt = I_b(iP);  ts.V_term = V_term(iP);
ts.F_wheel = F_wheel(iP);  ts.F_rr = F_rr(iP);  ts.F_aero = F_aero(iP);
ts.F_grade = F_grade(iP);  ts.F_backdrive = F_bd(iP);  ts.F_coast_drag = F_cdrag(iP);
ts.F_brake = F_brake(iP);  ts.F_trac_limit = F_trac_n(iP);  ts.headwind = hw_log(iP);
ts.v_lim = v_lim(iP);  ts.v_env = v_env(iP);
ts.hit_current = hitI(iP);  ts.hit_duty = hitV(iP);
ts.hit_traction = hitTrac(iP);  ts.hit_motor_speed = hitSpeed(iP);

% Summary
M_TO_MI = 1 / 1609.344;
sm.finished = finished && t_finish <= p.rules.t_limit;
sm.dnf      = ~sm.finished;
if sm.finished
    sm.stop_reason = 'completed';
elseif capacity_hit
    sm.stop_reason = 'battery_capacity';
else
    sm.stop_reason = 'time_limit';
end
if finished
    sm.time_s = t_finish;
    sm.E_J    = E_finish;
    sm.E_chem_J = E_chem_finish;
else
    sm.time_s = NaN;
    sm.E_J    = E_cum(nLast+1);
    sm.E_chem_J = E_chem(nLast+1);
end
sm.time_min   = sm.time_s / 60;
sm.distance_m = min(x(nLast+1), d_total);
sm.E_Wh       = sm.E_J / 3600;
sm.E_kWh      = sm.E_J / 3.6e6;
sm.E_chem_Wh  = sm.E_chem_J / 3600;
if sm.finished
    sm.km_per_kWh = (d_total / 1000) / sm.E_kWh;
    sm.mi_per_kWh = (d_total * M_TO_MI) / sm.E_kWh;
else
    sm.km_per_kWh = NaN;
    sm.mi_per_kWh = NaN;
end
if finished
    sm.v_avg = d_total / sm.time_s;            % [m/s]
else
    sm.v_avg = sm.distance_m / t(nLast+1);     % [m/s]
end

% Feasibility
sm.max_current_A     = max(ts.I);
sm.max_duty          = max(ts.duty);
sm.max_motor_rpm     = max(ts.rpm_motor);
sm.max_wheel_force_N = max(ts.F_wheel);
sm.traction_limit_N  = d.F_trac;
sm.max_traction_use  = max(ts.F_wheel ./ ts.F_trac_limit);    % [-] <= 1 means within grip
sm.max_batt_current_A = max(ts.I_batt);
sm.min_terminal_V    = min(ts.V_term);
sm.max_terminal_W    = max(ts.P_batt);
coast_cpl = ~ts.drive & ts.coupled;
sm.max_backEMF_coasting_V = max([0; ts.V_eff(coast_cpl)]);
sm.n_hit_current     = nnz(ts.hit_current);
sm.n_hit_duty        = nnz(ts.hit_duty);
sm.n_hit_traction    = nnz(ts.hit_traction);
sm.n_hit_motor_speed = nnz(ts.hit_motor_speed);
sm.n_overspeed       = nnz(ts.omega_motor > d.w_max);   % e.g. back-driven downhill
sm.n_backEMF_over_Vbatt = nnz(coast_cpl & ts.V_eff > V_batt);
sm.max_over_v_lim    = max(ts.v(1:end-1) - ts.v_lim);   % [m/s] <= 0 means never above
sm.n_corner_violations = nnz(ts.v(1:end-1) > ts.v_lim + VIOL_TOL);
sm.n_brake_steps     = nnz(ts.F_brake > 0);
sm.drive_fraction    = mean(ts.drive);
sm.n_engagements     = n_engage;
% "Physically possible": the car never exceeded a corner limit, a motor-speed
% limit, or the pack voltage while back-driven. (The caps keep current, duty
% and traction in range by construction, so they cannot fail here.)
sm.physically_feasible = sm.n_corner_violations == 0 && sm.n_overspeed == 0 ...
                         && sm.n_backEMF_over_Vbatt == 0;
sm.within_capacity   = sm.E_chem_Wh <= p.elec.capacity_Wh && ~capacity_hit;
sm.energy_budget_Wh  = d.E_budget_Wh;                     % [Wh] allowed by the team goal
if sm.finished
    sm.energy_margin_Wh = sm.energy_budget_Wh - sm.E_Wh;  % [Wh] > 0 means the goal is met
else
    sm.energy_margin_Wh = NaN;
end
sm.feasible   = sm.finished && sm.physically_feasible && sm.within_capacity;
sm.target_met = sm.feasible && sm.E_Wh <= sm.energy_budget_Wh;

% Energy balance over the simulated steps:
% terminal energy = road work + net grade work + every loss + kinetic energy gained
KE_end = 0.5 * m_eq_f * v(nLast+1)^2 + 0.5 * J * w_r^2;     % [J] KE_start = 0
E.kinetic_gain = KE_end;
E.battery      = E_cum(nLast+1);
E.chemical     = E_chem(nLast+1);
sinks = E.rolling + E.aero + E.grade + E.controller + E.copper + E.motor_friction ...
      + E.drivetrain + E.brakes + E.coast_drag + E.aux + E.kinetic_gain;
E.residual     = E.battery - sinks;
E.residual_pct = 100 * E.residual / E.battery;
% The pack's own I^2*R is upstream of the joulemeter: chemical = terminal + pack
E.pack_residual = E.chemical - E.battery - E.pack;
sm.energy = E;
end

function [I_m, I_p] = currentLimits(w, T_loss, P_extra, p, d)
% Largest winding current allowed at motor speed w [rad/s], in two parts so the
% caller can report which kind of limit bound:
%   I_m  motor side: current rating, usable torque, usable shaft power
%   I_p  pack side:  duty cycle <= 1 at the loaded terminal voltage, plus the
%                    pack's current / power / minimum-voltage envelope
% P_extra [W] is terminal power already committed this step (the rotor spin-up
% when a freewheel re-engages), so the pack envelope covers it too.
kt = d.kt;  R = p.motor.R;  emf = d.ke * w;
Voc = p.elec.V_batt;  Rb = p.elec.R_batt;

I_m = min(p.motor.I_max, (p.motor.T_max + T_loss) / kt);
if w > 0
    I_m = min(I_m, (p.motor.P_shaft_max / w + T_loss) / kt);
end

I_p = (Voc - emf) / R;                       % duty cycle <= 1 with an ideal pack
if isfinite(d.Pt_cap)                        % motor power the terminals can supply
    Pm  = max(0, (d.Pt_cap - p.elec.P_aux - P_extra) * d.eta_elec);   % [W]
    den = emf + sqrt(emf^2 + 4 * R * Pm);
    if den > 0
        I_p = min(I_p, 2 * Pm / den);
    else
        I_p = 0;
    end
end
I_nl = T_loss / kt;                          % [A] current that only feeds motor losses
if Rb > 0 && I_p > I_nl
    % Terminal voltage sags with load: find the largest current whose motor
    % voltage still fits under it (the residual rises monotonically with I)
    if sagResidual(I_p, emf, P_extra, p, d) > 0
        if sagResidual(I_nl, emf, P_extra, p, d) > 0
            I_p = 0;                         % not even the no-load current fits
        else
            lo = I_nl;  hi = I_p;
            for k = 1:40
                mid = 0.5 * (lo + hi);
                if sagResidual(mid, emf, P_extra, p, d) <= 0
                    lo = mid;
                else
                    hi = mid;
                end
            end
            I_p = lo;
        end
    end
end
end

function res = sagResidual(I, emf, P_extra, p, d)
% Motor voltage minus loaded terminal voltage at winding current I (<= 0 is feasible)
Pt = p.elec.P_aux + P_extra + (emf + p.motor.R * I) * I / d.eta_elec;
[~, Vt] = batteryPoint(p.elec.V_batt, p.elec.R_batt, Pt);
res = emf + p.motor.R * I - Vt;
end

function [I, V] = batteryPoint(Voc, Rb, Pt)
% Pack current and terminal voltage that deliver terminal power Pt.
% Stable (lower-current) root of Pt = (Voc - Rb*I)*I.
if Rb == 0
    I = Pt / Voc;
    V = Voc;
else
    I = 2 * Pt / (Voc + sqrt(max(0, Voc^2 - 4 * Rb * Pt)));
    V = Voc - Rb * I;
end
end

function val = lookupLin(arr, s, ds)
% Linear interpolation on a periodic grid with step ds (arr(1) at s = 0).
% Next to an Inf (no corner limit) it returns the smaller neighbour.
n  = numel(arr);
k  = s / ds;
i0 = floor(k);
f  = k - i0;
i0 = mod(i0, n);
i1 = mod(i0 + 1, n);
a0 = arr(i0+1);  a1 = arr(i1+1);
if isinf(a0) || isinf(a1)
    val = min(a0, a1);
else
    val = a0 + f * (a1 - a0);
end
end

function tf = lookupAny(arr, s, ds)
% True if either grid point around s is true (periodic logical grid)
n  = numel(arr);
i0 = mod(floor(s / ds), n);
i1 = mod(i0 + 1, n);
tf = arr(i0+1) || arr(i1+1);
end

%% =====================================================================
%  Printed summary
%  =====================================================================
function printSummary(res)
sm = res.summary;  p = res.params;  d = res.derived;  ti = res.track.info;  E = sm.energy;

fprintf('\n=== lapsim: %d laps, %.0f m, limit %.0f s ===\n', ...
    p.rules.n_laps, p.rules.d_total, p.rules.t_limit);
if sm.finished
    fprintf('Result       FINISHED in %.1f s (%.2f min), average %.2f m/s\n', ...
        sm.time_s, sm.time_min, sm.v_avg);
    fprintf('Energy       %.2f Wh = %.5f kWh at the joulemeter\n', sm.E_Wh, sm.E_kWh);
    fprintf('Efficiency   %.1f km/kWh = %.1f mi/kWh\n', sm.km_per_kWh, sm.mi_per_kWh);
else
    fprintf('Result       DNF (%s): %.0f of %.0f m covered in %.0f s\n', ...
        strrep(sm.stop_reason, '_', ' '), sm.distance_m, p.rules.d_total, p.rules.t_limit);
    fprintf('Energy       %.2f Wh used before the run ended\n', sm.E_Wh);
end
if sm.finished
    fprintf('Goal         %.0f mi/kWh allows %.2f Wh: margin %+.2f Wh, %s\n', ...
        p.goal.target_mi_per_kWh, sm.energy_budget_Wh, sm.energy_margin_Wh, ...
        iff(sm.target_met, 'goal met', 'goal NOT met'));
end

fprintf('\nFeasibility\n');
fprintf('  max motor current  %6.2f A     (limit %.1f A, capped on %d steps)\n', ...
    sm.max_current_A, p.motor.I_max, sm.n_hit_current);
fprintf('  max duty cycle     %6.3f       (limit 1, capped on %d steps)\n', ...
    sm.max_duty, sm.n_hit_duty);
fprintf('  max motor speed    %6.0f rpm   (limit %.0f rpm, capped on %d steps, over on %d)\n', ...
    sm.max_motor_rpm, p.motor.n_max, sm.n_hit_motor_speed, sm.n_overspeed);
fprintf('  max wheel force    %6.1f N     (traction limit %.1f N, capped on %d steps)\n', ...
    sm.max_wheel_force_N, sm.traction_limit_N, sm.n_hit_traction);
fprintf('  corner limit       max v - v_lim = %+.3f m/s, %d steps over\n', ...
    sm.max_over_v_lim, sm.n_corner_violations);
if ~p.drivetrain.freewheel
    fprintf('  back-EMF coasting  max %.1f V (pack %.1f V, over on %d steps)\n', ...
        sm.max_backEMF_coasting_V, p.elec.V_batt, sm.n_backEMF_over_Vbatt);
end
fprintf('  traction use       max wheel force / grip left after cornering = %.2f\n', sm.max_traction_use);
if p.elec.R_batt > 0 || isfinite(d.Pt_cap) || p.elec.V_min > 0
    fprintf('  pack               max %.2f A, min terminal %.1f V, max %.0f W (cap %.0f W); chemical %.2f Wh of %.0f Wh\n', ...
        sm.max_batt_current_A, sm.min_terminal_V, sm.max_terminal_W, d.Pt_cap, ...
        sm.E_chem_Wh, p.elec.capacity_Wh);
end
fprintf('  brakes             used on %d steps, %.1f J\n', sm.n_brake_steps, E.brakes);
fprintf('  motor driving      %.1f %% of the time, %d freewheel engagements\n', ...
    100 * sm.drive_fraction, sm.n_engagements);

fprintf('\nEnergy balance (simulated steps)       kJ      %% of battery\n');
rows = {'rolling resistance', E.rolling; 'aero drag', E.aero; ...
        'grade (net climb)', E.grade; 'controller + commutation', E.controller; ...
        'motor copper', E.copper; 'motor friction + windage', E.motor_friction; ...
        'drivetrain', E.drivetrain; 'brakes', E.brakes; 'coast drag', E.coast_drag; ...
        'auxiliary', E.aux; 'kinetic energy at the end', E.kinetic_gain};
for k = 1:size(rows, 1)
    fprintf('  %-28s %10.2f %10.2f\n', rows{k, 1}, rows{k, 2}/1e3, 100*rows{k, 2}/E.battery);
end
fprintf('  %-28s %10.2f   (terminals, i.e. the joulemeter)\n', 'battery', E.battery/1e3);
fprintf('  %-28s %10.3f %10.3f   (closes to this)\n', 'residual', ...
    E.residual/1e3, E.residual_pct);

fprintf('\nCar          m = %.2f kg (ballast %.2f kg), m_eq = %.2f kg coupled / %.2f kg free\n', ...
    d.m, d.ballast, d.m_eq_coupled, d.m_eq_free);
fprintf('Motor        T_f = %.4f N*m, B = %.3e N*m*s/rad, kt = ke = %.4f (ks gives %.4f)\n', ...
    d.T_f, d.B, d.kt, d.ke_from_ks);
fprintf('Track        GPS lap %.1f m, used %.1f m; start at CSV s = %.1f m (%.1f m snap)\n', ...
    ti.L_gps, res.track.L, ti.s0_csv, ti.start_snap);
fprintf('             heading %.1f deg, climb %.2f m/lap, grade %+.2f..%+.2f %%\n', ...
    ti.heading_deg, ti.climb, 100*ti.grade_range(1), 100*ti.grade_range(2));
fprintf('             tightest R = %.1f m at s = %.0f m; corner a_y limit %.2f m/s^2\n', ...
    ti.R_min, ti.s_R_min, d.a_y_max);
fprintf('             corner limit = min(grip %.2f, rollover %.2f, set %.2f, comfort %.2f) m/s^2\n', ...
    p.vehicle.mu * p.env.g, d.a_y_roll, p.track.a_y_rollover, p.track.a_y_comfort);
if p.env.wind_speed > 0
    fprintf('Wind         %.1f m/s from %.0f deg: along-track %+.1f..%+.1f m/s (headwind > 0)\n', ...
        p.env.wind_speed, p.env.wind_from_deg, min(res.track.headwind), max(res.track.headwind));
end
fprintf('Strategy     %s\n', strrep(char(p.strategy.mode), '_', ' '));
if sm.energy.pack ~= 0
    fprintf('Pack         internal loss %.2f kJ upstream of the joulemeter (chemical - terminal = %.3f kJ)\n', ...
        sm.energy.pack/1e3, (sm.energy.chemical - sm.energy.battery)/1e3);
end
end

function out = iff(cond, a, b)
if cond, out = a; else, out = b; end
end
