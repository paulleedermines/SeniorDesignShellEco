function p = parameters()
%PARAMETERS  The single source of every input to lapsim.m.
%
%   p = parameters() returns one struct. Every value carries its units and
%   its source. Values set to NaN are BLANK: nobody knows them yet, and they
%   must be filled in (measurement, datasheet or team decision) before
%   lapsim will run. lapsim() refuses to run while any field is NaN and
%   lists the ones that are left.
%
%   Source tags
%     RULE    SEM 2024-26 Americas rules, as summarised in README section 4.
%             Recheck against the current year's rulebook every season.
%     LEGACY  Carried over unchanged from
%             Old Stuff/Vehicle Model/Shell_Track_Profile_Final.m (line Lnn).
%             These are the team's own values. None has a recorded source.
%     REVIEW  From the verified audit in README.md / AGENTS.md.
%     BLANK   Unknown. NaN until someone fills it in.
%
%   For a study, change a copy of the struct rather than this file:
%       p = parameters();  p.drivetrain.GR = 10;  res = lapsim(p);

MPH_TO_MS = 0.44704;   % [m/s per mph] exact definition

%% Competition rules (RULE)
p.rules.n_laps        = 4;      % [-]  laps per attempt (Ch. II Art. 226)
p.rules.d_total       = 15330;  % [m]  attempt distance (Ch. II Art. 226); lap = d_total/n_laps = 3832.5 m
p.rules.t_limit       = 2100;   % [s]  35 min for the attempt (Ch. II Art. 226)
p.rules.V_max         = 60;     % [V]  max voltage anywhere on the car (Ch. I Art. 57a)
p.rules.m_driver_min  = 50.0;   % [kg] driver incl. gear; ballast tops a lighter driver up to this (Ch. I Art. 20)
p.rules.m_vehicle_max = 140;    % [kg] vehicle without driver (README section 4)

%% Environment
p.env.g   = 9.81;   % [m/s^2] gravity. LEGACY L35
p.env.rho = 1.15;    % [kg/m^3] BLANK. Air density at IMS on race day (~220 m altitude).
                    %   Legacy 1.004 "Colorado" (L33) is wrong for Indianapolis
                    %   (README 7.1 #3); the review gives about 1.15-1.19 for IMS.

%% Mass
p.mass.m_vehicle = 33.6;  % [kg] BLANK. Car without driver (rule max 140 kg).
p.mass.m_driver  = 50;  % [kg] BLANK. Driver with race gear. lapsim adds ballast up
                         %   to p.rules.m_driver_min automatically.
                         %   Legacy (L40-41) used one total, (61+61+65)/2.205 - 1.2
                         %   = 83.6 kg. The -1.2 kg is unexplained and it is not
                         %   recorded whether the driver was on the scales
                         %   (README 7.1 #19, 7.2).

%% Vehicle body, aero and tyre-road contact
p.vehicle.n_wheels       = 3;            % [-] LEGACY L54, L91
p.vehicle.corner_weights = [61 61 65];   % [lb] FL, FR, R scale readings. LEGACY L40.
                                         %   Used ONLY for the share of weight on the
                                         %   rear (driven) wheel, so the units cancel.
                                         %   CONFIRM whether the driver was aboard.
p.vehicle.h_cg  = 0.375;   % [m] CG height. LEGACY L43-44, where it is ASSUMED equal
                           %   to the aero height (never measured).
p.vehicle.l_wb  = 1.397;   % [m] wheelbase. LEGACY L62
p.vehicle.C_rr  = 0.0056;  % [-] rolling-resistance coefficient. LEGACY L39. Unsourced;
                           %   README 7.2 recommends measuring it by coast-down.
p.vehicle.C_d   = 0.1;     % [-] drag coefficient. LEGACY L38. Unsourced.
p.vehicle.A_f   = 0.71;    % [m^2] frontal area. LEGACY L83. README 7.2 notes this is
                           %   large for a prototype; measure CdA by coast-down.
p.vehicle.mu    = 0.7;     % [-] tyre-road friction coefficient. LEGACY L66 (mu_t). Unsourced.

%% Wheels and tyres (all three wheels identical, as in the legacy model)
p.wheel.r_tire = 0.2413;   % [m] LEGACY L58 "outer radius of tire". Used as the rolling
                           %   radius; the loaded rolling radius has not been measured.
p.wheel.r_rim  = 0.203;    % [m] LEGACY L57
p.wheel.m_tire = 0.35;     % [kg] one tyre. LEGACY L59
p.wheel.m_rim  = 0.65;     % [kg] one wheel without its tyre. LEGACY L60
                           %   lapsim estimates each wheel's inertia from these four
                           %   values exactly as the legacy model did (L63-64): rim as a
                           %   solid disc, tyre as a thick annulus. Not measured.

%% Motor: Maxon EC 90 flat, 30 V winding, driven from the 60 V pack
%  Datasheet values as the legacy motor function used them (L796-801, the
%  "30 V column"). Its two adjustments are deliberately NOT carried over:
%    kt*0.9 (L797)            -> any such penalty is p.motor.eta_comm below (README 7.1 #1)
%    I0/2 and 2080*2 (L800-801) -> the no-load point stays at 30 V, where the
%                                  datasheet measured it (README 7.1 #4)
p.motor.R         = 0.275;  % [ohm] terminal resistance
p.motor.kt        = 0.136;  % [N*m/A] torque constant. lapsim uses ke = kt (SI units).
p.motor.ks        = 70.2;   % [rpm/V] speed constant. Only used to cross-check kt:
                            %   60/(2*pi*70.2) = 0.13603 V*s/rad.
p.motor.I0        = 0.490;  % [A] no-load current at 30 V. LEGACY L800.
                            %   CONFIRM on the datasheet: the legacy map section (L702)
                            %   uses 0.493 for the same quantity.
p.motor.n0        = 2080;   % [rpm] no-load speed at 30 V. LEGACY L801 (before the *2)
p.motor.frac_visc = 0.20;   % [-] share of the no-load loss that is viscous (B*w); the rest
                            %   is Coulomb friction. LEGACY L810, "This is a guess".
p.motor.I_max     = 10;     % [A] LEGACY L37 ("Current of car"). Unsourced. Applied here as
                            %   the motor current limit. CONFIRM what it really is
                            %   (controller current limit? fuse? motor rating?).
p.motor.J_rotor   = 4000e-7; % [kg*m^2] BLANK. Rotor inertia for our winding, from the
                            %   datasheet: 4000 g*cm^2 (1 g*cm^2 = 1e-7 kg*m^2). Enter it
                            %   as <datasheet value>e-7. README 7.1 #15 quotes
                            %   3,060-5,100 g*cm^2 across EC 90 flat windings.
p.motor.n_max     = Inf;    % [rpm] BLANK. Max permissible motor speed (datasheet).
                            %   Inf = no speed limit.
p.motor.eta_comm  = 1;    % [-] BLANK. Commutation penalty as a separate efficiency on
                            %   motor electrical power. Legacy cut kt by 0.9 with the note
                            %   "Maxon has a sheet that shows that when using sinusoidal
                            %   control kt is 0.9 of datasheet value" (L797); README 9
                            %   lists whether that is real as an open question.
                            %   1 = no penalty.

%% Drivetrain
p.drivetrain.GR        = 9.23;  % [-] motor-to-wheel ratio. LEGACY L48 (13T motor pulley;
                                %   README 7.2: 120/13 = 9.2308 is the raced ratio).
p.drivetrain.eta       = 0.93;  % [-] drivetrain efficiency. LEGACY L74, "CDR estimate".
                                %   Also used for back-driving when there is no freewheel.
p.drivetrain.freewheel = true;   % [true/false] BLANK. Is there a freewheel or clutch between
                                %   motor and wheel? true: the motor decouples and spins
                                %   down when coasting. false: the wheel back-drives the
                                %   motor. README 7.1 #2: this alone moves the result ~13 %.

%% Electrical: pack, controller, accessories (everything is on the one metered pack)
p.elec.V_batt   = 60;    % [V] pack voltage. LEGACY L36 (= rule max). Held constant;
                         %   there is no state-of-charge or sag model.
p.elec.eta_ctrl = 0.95;  % [-] motor controller efficiency. LEGACY L75,
                         %   "Good estimate from EE team".
p.elec.P_aux    = 0;   % [W] BLANK. Everything else the joulemeter sees: controller
                         %   standby, telemetry, dash, horn, lights... Legacy had
                         %   AccessoryPower = 0 "ADD COMMS POWER ETC HERE" (L72) and never
                         %   used it (README 7.1 #5).

%% Driving strategy: pulse and glide
p.strategy.T_pulse = 0.9;               % [N*m] motor shaft torque during a pulse. LEGACY L47.
                                        %   NB README 7.1 #16: the legacy model applied 0.9 N*m
                                        %   at the wheel with no drivetrain loss, so its motor
                                        %   really ran at 0.968 N*m. Here it is motor torque.
p.strategy.v_lo    = 16.5 * MPH_TO_MS;  % [m/s] glide ends, pulse starts. LEGACY L210
p.strategy.v_hi    = 18.5 * MPH_TO_MS;  % [m/s] pulse ends, glide starts. LEGACY L211

%% Track (README section 5, AGENTS.md "Track data pipeline")
here = fileparts(mfilename('fullpath'));
p.track.csv_file    = fullfile(here, '..', 'Old Stuff', 'Vehicle Model', 'sem_2023_us.csv');
                                  % 2023 GPS trace of one IMS lap (lat, lon, altitude)
p.track.start_lat   = 39.79315;   % [deg] start/finish line. REVIEW: about CSV s = 3,134 m,
p.track.start_lon   = -86.23871;  % [deg]   found by aligning with the legacy turn table.
                                  %   CONFIRM on site.
p.track.R_earth     = 6371000;    % [m] mean Earth radius, for the local east/north projection
p.track.ds_curv     = 1;          % [m] resampling step for curvature. REVIEW
p.track.sigma_curv  = 5;          % [m] Gaussian smoothing for curvature. REVIEW (range 3-8 m)
p.track.ds_grade    = 5;          % [m] resampling step for elevation. REVIEW
p.track.L_grade_avg = 25;         % [m] moving-average window for elevation. REVIEW
p.track.scale_to_official = false;  % [true/false] BLANK. The GPS lap is 3,849 m but the
                                  %   official lap is 3,832.5 m. true: shrink the track 0.4 %
                                  %   to the official length. false: keep the GPS geometry
                                  %   (the race still ends at p.rules.d_total).
p.track.a_y_rollover = Inf;       % [m/s^2] BLANK. Lateral acceleration at which the car would
                                  %   tip over. Inf = not limiting.
p.track.a_y_comfort  = Inf;       % [m/s^2] BLANK. Lateral acceleration the driver will hold
                                  %   in corners. Inf = not limiting.
                                  % Corner limit = min(mu*g, a_y_rollover, a_y_comfort).
                                  % README 5: at mu*g = 6.9 m/s^2 no corner limits the car;
                                  % near 0.3 g, T12, T13, T1 and T10 start to.

%% Numerics
p.sim.dt = 0.1;   % [s] time step. LEGACY L118; converged to about 0.1 % (README section 6)

end
