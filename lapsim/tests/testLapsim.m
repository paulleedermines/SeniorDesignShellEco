classdef testLapsim < matlab.unittest.TestCase
%TESTLAPSIM Regression and physics checks for lapsim.
%   Run from MATLAB:   results = runtests('lapsim/tests'); assertSuccess(results);
%   or headless:       matlab -batch "cd lapsim; r = runtests('tests'); assertSuccess(r)"
%
%   Every test starts from reference(), a fixed set of inputs written out in
%   this file, so editing parameters.m cannot move the pinned numbers. The
%   suite follows the minimum list in AGENTS.md (energy balance, distance,
%   time limit/DNF, limits, convergence) and adds checks for the features
%   ported from the Chat branch (wind, pack model, cruise, friction circle,
%   rollover, brakes, studies). Its tests are modelled on the Chat branch's
%   tests/testLapSim.m, rewritten for this model.
%
%   NOT included: the "legacy regression" test in AGENTS.md (reproduce 236.4
%   mi/kWh on a flat 3825 m lap with the legacy kt derate, I0/2 and freewheel).
%   lapsim has no switch for the legacy kt derate or I0 halving, and its lap
%   comes from the GPS track, so that test needs a legacy mode first.

    properties
        base    % result of the reference run, computed once
    end

    methods (TestClassSetup)
        function addModel(testCase)
            here = fileparts(mfilename('fullpath'));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(here)));
            warning('off', 'lapsim:rule');
            testCase.base = lapsim(testLapsim.reference(), false);
        end
    end

    methods (Static)
        function p = reference()
            % The inputs behind the recorded baseline: 1969.2 s, 40.37 Wh.
            MPH = 0.44704;
            p = parameters();
            p.env.g = 9.81;  p.env.rho = 1.15;  p.env.wind_speed = 0;  p.env.wind_from_deg = 0;
            p.mass.m_vehicle = 33.6;  p.mass.m_driver = 50;
            p.vehicle.C_rr = 0.0056;  p.vehicle.C_d = 0.1;  p.vehicle.A_f = 0.71;
            p.vehicle.mu = 0.7;  p.vehicle.h_cg = 0.375;  p.vehicle.l_wb = 1.397;
            p.vehicle.corner_weights = [61 61 65];
            p.vehicle.front_track = 0.8;  p.vehicle.F_brake_max = 400;
            p.wheel.r_tire = 0.2413;  p.wheel.r_rim = 0.203;
            p.wheel.m_tire = 0.35;  p.wheel.m_rim = 0.65;
            p.motor.R = 0.275;  p.motor.kt = 0.136;  p.motor.ks = 70.2;
            p.motor.I0 = 0.490;  p.motor.n0 = 2080;  p.motor.frac_visc = 0.20;
            p.motor.I_max = 10;  p.motor.J_rotor = 4000e-7;  p.motor.n_max = Inf;
            p.motor.T_max = Inf;  p.motor.P_shaft_max = Inf;  p.motor.eta_comm = 1;
            p.drivetrain.GR = 9.23;  p.drivetrain.eta = 0.93;
            p.drivetrain.freewheel = true;  p.drivetrain.coast_drag = 0;
            p.elec.V_batt = 60;  p.elec.R_batt = 0;  p.elec.V_min = 0;
            p.elec.I_batt_max = Inf;  p.elec.P_term_max = Inf;
            p.elec.capacity_Wh = 1000;  p.elec.eta_ctrl = 0.95;  p.elec.P_aux = 0;
            p.strategy.mode = 'pulse_glide';  p.strategy.T_pulse = 0.9;
            p.strategy.v_lo = 16.5 * MPH;  p.strategy.v_hi = 18.5 * MPH;
            p.strategy.v_cruise = 17.5 * MPH;
            p.track.scale_to_official = false;
            p.track.a_y_rollover = Inf;  p.track.a_y_comfort = Inf;
            p.sim.dt = 0.1;
        end

        function sm = simulateSummary(p)
            sm = lapsim(p, false).summary;
        end
    end

    methods (Test)
        %% Recorded baseline and bookkeeping ------------------------------
        function baselineMatchesRecordedResult(tc)
            sm = tc.base.summary;
            tc.verifyTrue(sm.finished);
            tc.verifyEqual(sm.time_s, 1969.206, 'RelTol', 1e-3);
            tc.verifyEqual(sm.E_Wh, 40.368, 'RelTol', 5e-4);
            tc.verifyEqual(sm.mi_per_kWh, 236.0, 'RelTol', 1e-3);
        end

        function energyBalanceCloses(tc)
            % Terminal energy = road work + every loss + kinetic energy, within 0.5 %
            cases = {@(q) q, ...
                     @(q) setfield(q, 'drivetrain', setfield(q.drivetrain, 'freewheel', false)), ...
                     @(q) setfield(q, 'elec', setfield(q.elec, 'P_aux', 8)), ...
                     @(q) setfield(q, 'drivetrain', setfield(q.drivetrain, 'coast_drag', 1.5)), ...
                     @(q) setfield(q, 'env', setfield(setfield(q.env, 'wind_speed', 4), 'wind_from_deg', 30)), ...
                     @(q) setfield(q, 'strategy', setfield(q.strategy, 'mode', 'cruise'))}; %#ok<SFLD>
            for k = 1:numel(cases)
                sm = testLapsim.simulateSummary(cases{k}(testLapsim.reference()));
                tc.verifyLessThan(abs(sm.energy.residual_pct), 0.5, sprintf('case %d', k));
            end
        end

        function distanceIsSetByTheRules(tc)
            sm = tc.base.summary;
            tc.verifyEqual(sm.distance_m, tc.base.params.rules.d_total, 'AbsTol', 1e-9);
            tc.verifyGreaterThanOrEqual(tc.base.ts.x(end), tc.base.params.rules.d_total);
            tc.verifyLessThan(tc.base.ts.x(end-1), tc.base.params.rules.d_total);
            % Three laps instead of four: shorter race, nothing hard-coded
            q = testLapsim.reference();
            q.rules.n_laps = 3;  q.rules.d_total = 3 * 3832.5;
            s3 = testLapsim.simulateSummary(q);
            tc.verifyEqual(s3.distance_m, 3 * 3832.5, 'AbsTol', 1e-9);
            tc.verifyEqual(s3.time_s / sm.time_s, 0.75, 'RelTol', 0.02);
        end

        function timeSeriesIsTrimmedAndStartsFromRest(tc)
            ts = tc.base.ts;
            tc.verifyEqual(numel(ts.t), numel(ts.s_lap) + 1);
            tc.verifyEqual(ts.v(1), 0);
            tc.verifyGreaterThanOrEqual(min(ts.v), 0);
            tc.verifyTrue(all(diff(ts.x) >= 0));
        end

        function trackMatchesTheAudit(tc)
            ti = tc.base.track.info;       % README section 5 / AGENTS.md track pipeline
            tc.verifyEqual(ti.L_gps, 3849, 'AbsTol', 2);
            tc.verifyEqual(ti.heading_deg, -360, 'AbsTol', 1);
            tc.verifyEqual(ti.climb, 6.3, 'AbsTol', 0.3);
            tc.verifyEqual(ti.R_min, 16.5, 'AbsTol', 1);
            tc.verifyLessThan(max(abs(ti.grade_range)), 0.03);
        end

        function scalingTheTrackGivesTheOfficialLap(tc)
            q = testLapsim.reference();  q.track.scale_to_official = true;
            r = lapsim(q, false);
            tc.verifyEqual(r.track.L, 3832.5, 'AbsTol', 1e-6);
            tc.verifyEqual(r.summary.distance_m, q.rules.d_total, 'AbsTol', 1e-9);
        end

        %% Time limit and DNF ---------------------------------------------
        function slowCarReportsDnfWithoutCrashing(tc)
            q = testLapsim.reference();
            q.strategy.T_pulse = 0.15;     % cannot even hold the glide band: too little torque
            sm = testLapsim.simulateSummary(q);
            tc.verifyFalse(sm.finished);
            tc.verifyTrue(sm.dnf);
            tc.verifyEqual(sm.stop_reason, 'time_limit');
            tc.verifyTrue(isnan(sm.time_s));
            tc.verifyTrue(isnan(sm.mi_per_kWh));
            tc.verifyLessThan(sm.distance_m, q.rules.d_total);
        end

        function finishWithinTheTimeLimit(tc)
            tc.verifyLessThanOrEqual(tc.base.summary.time_s, tc.base.params.rules.t_limit);
            tc.verifyEqual(tc.base.summary.stop_reason, 'completed');
        end

        function emptyBatteryEndsTheRun(tc)
            q = testLapsim.reference();  q.elec.capacity_Wh = 20;
            sm = testLapsim.simulateSummary(q);
            tc.verifyTrue(sm.dnf);
            tc.verifyEqual(sm.stop_reason, 'battery_capacity');
            tc.verifyEqual(sm.E_chem_Wh, 20, 'AbsTol', 0.2);
            tc.verifyFalse(sm.within_capacity);
            tc.verifyFalse(sm.feasible);
        end

        %% Hard limits -----------------------------------------------------
        function limitsAreNeverExceeded(tc)
            q = testLapsim.reference();
            q.motor.I_max = 5;  q.motor.n_max = 3000;  q.motor.T_max = 0.5;
            r = lapsim(q, false);  ts = r.ts;  sm = r.summary;
            tc.verifyLessThanOrEqual(max(ts.I), 5 + 1e-9);
            tc.verifyLessThanOrEqual(max(ts.duty), 1 + 1e-9);
            tc.verifyLessThanOrEqual(max(ts.T_motor), 0.5 + 1e-9);
            tc.verifyLessThanOrEqual(sm.max_traction_use, 1 + 1e-9);
            tc.verifyGreaterThan(sm.n_hit_current + sm.n_hit_motor_speed, 0, ...
                'a 0.5 N*m / 3000 rpm cap must bind during a 0.9 N*m pulse');
            tc.verifyLessThanOrEqual(max(ts.rpm_motor), 3000 + 1);
        end

        function dutyCycleCapBindsWhenVoltageIsLow(tc)
            q = testLapsim.reference();  q.elec.V_batt = 30;
            sm = testLapsim.simulateSummary(q);
            tc.verifyLessThanOrEqual(sm.max_duty, 1 + 1e-9);
            tc.verifyGreaterThan(sm.n_hit_duty, 0);
        end

        function tractionNeverExceedsGripLeftAfterCornering(tc)
            q = testLapsim.reference();  q.strategy.mode = 'cruise';  % hardest launch
            r = lapsim(q, false);
            tc.verifyTrue(all(r.ts.F_wheel <= r.ts.F_trac_limit + 1e-9));
            tc.verifyTrue(all(r.ts.F_trac_limit <= r.derived.F_trac + 1e-9));
            tc.verifyTrue(all(r.ts.F_trac_limit >= 0));
        end

        %% Numerics ---------------------------------------------------------
        function timeStepConverges(tc)
            q = testLapsim.reference();  q.sim.dt = 0.01;
            fine = testLapsim.simulateSummary(q);
            coarse = tc.base.summary;
            tc.verifyEqual(coarse.time_s, fine.time_s, 'RelTol', 0.005);
            tc.verifyEqual(coarse.E_Wh, fine.E_Wh, 'RelTol', 0.005);
            tc.verifyLessThan(abs(fine.energy.residual_pct), abs(coarse.energy.residual_pct) + 0.01);
        end

        %% Parameter handling ----------------------------------------------
        function currentParametersFileRuns(tc)
            r = lapsim(parameters(), false);       % whatever is in parameters.m now
            tc.verifyTrue(isstruct(r.summary));
        end

        function blankParameterIsRefused(tc)
            q = testLapsim.reference();  q.mass.m_vehicle = NaN;
            tc.verifyError(@() lapsim(q, false), 'lapsim:blankParameters');
        end

        function invalidInputsAreRejected(tc)
            R = @testLapsim.reference;
            q = R(); q.mass.m_vehicle = -1;            tc.verifyError(@() lapsim(q, false), 'lapsim:badValue');
            q = R(); q.drivetrain.eta = 1.2;           tc.verifyError(@() lapsim(q, false), 'lapsim:badValue');
            q = R(); q.strategy.v_hi = q.strategy.v_lo; tc.verifyError(@() lapsim(q, false), 'lapsim:badValue');
            q = R(); q.strategy.mode = 'sprint';       tc.verifyError(@() lapsim(q, false), 'lapsim:badValue');
            q = R(); q.elec.R_batt = -0.1;             tc.verifyError(@() lapsim(q, false), 'lapsim:badValue');
            q = R(); q.drivetrain.freewheel = 2;       tc.verifyError(@() lapsim(q, false), 'lapsim:badFlag');
        end

        function auxiliariesTheCellsCannotPowerStopTheRun(tc)
            q = testLapsim.reference();  q.elec.P_term_max = 1;  q.elec.P_aux = 5;
            tc.verifyError(@() lapsim(q, false), 'lapsim:auxInfeasible');
        end

        function rotorInertiaInWrongUnitsWarns(tc)
            q = testLapsim.reference();  q.motor.J_rotor = 4000;     % g*cm^2 entered as kg*m^2
            tc.verifyWarning(@() lapsim(q, false), 'lapsim:rotorInertia');
        end

        %% Physics: directions of the effects ------------------------------
        function energyRisesWithMassDragAndRolling(tc)
            E0 = tc.base.summary.E_Wh;
            q = testLapsim.reference();  q.mass.m_vehicle = 43.6;   tc.verifyGreaterThan(testLapsim.simulateSummary(q).E_Wh, E0);
            q = testLapsim.reference();  q.vehicle.C_d = 0.15;      tc.verifyGreaterThan(testLapsim.simulateSummary(q).E_Wh, E0);
            q = testLapsim.reference();  q.vehicle.C_rr = 0.008;    tc.verifyGreaterThan(testLapsim.simulateSummary(q).E_Wh, E0);
            q = testLapsim.reference();  q.env.rho = 1.3;           tc.verifyGreaterThan(testLapsim.simulateSummary(q).E_Wh, E0);
        end

        function auxiliaryPowerIsChargedEveryStep(tc)
            q = testLapsim.reference();  q.elec.P_aux = 10;
            sm = testLapsim.simulateSummary(q);
            expected = 10 * sm.time_s / 3600;           % [Wh] 10 W for the whole run, coasting included
            tc.verifyEqual(sm.E_Wh - tc.base.summary.E_Wh, expected, 'RelTol', 0.01);
            tc.verifyEqual(sm.energy.aux / 3600, expected, 'RelTol', 0.01);
        end

        function noFreewheelCostsEnergyAndResidualDragToo(tc)
            E0 = tc.base.summary.E_Wh;
            q = testLapsim.reference();  q.drivetrain.freewheel = false;
            tc.verifyGreaterThan(testLapsim.simulateSummary(q).E_Wh, 1.05 * E0);     % README 7.1 #2: about 13 %
            q = testLapsim.reference();  q.drivetrain.coast_drag = 2;
            sm = testLapsim.simulateSummary(q);
            tc.verifyGreaterThan(sm.E_Wh, E0);
            tc.verifyGreaterThan(sm.energy.coast_drag, 0);
        end

        function motorLossIsCalibratedAtTheDatasheetPoint(tc)
            d = tc.base.derived;
            % kt*I0 at the no-load speed equals T_f + B*w0, and kt equals ke
            tc.verifyEqual(d.T_f + d.B * d.w0, d.kt * tc.base.params.motor.I0, 'RelTol', 1e-12);
            tc.verifyEqual(d.kt, d.ke);
        end

        function pulseGlideDrivesAboutAQuarterOfTheTime(tc)
            tc.verifyGreaterThan(tc.base.summary.drive_fraction, 0.15);
            tc.verifyLessThan(tc.base.summary.drive_fraction, 0.40);
        end

        function pulseGlideBeatsCruiseAtTheSameAverageSpeed(tc)
            q = testLapsim.reference();  q.strategy.mode = 'cruise';
            cruise = testLapsim.simulateSummary(q);
            tc.verifyTrue(cruise.finished);
            tc.verifyGreaterThan(cruise.E_Wh, tc.base.summary.E_Wh);
        end

        function cruiseHoldsItsSpeed(tc)
            q = testLapsim.reference();  q.strategy.mode = 'cruise';
            r = lapsim(q, false);
            late = r.ts.t > 60;
            tc.verifyGreaterThan(min(r.ts.v(late)), q.strategy.v_cruise - 0.15);
            tc.verifyLessThan(max(r.ts.v), q.strategy.v_cruise + 0.6);  % downhill can run a little over
            tc.verifyGreaterThan(r.summary.drive_fraction, 0.5);
        end

        %% Wind --------------------------------------------------------------
        function windAlongTheTrackAveragesToZeroAroundTheLoop(tc)
            q = testLapsim.reference();  q.env.wind_speed = 4;  q.env.wind_from_deg = 30;
            hw = lapsim(q, false).track.headwind;
            tc.verifyEqual(max(hw), 4, 'AbsTol', 0.05);
            tc.verifyEqual(min(hw), -4, 'AbsTol', 0.05);
            tc.verifyLessThan(abs(mean(hw)), 0.3);
            tc.verifyEqual(max(abs(tc.base.track.headwind)), 0);   % calm
        end

        function windCostsEnergyOnALoop(tc)
            % Drag grows with airspeed squared, so the headwind half costs more
            % than the tailwind half gives back
            E0 = tc.base.summary.E_Wh;
            for from = [0 90]
                q = testLapsim.reference();  q.env.wind_speed = 4;  q.env.wind_from_deg = from;
                tc.verifyGreaterThan(testLapsim.simulateSummary(q).E_Wh, E0, sprintf('wind from %d', from));
            end
        end

        function aeroForceIsSignedWithTheAirspeed(tc)
            % A tailwind faster than the car pushes it: aero work is negative there
            q = testLapsim.reference();  q.env.wind_speed = 15;  q.env.wind_from_deg = 0;
            r = lapsim(q, false);
            tc.verifyLessThan(min(r.ts.F_aero), 0);
            tc.verifyGreaterThan(max(r.ts.F_aero), 0);
        end

        %% Pack ---------------------------------------------------------------
        function packResistanceIsUpstreamOfTheJoulemeter(tc)
            q = testLapsim.reference();  q.elec.R_batt = 0.15;
            sm = testLapsim.simulateSummary(q);
            tc.verifyEqual(sm.E_Wh, tc.base.summary.E_Wh, 'RelTol', 1e-9);   % score unchanged
            tc.verifyGreaterThan(sm.E_chem_Wh, sm.E_Wh);                       % cells do more work
            tc.verifyEqual(sm.energy.pack_residual, 0, 'AbsTol', 1e-6);        % chemical = terminal + I^2*R
            tc.verifyLessThan(sm.min_terminal_V, 60);
            tc.verifyGreaterThan(sm.max_duty, tc.base.summary.max_duty);       % sag raises the duty cycle
        end

        function packLimitsHoldIncludingTheFreewheelSpinUp(tc)
            q = testLapsim.reference();  q.elec.R_batt = 0.15;  q.elec.I_batt_max = 5;
            r = lapsim(q, false);
            tc.verifyLessThanOrEqual(max(r.ts.I_batt), 5 + 1e-6);
            q = testLapsim.reference();  q.elec.P_term_max = 300;
            r = lapsim(q, false);
            tc.verifyLessThanOrEqual(max(r.ts.P_batt), 300 + 1e-6);
            tc.verifyTrue(r.summary.finished);
        end

        function lowTerminalVoltageLimitBinds(tc)
            q = testLapsim.reference();  q.elec.R_batt = 0.15;  q.elec.V_min = 58;
            r = lapsim(q, false);
            tc.verifyGreaterThanOrEqual(r.summary.min_terminal_V, 58 - 1e-6);
        end

        %% Corners: friction circle, rollover, brakes -------------------------
        function frictionCircleShrinksTractionInCorners(tc)
            r = tc.base;
            tc.verifyLessThan(min(r.ts.F_trac_limit), r.derived.F_trac - 1);
        end

        function narrowerTrackTipsOverSooner(tc)
            q = testLapsim.reference();  q.vehicle.front_track = 0.4;
            wide = tc.base.derived.a_y_roll;
            narrow = lapsim(q, false).derived.a_y_roll;
            tc.verifyLessThan(narrow, wide);
            tc.verifyEqual(narrow / wide, 0.5, 'RelTol', 1e-9);     % linear in track width
        end

        function cornerLimitCostsTimeNotFeasibility(tc)
            q = testLapsim.reference();  q.track.a_y_comfort = 2.0;
            r = lapsim(q, false);
            tc.verifyTrue(r.summary.feasible);
            tc.verifyGreaterThan(r.summary.n_brake_steps, 0);
            tc.verifyGreaterThan(r.summary.time_s, tc.base.summary.time_s);
        end

        function tooWeakBrakesAreFlaggedInfeasible(tc)
            q = testLapsim.reference();  q.track.a_y_comfort = 2.0;  q.vehicle.F_brake_max = 5;
            sm = testLapsim.simulateSummary(q);
            tc.verifyGreaterThan(sm.n_corner_violations, 0);
            tc.verifyFalse(sm.physically_feasible);
            tc.verifyFalse(sm.feasible);
        end

        %% Studies --------------------------------------------------------------
        function sensitivityRowsStartFromTheBaselineAndNeverCompound(tc)
            T = sensitivity(testLapsim.reference(), {'vehicle.C_rr', 'vehicle.C_d'}, [0.8 1 1.2]);
            tc.verifyEqual(height(T), 6);
            ones_ = T.factor == 1;
            tc.verifyEqual(T.delta_E_Wh(ones_), zeros(2, 1), 'AbsTol', 1e-12);
            tc.verifyEqual(T.parameter_value(T.parameter_path == "vehicle.C_rr" & T.factor == 1.2), ...
                1.2 * 0.0056, 'RelTol', 1e-12);
            tc.verifyLessThan(T.delta_E_Wh(T.parameter_path == "vehicle.C_rr" & T.factor == 0.8), 0);
            tc.verifyGreaterThan(T.delta_E_Wh(T.parameter_path == "vehicle.C_rr" & T.factor == 1.2), 0);
        end

        function sweepsKeepRejectedCasesAsRows(tc)
            T = sensitivity(testLapsim.reference(), {'drivetrain.eta'}, [1 1.2]);   % 0.93*1.2 > 1
            tc.verifyEqual(T.status(1), "feasible");
            tc.verifyEqual(T.status(2), "invalid");
            tc.verifyEqual(T.error_identifier(2), "lapsim:badValue");
            tc.verifyTrue(isnan(T.E_Wh(2)));
            U = targetSweep(testLapsim.reference(), 'no.such.field', 1);
            tc.verifyEqual(U.status, "invalid");
            tc.verifyEqual(U.error_identifier, "lapsim:UnknownStudyParameter");
        end

        function targetSweepKeepsOrderAndMeasuresTheGoal(tc)
            vals = [0.0075 0.002 0.0056];
            T = targetSweep(testLapsim.reference(), 'vehicle.C_rr', vals);
            tc.verifyEqual(T.requested_value, vals(:));
            tc.verifyLessThan(T.E_Wh(2), T.E_Wh(3));
            tc.verifyLessThan(T.E_Wh(3), T.E_Wh(1));
            tc.verifyEqual(T.energy_margin_Wh(3), T.energy_budget_Wh(3) - T.E_Wh(3), 'AbsTol', 1e-9);
            tc.verifyEqual(T.target_met, T.E_Wh <= T.energy_budget_Wh & T.feasible);
        end

        function dnfRowsCarryNoEfficiency(tc)
            T = targetSweep(testLapsim.reference(), 'strategy.T_pulse', 0.15);
            tc.verifyFalse(T.finished);
            tc.verifyTrue(isnan(T.E_Wh));
            tc.verifyTrue(isnan(T.mi_per_kWh));
            tc.verifyFalse(T.target_met);
        end

        function strategySweepMarksOneBestFeasibleCandidate(tc)
            MPH = 0.44704;
            T = strategySweep(testLapsim.reference(), [17 18] * MPH, [0 2] * MPH);
            tc.verifyEqual(height(T), 4);
            tc.verifyEqual(T.strategy_mode(T.band_mps == 0), ["cruise"; "cruise"]);
            tc.verifyEqual(T.strategy_mode(T.band_mps > 0), ["pulse_glide"; "pulse_glide"]);
            tc.verifyEqual(nnz(T.is_best_feasible), 1);
            best = T.E_Wh(T.is_best_feasible);
            tc.verifyEqual(best, min(T.E_Wh(T.feasible)));
        end

        %% Plotting and the entry script ---------------------------------------
        function plotDrawsWithoutSideEffects(tc)
            before = get(groot, 'DefaultFigureVisible');
            fig = plotResult(tc.base, 'off');
            cleanup = onCleanup(@() close(fig));
            tc.verifyEqual(fig.Visible, matlab.lang.OnOffSwitchState.off);
            tc.verifyEqual(get(groot, 'DefaultFigureVisible'), before);
        end

        function baselineScriptWritesItsOutputs(tc)
            folder = tc.createTemporaryFolder();
            out = runBaseline(folder, false);
            tc.verifyEqual(out.baseline.summary.E_Wh, lapsim(parameters(), false).summary.E_Wh, 'RelTol', 1e-12);
            for name = ["summary.csv" "trace.csv" "energy.csv" "baseline.png" "results.mat"]
                tc.verifyTrue(isfile(fullfile(folder, name)), name);
            end
        end
    end
end
