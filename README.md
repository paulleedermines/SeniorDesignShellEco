# Shell Eco-marathon LapSim

A toolbox-free MATLAB vehicle simulation for setting battery-electric efficiency targets. The new entry point is `runLapSim.m`; the original scripts are preserved in `Old Stuff/` and explained in [the legacy review](docs/legacy-model-review.md).

The configured mission comes from your requirements: **four consecutive laps, 15.33 km total, no more than 35 minutes, and a 300 mi/kWh target**. That target permits **31.752 Wh** at the battery terminals. The additional **16.6 mph minimum average** is an editable planning constraint; the quoted distance/time requirement alone implies 16.33 mph. Average speed allows a standing start and slower corners. The 50 kg value is treated as your minimum **combined** mass, not a measured baseline mass or an independently verified competition rule.

## Run it

In MATLAB, change to this repository and run:

```matlab
results = runLapSim;  % baseline + sensitivity, driving-strategy, and target studies
% Faster baseline only:
results = runLapSim('results/baseline', false);
% Regression checks:
tests = runtests('tests'); assertSuccess(tests);
```

`results/latest/` contains a speed/energy plot, the full trace, energy breakdown, study CSVs, and a MAT file containing the exact inputs. Generated outputs are ignored by Git. Tested on MATLAB R2026a; no Simulink or add-on toolbox is required.

## Tune the car

```matlab
p = lapsim.defaultParameters();
track = lapsim.exampleTrack();
p.vehicle.chassis_mass_kg = 35;  % entire car incl. wheels, battery, drivetrain
p.vehicle.driver_mass_kg = 50;  % total simulated mass is the sum
p.aero.cd = 0.10;
p.aero.frontal_area_m2 = 0.60;
p.tires.crr = 0.003;
p.vehicle.cg_height_m = 0.30;
p.vehicle.cg_from_front_m = 0.50; % combined car + driver CG, measured from front axle
p.auxiliary.power_W = 3;
p.race.target_mi_per_kWh = 300;
r = lapsim.simulate(p, track);
disp(r.summary)
lapsim.plotResult(r);
```

Other inputs include wheel inertia/radius, lateral CG offset, front track and wheelbase, tire grip, transmission ratio/efficiency, motor torque/current/power/RPM/losses, controller losses, battery voltage/sag/current/capacity, air density, headwind, grade, corner radius, accessories, and cruise or pulse/coast driving. Every input has units in [defaultParameters.m](+lapsim/defaultParameters.m).

**Replace the mass assumptions first.** Defaults retain the old 83.61 kg scale-derived car mass and add an explicitly assumed 70 kg driver (153.61 kg combined). The old code never established whether its scale readings included the driver. If your scale measurement already includes the driver, enter it as `chassis_mass_kg` and set `driver_mass_kg = 0`. Combined CG must be measured or recalculated separately when mass distribution changes.

## Build design targets

```matlab
% One-at-a-time sensitivities; each case starts from exactly the same p.
S = lapsim.sensitivity(p, track, ...
    {'vehicle.chassis_mass_kg','aero.cd','tires.crr','vehicle.cg_height_m'}, [.8 1 1.2]);

% Find which absolute rolling-resistance values achieve the configured target.
T = lapsim.targetSweep(p, track, 'tires.crr', [.001 .002 .003 .004 .0056]);
disp(T(:, {'parameter_value','mi_per_kWh','feasible','target_met','energy_margin_Wh'}))

% Compare cruise (zero band) and pulse/coast; speeds and full band widths are m/s.
D = lapsim.strategySweep(p, track, [16.6 17 17.5 18]*0.44704, [0 1 2]*0.44704);
disp(D(D.is_best_feasible,:))
```

The studies retain invalid and incomplete runs with reasons. `target_met` requires completion, the time and average-speed requirements, minimum mass, battery capacity, physical constraints, and the energy goal. The best strategy is only the best **feasible candidate in the tested grid**. Compare energy only for the same completed mission; a stalled car is not efficient.

`lapsim.exampleTargetParameters()` provides a reproducible combined design candidate: 85 kg total, Cd 0.10, area 0.60 m², Crr 0.003, and 3 W auxiliaries. `runLapSim` includes this candidate in the study exports. These are illustrative design inputs, not measured performance or guaranteed component capabilities; inspect [the example results](docs/example-results.md) before adopting them as targets.

## Track and model scope

`track` is a table of positive segment `length_m`, signed `grade` (rise/run), `radius_m` (`Inf` for straight), and `speed_limit_mps` (`Inf` for unrestricted). The lengths define the lap distance everywhere. The sample layout places the old slow turn near 3,256 m and adjusts the final straight to 3,832.5 m per lap. It is a **layout assumption**, not a surveyed course. Supply measured curvature and elevation for real targets; the archived 2023 GPS file is not silently used for this mission.

The model includes battery-terminal energy and a separate battery internal-loss/capacity budget. It checks longitudinal traction/load transfer, approximate three-wheel rollover limits, corner speeds, braking, voltage/current/power/RPM, and accessories. It has no regeneration and assumes a motor freewheel while coasting. A nonzero residual coast drag can be configured. Finish speed is retained; leftover kinetic energy is included, with no free braking or energy recovery at the finish.

See [model equations, assumptions, and calibration guidance](docs/model.md) and [verified example results](docs/example-results.md). This is an engineering target model; absolute results depend on measured road load, mass, motor/controller data, and track geometry.
