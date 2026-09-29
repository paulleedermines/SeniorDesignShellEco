# Legacy vehicle-model review

This review describes the source files in `Old Stuff/Vehicle Model/` as received. It is a static code audit, not a reproduction of competition measurements. References below use original file names and one-based line numbers. The legacy files are preserved for provenance; their output should not be treated as a validated efficiency target.

## What the old model was trying to do

`Shell_Track_Profile_Final.m` is the clearest final entry point. Its header credits Jordan Jeong and Dominic Padula, Mines 2026, and describes an Indianapolis event, an April 11 competition edit, and an April 21 cleanup (lines 1–16). It combines a longitudinal vehicle simulation, a hand-written track controller, a simplified electric motor model, and plotting in one script.

The vehicle starts from rest and runs four laps. Every 0.1 seconds it computes quadratic aerodynamic drag and constant rolling resistance, chooses drive/coast behavior from distance along the lap, updates speed and position with explicit Euler integration, and calculates motor electrical power. On selected straights it pulses between 16.5 and 18.5 mph; other sections accelerate below a 17.5 mph target and otherwise coast. It coasts through the main corners. The final version lowers these speeds because its comments say the event time allowance changed from 30 to 35 minutes (lines 210–221). That is historical source commentary, not verification of current competition rules.

The coastdown formula is the closed-form distance for `m_eq * dv/dt = -(b + c*v^2)`, where `b = Crr*m*g` and `c = 0.5*rho*Cd*A`. It was intended to time throttle release ahead of corners. The final model keeps that idea, but some thresholds are obsolete and the Turn 13 approach is a hard-coded distance instead of the calculated coast distance.

## Vehicle and motor provenance

These are source values, not verified measurements. The files do not include measurement reports, the referenced CDR, the motor datasheet, or the cited Maxon sinusoidal-control note.

| Quantity | Final source value | Source and evidence quality |
| --- | --- | --- |
| Vehicle mass | `sum([61 61 65]/2.205)-1.2` = about 83.607 kg | Final lines 40–41. Three wheel-scale loads appear to be pounds, converted to kg; the 1.2 kg subtraction is unexplained. Driver inclusion is unspecified. |
| Rear weight fraction | `(65/2.205)/m`, about 0.353 | Final line 45. Uses the original rear load after total mass is reduced; removal location is unspecified. |
| Cd / frontal area | 0.1 / 0.71 m² | Final lines 38 and 83. No test or CAD reference supplied. |
| Rolling coefficient | 0.0056 | Final line 39. No tire pressure, road surface, temperature, or coastdown data supplied. |
| Air density | 1.004 kg/m³ | Final line 33 explicitly labels this Colorado density despite the Indianapolis event description. |
| CG height | 0.375 m | Final lines 43–44 assume CG height equals aerodynamic force height. No measured CG location. |
| Wheelbase | 1.397 m | Final line 62. No drawing reference. |
| Wheel geometry | 0.203 m rim radius; 0.2413 m outer tire radius | Final lines 57–58. Loaded rolling radius is not verified. |
| Wheel / tire mass | 0.65 / 0.35 kg per wheel | Final lines 59–65. Disk wheel and annular tire inertia approximations. |
| Tire-road friction | 0.7 | Final line 66. No surface or tire test reference. |
| Shaft torque / gear ratio | 0.9 N·m / 9.23 | Final lines 47–52. Comments identify a 13-tooth small pulley and say a ratio of 10 slipped and approached the motor speed limit. |
| Drivetrain / controller efficiency | 0.93 / 0.95 | Final lines 74–75 identify a CDR estimate and an EE-team estimate. |
| Battery voltage / current limit | 60 V / 10 A | Final lines 36–37. The current limit is not enforced; motor versus battery current is not distinguished. |
| Accessory load | 0 W | Final line 72. Declared but never included in the energy calculation. |

The local motor function (Final lines 790–865) computes torque-producing current, effective PWM voltage, duty cycle, copper loss, and efficiency at a requested shaft torque and speed. It uses resistance 0.275 ohm, torque constant `0.136*0.9` N·m/A, speed constant 70.2 rpm/V, no-load speed 4,160 rpm, and no-load current 0.245 A. These are described as a 30 V motor operated at 60 V. Eighty percent of inferred no-load loss becomes a constant friction torque and twenty percent a speed-dependent torque; the code explicitly calls this split a guess (lines 809–810).

The map and runtime function duplicate parameters and differ slightly (`0.493/2` versus `0.490/2` no-load current; approximate versus exact pi). Halving no-load current when doubling voltage is an assumption (lines 693–703), not a measured loss curve. The torque constant has an extra 0.9 factor whereas the back-EMF constant comes directly from the speed constant; current, voltage, and commutation conventions need checking before those constants are used together for calibrated predictions.

## Track provenance and contradictions

Final lines 134–135 say lengths and radii were estimated from the center of the track using Google Maps/Earth satellite imagery. Lines 125–127 warn that the track changed at the event. No source map, survey, or current event layout is included.

The described segment boundaries are 0, 636, 656, 696, 765, 1040, 1075, 1135, 1249, 1305, 2019, 2054, 2155, 2315, 2393, 2423, 3101, 3136, 3256, 3301, 3516, and 3826 m. Some controller regions combine several of these segments. Turns 1 and 7 retain coast logic even though the header says they no longer mattered after the layout change.

| Conflict | Source |
| --- | --- |
| Header says 3,826 m per lap and 15,304 m total; executable lap wrapping uses 3,825 m, or 15,300 m total. | Final lines 28, 575–577, 653. |
| Retained pre-event comments say 3,926 m per lap and 15,704 m total. | Final lines 206–208. |
| Turn 1 is described as 20 m over 90 degrees but its written radius is 25.477 m. The arithmetic gives 12.732 m. | Final lines 145–146. |
| Turn 7 is described as 35 m, but the radius formula uses 40 m. | Final lines 174–175. |
| Turn 13 is described as 135 degrees but its radius calculation uses `3*pi/2` (270 degrees). A 45 m arc over 135 degrees gives about 19.099 m, not 9.554 m. | Final lines 197–199. |

The independent `sem_2023_us.csv` has 2,696 rows of latitude, longitude, and elevation. Summing consecutive great-circle distances using a 6,371,000 m Earth radius gives approximately 3,848.27 m, before closing the last point to the first. Elevation ranges from 218.9818 to 222.7558 m. This differs from each hand-written track length. The filename suggests a 2023 layout; no script reads this CSV or its accompanying XLSX. It must not silently become the authoritative 2026 layout. Elevation accuracy, route direction, start line, and relationship to later layouts remain unverified.

## Findings affecting efficiency targets

1. **Force limits are bypassed.** Final lines 284–290 choose the lower power/traction force and then overwrite it with `T_out/r_t`. The controller repeatedly assigns the same unconstrained torque force. `I_max`, available power, and rear traction therefore do not constrain acceleration. The motor function returns `physically_possible` from PWM duty (line 833), but the simulator never acts on it. Infeasible motor operating points can still contribute to a completed run.

2. **The declared motor torque is not applied consistently.** Wheel force is based on `T_motor*GR/r_t`, without drivetrain efficiency. Later, required shaft torque is reconstructed by dividing wheel force by drivetrain efficiency (line 601). A configured 0.9 N·m consequently requests about 0.968 N·m from the motor. A replacement needs one shaft-to-wheel convention shared by dynamics, limits, and losses.

3. **Rotating inertia uses the wrong kinematic radius.** Final line 91 divides wheel inertia by the rim radius squared, although linear speed and torque use the outer tire radius. Equivalent mass should use the same rolling radius as wheel angular speed. Earlier `vehiclemodel.m` line 32 additionally triples the entire vehicle mass.

4. **Corner targets are not enforced limits.** The final controller does not calculate curvature-dependent traction or rollover limits and has no braking model. A “hold” state often simply sets force to zero. Coast distances use fixed nominal approach speeds, not actual pulse speed (lines 306–310). Because the updated straight speed is below the old Turn 1/7 values, their calculated coast distances can be negative. Turn 13's calculated coast distance is not used; the model coasts from 3101 m (lines 530–533). There is no corner-overspeed failure report.

5. **CG tuning has very limited meaning.** CG height affects only an ultimately bypassed rear-traction formula. There is no lateral CG offset, front track width, triangular support polygon, rollover calculation, or coupled longitudinal/lateral tire demand. On this three-wheel layout, lowering the CG cannot be claimed to improve a cornering result the model never computes.

6. **Electrical energy excludes known loads.** Accessory power is unused, battery resistance/capacity and controller standby loads are absent, and coast mode sets motor RPM to zero even while the vehicle moves (lines 602–604). That implicitly requires a decoupled/freewheel drivetrain, but the hardware assumption is undocumented. Calling the motor function at zero torque and zero speed still adds a small artificial friction-related electrical loss. There is no regenerative braking model.

7. **Completion and plotting can mislead.** `while true` has no time or progress guard (line 266), while the time vector ends at 2100 s (line 94). Slow or stalled parameter cases can outgrow arrays or fail at reporting instead of returning a useful infeasibility result. Position resets every lap, unused preallocated samples remain in plots/integration, and line 659 forcibly writes 15,304 m into the last position sample. Distance and energy should both come from the actual simulated trace, ending at an interpolated finish event.

8. **Sensitivity cases are not comparable.** `Shell_Sensitivity_With_Hypermiling.m` overrides baseline force with 131.9675 N (line 98), while parameter runs use a different limiter. It computes `m_eq_p` at line 226 but uses baseline `m_eq` at lines 279 and 282. `isPulsing` is initialized only once (line 75), not for every case. Efficiency can exceed one because its clamp is commented out (lines 218–222). Its allocated result size is 11 by 10 but the loops fill 12 by 13 (lines 180–185). It simulates fixed-duration straight-line travel, not the final lap profile.

## Other scripts reviewed

| File | Purpose and status |
| --- | --- |
| `vehiclemodel.m` | Earliest 30-minute pulse/coast model, 48 V motor, 85 kg vehicle. Rolling resistance is incorrectly multiplied by speed (line 96). Startup uses positive torque/current with zero voltage (lines 83–105), so the first mechanical acceleration has no electrical cost. |
| `vehiclemodel (1).m` | Variant with 3,750 rpm no-load speed, ratio 9.12, different speed band, and an unused efficiency lookup experiment (lines 35–47, 80–87). Retains the earliest mass, rolling-resistance, and startup-energy issues. |
| `Shell_Eco_Model.m` | 1 s straight-line, accelerate-and-hold model. Integrates wheel mechanical power rather than battery energy (lines 93–98); tire inertia is computed but omitted from equivalent mass (line 46). Calls `average(P_x)` at line 129; no such helper exists in this repository. |
| `Improved_Shell_track_profile.m` | Earlier 3,926 m constant-speed-corner/coast-approach model. Contains incomplete assignments `v_max =;` and `v_min =;` at lines 94–95, preventing parsing. Uses ideal transmission efficiency. |
| `Shell_track_Hypermiling.m` | Earlier 19.5–21.5 mph pulse/coast model on the 3,926 m layout, torque 1.49 N·m, ratio 8, transmission efficiency 1. Its energy uses force times speed (line 487), without the final motor/controller model. |
| `Feb9Model.m` | Near-final integrated vehicle/motor script on the old 3,926 m layout, 19.5–21.5 mph pulse range. Also contains incomplete assignments at lines 112–113. Its low-gear change alters motor RPM/torque reporting but does not consistently change the wheel-force command. |
| `oldmotor_efficiency_plot.m` | Standalone motor map using a different 60 V baseline: 1.26 ohm, 0.286 N·m/A, 1,980 rpm (lines 5–11). Not the same parameter set as the final runtime function. |
| `EC90_30V_plot.m` | Motor map and hand-estimated acceleration/coast cycle energy for a 30 V motor on 60 V. Coast duration uses a mph speed difference with acceleration in m/s² (line 112). Its operating-point function uses `V_input*I` rather than PWM effective voltage (line 171), despite calculating back-EMF. Race distance is assumed to be ten miles (line 123). |
| `EC90_lastyear_estimation.m` | Similar 48 V estimate, but helper calls use 60 V (line 233) and no-load loss calibration also uses 60 V (line 145). Does not establish a consistent electrical model. |
| `motorgearstuff.m` | Exploratory gear/efficiency calculation. Converts wheel speed to revolutions/s (line 43) then uses that as angular speed in torque-power expressions (lines 102–103), missing a factor of `2*pi`. `Kt=1/Ke` at line 71 is also dimensionally inappropriate for a single SI motor convention. |

`Simulink/FOCModel.slx` is an additional binary artifact, not a MATLAB script. Its controls, motor parameters, and execution were not validated by this source review. The XLSX was not independently compared against the CSV.

## Requirements carried into the replacement

Use one parameter structure with SI units and separate vehicle, environment, battery, motor, track, and driving strategy data. Derive total mass, wheel inertia, axle loads, lap length, and energy metrics from those inputs rather than duplicated constants. Expose measured/estimated status alongside the baseline.

Enforce motor torque, motor current, battery current, voltage, speed, tire demand, and completion constraints in the dynamics. Include grade, wind, accessory draw, drivetrain/controller losses, and any stated freewheel or regen assumption. Make CG affect rear-wheel normal force and the appropriate three-wheel cornering stability model; report the limitations of any quasi-static approximation.

Use the same simulator for baseline, parameter sweeps, strategy comparisons, and target searches. Every run must start with independent state and return distance, elapsed time, battery energy, Wh/km, mi/kWh, losses, and feasibility. An efficient result that misses the event time or violates physical limits must not qualify as an achieved target.

Preserve the old 2026 layout only as an explicitly provisional demonstration. Before using predicted values as build targets, measure all-up mass and wheel loads with the driver, loaded tire radius, CG location, rolling coastdown behavior, CdA, motor/controller efficiency and current conventions, accessory draw, actual track geometry, and event constraints. Compare predicted speed and battery-energy traces against logged runs; calibration should reduce uncertainty, not force the replacement to match a legacy output containing the defects above.
