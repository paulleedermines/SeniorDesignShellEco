# Reproducible efficiency targets

These results use the supplied four-lap, 15.33 km mission and the approximate example track. They are simulations with assumed inputs, not measured car performance. The 300 mi/kWh goal equals **482.8032 km/kWh**, **2.07124 Wh/km**, or **31.7521 Wh per attempt** at the battery terminals.

Run `results = runLapSim` from the repository root to regenerate the baseline, target candidate, sensitivity tables and strategy comparisons. Output goes to `results/latest/`; the MAT file preserves the inputs as well as results. To run just the candidate:

```matlab
p = lapsim.exampleTargetParameters();
r = lapsim.simulate(p, lapsim.exampleTrack());
disp(r.summary)
lapsim.plotResult(r)
```

## Baseline and combined candidate

| Input / outcome | Estimated baseline | Illustrative design candidate |
| --- | ---: | ---: |
| Complete car mass, excluding driver | 83.61 kg | 35 kg |
| Driver mass | 70 kg assumed | 50 kg assumed |
| Combined mass | 153.61 kg | 85 kg |
| Cd | 0.10 | 0.10 |
| Frontal area | 0.71 m² | 0.60 m² |
| CdA | 0.071 m² | 0.060 m² |
| Crr | 0.0056 | 0.0030 |
| Auxiliary draw | 5 W | 3 W |
| CG height | 0.375 m | 0.300 m |
| CG distance behind front axle | 0.486 m | 0.500 m |
| Delivered terminal energy | 62.30 Wh | 27.26 Wh |
| Energy including internal battery loss | 63.08 Wh | 27.59 Wh |
| Efficiency | 152.9 mi/kWh | 349.4 mi/kWh |
| Completion time | 33.22 min | 32.96 min |
| Average speed | 17.20 mph | 17.34 mph |
| Meets modeled race constraints | Yes | Yes |
| Meets 300 mi/kWh goal | No | Yes |
| Energy margin against goal | -30.54 Wh | +4.49 Wh |

Both use the same 16.5–18.5 mph pulse/coast band, powertrain and flat approximate track. The candidate meets the goal with about **14% of the allowed energy unused** under these assumptions. Its mass, Crr, CdA and auxiliary power are a combined target set to investigate. No individual change is claimed sufficient, and neither the driver mass nor the available components has been verified.

The baseline is intentionally explicit about its uncertain mass: the old scale sum may already have included a driver. A real all-up measurement can change the comparison substantially. The 50 kg minimum is a validity constraint; it is not used as the baseline all-up mass.

## Interpreting the loss budget

The baseline spends approximately 35.93 Wh on rolling resistance, 10.98 Wh on aerodynamic drag, 4.44 Wh on motor losses, 3.65 Wh on transmission losses, 2.98 Wh on controller losses, and 2.77 Wh on auxiliaries. Braking and kinetic energy remaining at the finish account for the rest of the delivered energy. Internal battery heating adds a separate 0.79 Wh.

With these assumed inputs, rolling resistance is the largest energy term. This makes measured all-up mass and coastdown-derived road load the first useful inputs to refine. Strategy sweeps compare the same vehicle on the same mission; they should not be expected to compensate for a large road-load error.

For the baseline, the best feasible strategy in the supplied grid is a 17 mph band center with a 1 mph full pulse/coast width: approximately 155.2 mi/kWh. That is only a modest change from 152.9 mi/kWh. A separate rolling-resistance sweep achieves about 367.0 mi/kWh at Crr 0.001 and 279.1 mi/kWh at Crr 0.002; both finish the mission. This brackets a numerical target for that assumed vehicle, but does not establish that either tire value is achievable on the actual surface.

Independent 20% reductions from the baseline produce these approximate changes in delivered energy:

| Changed input | Terminal-energy reduction |
| --- | ---: |
| Crr | 14.6% |
| Car mass excluding driver | 8.3% |
| Driver mass | 6.9% |
| Cd or frontal area (separate cases) | 4.4% |
| Auxiliary power | 0.9% |

These effects are not additive; rerun combined candidates. Driver mass here is a sensitivity input, not a recommendation to change the driver.

The CSV studies expose `valid`, `completed`, `stop_reason`, `within_time_limit`, `within_speed_requirement`, `within_mass_requirement`, `physically_feasible`, `feasible` and `target_met`. A valid simulation can still be an infeasible attempt. A cruise setting of exactly 16.6 mph falls below the required average after accounting for startup and slower corners.

## Validation

The regression suite checks a constant-speed case against independent closed-form electrical and road-load calculations; ideal acceleration against kinetic energy; startup copper losses; battery and motor limits; auxiliary infeasibility; partial battery/time-limited runs; segment/lap boundaries; braking across lap boundaries; signed grade work; mass/drag/rolling trends; grip and CG effects; validation errors; repeatable parameter studies; motor RPM transitions; and dynamic roll constraints.

Full-race timestep refinement from 0.25 s to 0.125 s changes the baseline energy by about 0.03% and time by about 0.07%. Its energy-ledger residual is below 1e-7 Wh. These checks validate numerical and physical consistency of the implemented approximations. They do not establish real-world predictive accuracy; calibration still requires measurements.
