# AGENTS.md: working notes for this repo

Onboarding notes for AI coding agents and new team members. [`README.md`](README.md) is the verified audit of the legacy code; read §1, §3 and §7 there before touching anything.

## Goal

Build a MATLAB **lap simulator** for the Colorado School of Mines battery-electric **Prototype** at Shell Eco-marathon Americas (Indianapolis Motor Speedway road course). For a given car, driver and strategy it must predict:

- finish time for the attempt (4 laps, 15.33 km, **≤ 35 min**, standing start);
- energy measured by the joulemeter (kWh), and efficiency in **km/kWh** (the official unit) and mi/kWh (the unit used in team history);
- whether every operating point is physically possible (current, voltage/duty cycle, traction, motor speed).

It should then be used for design trades (gear ratio, motor torque, mass, tyres, aero) and race strategy (pulse-and-glide band, corner speeds).

## Repo layout and ground rules

```
README.md                    verified audit of legacy code + rules + track facts
AGENTS.md / CLAUDE.md        these notes
Old Stuff/Vehicle Model/     legacy MATLAB scripts + track GPS data (reference only)
lapsim/                      (proposed, not yet created) the new simulator
```

- **Treat `Old Stuff/` as read-only reference.** Don't fix bugs there. Port ideas into `lapsim/` instead. The one exception is if the team explicitly asks for a legacy fix, e.g. to reproduce an old result.
- **Never copy a constant from a legacy file without checking it** against README §7.2. The legacy files disagree with each other and several values are wrong (air density, lap length, I0, kt).
- The most complete legacy model is `Old Stuff/Vehicle Model/Shell_Track_Profile_Final.m`. With its own parameters it gives **33.0667 min / 0.0402 kWh / 236.4221 mi/kWh**, which makes it a useful regression benchmark while porting.
- `Old Stuff/Vehicle Model/sem_2023_us.csv` (lat, lon, altitude of one IMS lap) is the only real track data. Nothing uses it yet.

## Environment

- Windows 11. MATLAB **R2026a Update 5** at `C:\Program Files\MATLAB\R2026a\bin\matlab.exe`, with **only** the Optimization Toolbox installed. There is no Simulink or Simscape, so `Simulink/FOCModel.slx` (an unrelated induction-motor demo) cannot be opened.
- Python 3.13 is available for one-off data analysis.
- All legacy code uses base MATLAB only.

### Running MATLAB headless

```powershell
& "C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -batch "cd('D:/EcoMarathon/SeniorDesignShellEco/Old Stuff/Vehicle Model'); set(groot,'DefaultFigureVisible','off'); Shell_Track_Profile_Final"
```

- **Speed:** start-up takes about 20 s, and `Shell_Track_Profile_Final` itself about 35 s. Batch several checks into one `-batch` call rather than launching MATLAB repeatedly.
- **Figures:** `set(groot,'DefaultFigureVisible','off')` keeps them from popping up.
- **Paths:** they contain spaces, so always quote them and `cd` into the folder first.
- **`clear` in legacy scripts:** every legacy script starts with `clear` (some with `clear all`, `close all`, `clf`), which wipes the caller's workspace. To time or post-process a run, stash values with `setappdata(0,'name',value)` before calling it, or read workspace variables *after* it returns.
- **Variable reuse:** after `Shell_Track_Profile_Final` finishes, `i` and `V` have been overwritten by the motor-map loop. Use `Time_comp` and `find(v>0,1,'last')` instead.
- **Invalid file name:** `vehiclemodel (1).m` is not a valid MATLAB name, so `run()` fails. Only `eval(fileread('vehiclemodel (1).m'))` works.
- **Known crash:** `Shell_Eco_Model.m` crashes at line 129 (`average` does not exist).
- **Code Analyzer:** `checkcode('file.m','-id')` is useful.

## Physics the new model must get right

The new model uses SI units internally; convert only at input and output.

- **Equivalent mass:** `m_eq = m_total + Σ I_wheel/r_t² + J_rotor·GR²/r_t²`.
  - `m_total` = vehicle + max(driver, 50 kg) + ballast.
  - Use the **rolling radius** `r_t`, not the rim radius.
  - Don't forget the rotor inertia.
- **Road load:**
  - `F = C_rr·m·g·cosθ + ½·ρ·CdA·v² + m·g·sinθ`.
  - Rolling resistance does **not** scale with speed.
  - ρ at IMS is about 1.15–1.19 kg/m³ (≈ 220 m altitude); it is **not** 1.004.
- **Motor (Maxon EC90 flat, 30 V winding driven from a 60 V pack):**
  - `kt = ke` in SI units (N·m/A = V·s/rad). The legacy 0.9 derate on kt alone creates a non-physical 10 % loss. Put any commutation penalty in a separate, named efficiency.
  - Electromagnetic torque `T_e = T_load + T_f + B·ω`. Calibrate friction `T_f` and viscous `B` from the datasheet no-load point **at 30 V**. Don't halve I0 when running at 60 V.
  - `I = T_e/kt`. Terminal voltage `V_eff = I·R + ke·ω`. Duty cycle `D = V_eff/V_batt`, which **must be ≤ 1**. Current must be ≤ I_max.
  - Electrical power is `V_eff·I`, not `V_batt·I`.
- **Drivetrain:** wheel force = `T_motor·GR·η_dt / r_t`. Command motor torque, and derive force from it (not the other way round), so gear changes actually change force.
- **Coasting:** state the assumption explicitly.
  - With a freewheel, the motor stops and adds no drag.
  - Without one, add back-drive drag `(T_f + B·ω)·GR/(r_t·η)` and motor losses.
  - The legacy code silently assumes a freewheel, and this assumption alone moves the result by about 13 %.
- **Battery (joulemeter) power:** `P_batt = P_elec/η_ctrl + P_aux`, charged **every** step, coasting included. P_aux covers the controller's standby draw, telemetry, dash and so on. Everything is on the one metered pack (the rules forbid a separate accessory battery). Battery internal I²R losses are upstream of the joulemeter and are not scored. There is no regeneration unless the team adds it; the joulemeter gives no credit for net-negative energy.
- **Limits:** apply every limit (torque, current, duty cycle/voltage, traction, motor speed) as a hard cap **after** the strategy picks a command, and flag any step where a cap was hit.
- **Integration:**
  - dt = 0.1 s explicit Euler has converged to about 0.1 % on the legacy model; semi-implicit Euler (update v, then x with the new v) is preferred.
  - Always check that the energy balance closes.
  - End the race by **distance** (laps × lap length). Bound the loop by the time limit and report a DNF instead of crashing or hanging.

### Competition constants

These come from the SEM 2024–26 rules. Check the current year's rulebook every season.

| Name | Value |
|---|---|
| Laps / total distance | 4 / 15,330 m (lap 3,832.5 m) |
| Time limit | 2,100 s (35 min), so minimum average speed is 7.30 m/s |
| Start / stops | Standing start; no stops for Prototypes |
| Max voltage | 60 V |
| Battery | 1 lithium pack, ≤ 1000 Wh, which powers everything |
| Driver mass | ≥ 50.0 kg incl. gear (ballast if lighter) |
| Vehicle mass | ≤ 140 kg without driver |
| Turning radius | ≤ 8 m |
| Scoring | km/kWh = 15.33 / E_joulemeter[kWh] |

Recent winning scores give a sense of scale. These come from the live results site and are unverified: 2025 winner about 341 km/kWh, 2026 winner about 470 km/kWh. The legacy model's 236.4 mi/kWh is about 380 km/kWh.

## Track data pipeline (`sem_2023_us.csv`)

- **File format:** columns `Latitude, Longitude, Metres above sea level`; 2,696 points about 1.43 m apart; one clockwise lap of 3,849 m.
- **Where the lap starts:** the file begins at the exit of T1, not at the start line.
- **Other defects:**
  - a 0.97 m seam between last and first point;
  - a 0.9 m GPS glitch at index 48–49;
  - float-noise decimals;
  - raw grades up to ±13 % that are pure noise.

Recommended processing, to be written as `lapsim/track/loadTrack.m`:

1. Project to local east/north metres around the mean latitude.
2. Close the loop by spreading the seam error along the lap. Use wrap-around filters everywhere. Sanity checks: total heading change is −360° and length ≈ 3,849 m (arc length taken from raw points, not from smoothed ones).
3. Curvature: resample at 1 m and apply Gaussian smoothing with σ ≈ 5 m (keep σ = 3–8 m as a sensitivity range).
4. Grade: resample at 5 m and apply a 25 m moving average. This gives about ±2 % grade and about 6.3 m of climb per lap.
5. Move the origin to the start/finish line. By alignment with the legacy turn table it is about CSV s = 3,134 m (39.79315, −86.23871); confirm on site. Index by `mod(s, L_lap)`.
6. Corner speed limit: `v_lim(s) = sqrt(a_y_max/|κ(s)|)` with `a_y_max = min(μ·g, rollover limit, driver comfort)`. The tightest corners are T12 (R ≈ 16.5 m), T13 (R ≈ 18 m), T1 and T10 (R ≈ 27 m). Build the speed envelope with backward passes using the coast-down deceleration.
7. Decide how to reconcile the GPS lap (3,849 m) with the official lap (3,832.5 m): scale the track, or keep it and note the 0.4 % difference. The 2026 layout was also modified on site (see the header of `Shell_Track_Profile_Final.m`). Rerun the pipeline when Shell publishes the current year's track.

## Legacy traps

One line each; details and line links are in README §7.

- `F = T_out / r_t;` overwrites the traction/power limiter. Duty cycle, `I_max` and `physically_possible` are never enforced.
- `kt = 0.136*0.9` with an un-derated `ke`, plus `I0 = 0.490/2`: the motor model is internally inconsistent.
- Coasting sets motor rpm to 0, which is an undocumented freewheel assumption.
- `rho = 1.004 % Colorado`; no grade; `AccessoryPower = 0` is never used.
- Lap length is 3825 in code, 3826/3926 in comments, and the reported 15304/15704 are hard-coded, not simulated.
- `s_1` and `s_7` are negative (the corner targets sit above `v_st`), so the coast-in windows are dead. `s_13` is unused.
- The corner-radius arithmetic in the comments is wrong (T13 R is 19 m, not 9.55 m) and the turn positions drift up to about 120 m from the GPS track.
- `while true` has no time-limit check, so the script crashes or hangs on a DNF.
- There is a single fixed throttle level, so "hold speed" is 10 Hz on/off switching.
- Sensitivity script: `m_eq_p` is unused, the baseline is a hard-coded 131.97 N, `isPulsing` leaks between cases, dt = 1 s, and energy is wheel-only. **Do not trust its output.**
- Motor tools: rev/s used as rad/s (`motorgearstuff.m`), mph divided by m/s² (`EC90_30V_plot.m`), a 48 V file evaluated at 60 V (`EC90_lastyear_estimation.m`), and electrical power computed as `V_bus·I` (no PWM).

## Proposed lapsim architecture

All MATLAB functions, no scripts in the core:

```
lapsim/
  params/defaultParams.m      % one struct: every value has units + source/"ASSUMED" note
  track/loadTrack.m           % CSV -> struct(s, x, y, z, grade, kappa, L_lap, v_lim)
  model/roadLoad.m            % F_rr + F_aero + F_grade
  model/motorModel.m          % (T_cmd, omega, V_batt, p) -> T_e, I, V_eff, D, P_elec, feasible
  model/drivetrain.m          % motor <-> wheel, gear ratio, eta_dt, freewheel flag
  strategy/pulseGlide.m       % strategy = function handle: (state, track, p) -> T_cmd
  simulateRace.m              % (params, track, strategy) -> result (timeseries + summary)
  runBaseline.m               % example entry point, prints km/kWh + mi/kWh + time + DNF flag
  tests/                      % matlab.unittest
```

Minimum tests:

- Energy balance closes: battery energy = road-load work + ΔKE + ΔPE + every loss, within 0.5 %.
- Distance: the simulated distance equals laps × L_lap (no hard-coded constants).
- Time: finishing ≤ 2,100 s passes; a deliberately slow car reports a DNF and neither crashes nor hangs.
- Limits: over a run, max current ≤ I_max, max duty cycle ≤ 1, and wheel force ≤ the traction limit.
- Convergence: dt = 0.1 s vs 0.01 s agree within 0.5 %.
- Regression: with legacy parameters and a flat 3825 m lap, the result reproduces about 236.4 mi/kWh / 33.07 min. Keep the legacy switches (kt derate, I0/2, freewheel) for this test only.

## Conventions for new code

- Functions, not scripts. No `clear`, `clc` or `close all` in library code. No changes to global graphics defaults (`set(0,...)`).
- One parameter source (`defaultParams.m`). Never hard-code lap length, lap count, time limit, air density or motor constants anywhere else.
- Add a units comment on every physical quantity: `% [m/s]`, `% [N*m]`. Use `pi`, not `3.14`. Name conversions (`MPH_TO_MS = 0.44704`) instead of writing `/2.237`.
- Every parameter gets a source (datasheet, measurement, rule article) or an explicit `ASSUMED` note.
- Keep plotting separate from simulation. Trim arrays to the simulated length before plotting.
- Report results as time [min], energy [Wh and kWh], km/kWh and mi/kWh, and a feasibility summary (max current, max duty cycle, limit hits, DNF).

## Git workflow

- Work happens on branch **`claude`** (tracks `origin/claude` on github.com/paulleedermines/SeniorDesignShellEco). Merge into `main` via pull request only.
- Before quoting a number in a commit, a README or a PR: run the tests and the headless model, and paste the actual output.
- When a legacy issue is fixed in the new model, or a parameter gets a real source, update README §7 and this file.
