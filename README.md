# Shell Eco-marathon Lap Simulator (Mines 2026)

The goal of this repo is a MATLAB **lap simulator** for our battery-electric **Prototype** car at the Shell Eco-marathon (SEM) Americas at Indianapolis Motor Speedway (IMS). It should predict finish time, energy and efficiency (km/kWh and mi/kWh) well enough to drive design and race-strategy decisions.

**Status:** there is no trustworthy lapsim yet. Everything in [`Old Stuff/Vehicle Model/`](Old%20Stuff/Vehicle%20Model/) is a legacy model from the 2025–26 team, kept for reference. This README records what those models do, what is wrong with them (every issue below has been re-checked), and what a new model has to get right. Notes for AI coding agents and new contributors are in [`AGENTS.md`](AGENTS.md).

---

## 1. Status at a glance

| | |
|---|---|
| Most complete legacy model | [`Shell_Track_Profile_Final.m`](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m) ("FINAL 4.21.26") |
| Its output (reproduced exactly, MATLAB R2026a) | **33.07 min, 0.0402 kWh, 236.4 mi/kWh (≈ 380 km/kWh)** |
| With realistic corrections (Indy air density, motor no-load losses calibrated at 30 V, equivalent-mass fix, dt = 0.01 s) | **≈ 214.9 mi/kWh (−9.1 %)**, 33.15 min |
| Open hardware questions that move the answer most | Is there a **freewheel** between motor and wheel? (If not: −12.9 %.) Is the 0.9 **kt derate** real? (If it was a mistake: +11.7 %.) |
| Other models | All superseded, broken, or use wrong physics (see §3 and §7) |

## 2. Quick start: run the legacy model

This needs MATLAB (tested on R2026a Update 5). The scripts don't read or write any files. PowerShell:

```powershell
& "C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -batch "cd('D:/EcoMarathon/SeniorDesignShellEco/Old Stuff/Vehicle Model'); set(groot,'DefaultFigureVisible','off'); Shell_Track_Profile_Final"
```

About 35 s of simulation plus about 20 s of MATLAB start-up. Expected output:

```
Exiting loop because four laps have been completed!
Time_comp =
   33.0667
Distance [m]: 15304.0  Distance [mi]: 9.5095  Energy [kWh]: 0.0402  Eff [mi/kWh]: 236.4221
Global max efficiency = 83.65 %
  Occurs at: Torque = 0.903 N*m, Speed = 4062.2 rpm, Supply V = 60.0 V
```

The printed distance is hard-coded, not simulated ([§7.1](#71-shell_track_profile_finalm-the-latest-legacy-model), #13).

## 3. Repository map

All code is in `Old Stuff/Vehicle Model/`. Every file is a top-level script: no functions, no tests, no parameter file. Base MATLAB only.

| File | What it is | Status | Runs? (R2026a) | Headline output |
|---|---|---|---|---|
| [`Shell_Track_Profile_Final.m`](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m) | 4-lap segment-based track model with pulse-and-glide, EC90 motor loss model and drivetrain/controller efficiencies | **Latest legacy model** | Yes, about 35 s | 33.07 min, 236.4 mi/kWh |
| [`Feb9Model.m`](Old%20Stuff/Vehicle%20Model/Feb9Model.m) | Direct parent of Final: 3926 m lap, 30 min, 20.5 mph, two-speed gear | Superseded | Yes | 29.07 min, 211.6 mi/kWh |
| [`Improved_Shell_track_profile.m`](Old%20Stuff/Vehicle%20Model/Improved_Shell_track_profile.m) | First segment-based track model (2.4.26); counts energy at the wheel only | Superseded | Yes | 28.95 min, 285.5 mi/kWh (no losses) |
| [`Shell_track_Hypermiling.m`](Old%20Stuff/Vehicle%20Model/Shell_track_Hypermiling.m) | Sibling fork of Improved with `isPulsing` pulse-and-glide | Dead-end branch | Yes | 29.19 min, 288.8 mi/kWh (no losses) |
| [`Shell_Sensitivity_With_Hypermiling.m`](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m) | Fixed 30-minute run plus a 12-parameter × 13-level sensitivity sweep | **Unreliable, don't use** (§7.3) | Yes | 285.1 mi/kWh over 17.9 km |
| [`Shell_Eco_Model.m`](Old%20Stuff/Vehicle%20Model/Shell_Eco_Model.m) | Earliest model of this lineage: accelerate, then hold speed for 30 min | Broken | **No**, crashes at L129 | n/a |
| [`vehiclemodel.m`](Old%20Stuff/Vehicle%20Model/vehiclemodel.m) | Older, separate model (48 V linear stall-torque motor) | Wrong physics | Yes | Never finishes (14.3 km), 44.9 mi/kWh |
| [`vehiclemodel (1).m`](Old%20Stuff/Vehicle%20Model/vehiclemodel%20%281%29.m) | Edited duplicate of the above | Wrong physics, invalid file name | Only via `eval(fileread(...))` | 79.2 mi/kWh |
| [`EC90_30V_plot.m`](Old%20Stuff/Vehicle%20Model/EC90_30V_plot.m) | Maxon EC90 flat (30 V winding) driven at 60 V: efficiency/power maps and a back-of-envelope race energy | Motor tool; source of Final's motor model | Yes | 232.4 mi/kWh (unit bug, §7.5) |
| [`EC90_lastyear_estimation.m`](Old%20Stuff/Vehicle%20Model/EC90_lastyear_estimation.m) | 48 V clone of the above | Motor tool | Yes | 109.7 mi/kWh |
| [`oldmotor_efficiency_plot.m`](Old%20Stuff/Vehicle%20Model/oldmotor_efficiency_plot.m) | Efficiency map of the previous 60 V motor | Superseded motor tool | Yes | max η 86.7 % |
| [`motorgearstuff.m`](Old%20Stuff/Vehicle%20Model/motorgearstuff.m) | Hand calculation for the old 48 V motor and gear ratio | Scratch, wrong units | Yes | "990 mi/kWh" (meaningless) |
| [`sem_2023_us.csv`](Old%20Stuff/Vehicle%20Model/sem_2023_us.csv) | GPS trace of one lap of the IMS road course (lat, lon, altitude) | **Useful data, used by nothing** | n/a | see §5 |
| [`sem_2023_us.xlsx`](Old%20Stuff/Vehicle%20Model/sem_2023_us.xlsx) | Same points, plus one broken haversine formula (E2/F2) | Redundant | n/a | n/a |
| [`Simulink/FOCModel.slx`](Old%20Stuff/Vehicle%20Model/Simulink/FOCModel.slx) | Induction-motor field-oriented-control demo (Simscape Electrical, R2025b); not our motor, not wired sensibly | Unrelated | Can't open here (needs Simulink + Simscape Electrical) | n/a |

**Lineage** (from content diffs and header dates; git holds no history before the bulk add):

- `Shell_Eco_Model` → `Shell_Sensitivity_With_Hypermiling`
- `Shell_Eco_Model` → `Improved_Shell_track_profile` → `Shell_track_Hypermiling` (dead end)
- `Improved_Shell_track_profile` → `Feb9Model` → `Shell_Track_Profile_Final`
- The motor code in Feb9Model/Final comes from `oldmotor_efficiency_plot` → `EC90_30V_plot`.
- `vehiclemodel*.m` is a separate, earlier lineage that nothing reuses.

## 4. Competition constraints the simulator must respect

These come from the SEM 2024–2026 Americas rules (Chapter I = global rules, Chapter II = regional rules). Check them against the current year's rulebook before each season.

| Constraint | Value | Rule |
|---|---|---|
| Attempt | 4 consecutive laps, **15.33 km** total, within **35 min** | Ch. II Art. 226 (2026) |
| Implied lap length / minimum average speed | **3,832.5 m** / **7.30 m/s** (26.3 km/h, 16.3 mph) | derived |
| Start | Standing start, no push | Ch. I Art. 14, Ch. II Art. 230c |
| Stops | None for Prototypes (only Urban Concept cars have mandatory stops); stopping on track is forbidden | Ch. II Art. 228, Ch. I Art. 18a |
| Score | km/kWh from the organiser's **joulemeter**, which sits between the battery and *all* electrical loads (propulsion, controller, telemetry, …). Battery internal I²R losses happen upstream and are not counted. | Ch. I Art. 54d, 56c–d |
| Voltage | ≤ 60 V anywhere on the car | Ch. I Art. 57a |
| Battery | Exactly one (no separate accessory battery), lithium, ≤ 1000 Wh | Ch. I Art. 57b, 57d, 66b |
| Driver | ≥ 50.0 kg with gear; add ballast if lighter | Ch. I Art. 20 |
| Vehicle | ≤ 140 kg without driver; 3 or 4 wheels; turning radius ≤ 8 m; no movable aero | Ch. I Art. 39g, 25, 42 |
| Attempts | Up to 6; best counts | Ch. II Art. 229 |

## 5. Track data (`sem_2023_us.csv`)

- **What it is:** 2,696 points about 1.43 m apart, forming **one closed, clockwise lap of 3,849 m**. The seam between last and first point is 0.97 m. Measured by haversine.
- **Layout:** the IMS road course; the turn sequence matches the team's T1–T14.
- **Where the file starts:** at the exit of T1, not at the start line. Aligning to the scripts' turn table puts the scripts' origin at about CSV 3,134 m (not confirmed on site).
- **Elevation:** 218.98–222.76 m, a **3.77 m range**, with about 6.3 m of climb per lap after smoothing. Grades after smoothing fall within about ±2 %; raw point-to-point grades (up to ±13 %) are GPS noise. A 1 % grade is 8.2 N, more than rolling resistance (4.6 N).
- **Accuracy:** it matches the team's earlier Google Maps measurement (3,840 m). It is 0.4 % longer than the official 2024–26 lap (3,832.5 m).

**Corners measured from the GPS** (radius R at σ = 5 m smoothing; v_max = √(0.7·g·R)). Positions are in the scripts' lap coordinates, ±15 m:

| Turn | Dir | Position (m) | Arc (m) | Heading change | R (m) | v_max (m/s) | What `Shell_Track_Profile_Final.m` assumes |
|---|---|---|---|---|---|---|---|
| T1 | R | 626–698 | 72 | −83° | 27 | 13.6 | 636–656, "R 25.5" (actually 12.7 by its own numbers), uses 8 m/s |
| T2 | L | 702–771 | 69 | +82° | 35 | 15.4 | 696–765 |
| T3 | R | 821–953 | 132 | −65° | 53 | 19.1 | 765–1040 |
| T4 | R | 1003–1125 | 122 | −110° | 34 | 15.3 | 1040–1075, "60°" |
| T5/T6 | L/R | 1215–1302 | 83 | ±48° | 31–36 | 14.6 | 1249–1305, "R 17.8" |
| T7 | L | 1992–2112 | 120 | +92° | 55 | 19.5 | 2019–2054, uses 8.3 m/s |
| T8/T9 | R/L | 2150–2302 | 152 | ∓72° | 40–45 | 16.6 | 2155–2315 |
| T10 | R | 2360–2495 | 135 | −103° | 27 | 13.5 | 2393–2423, "60°" |
| T11 | R | 2495–3013 | 518 | −82° | 57 | 19.9 | 2423–3101 (678 m) |
| **T12** | R | 3013–3059 | 46 | −92° | **16.5** | **10.6 (tightest)** | 3101–3136; code coasts through at straight speed |
| **T13** | L | 3137–3222 | 85 | +114° | **18** | 11.2 | 3256–3301, "R 9.55" (arithmetic error), uses 7 m/s |
| T14 | R | 3299–3552 | 253 | −116° | 52 | 18.9 | 3301–3516 |

- **Corners don't limit speed at μ = 0.7.** All of them allow at least 10.6 m/s, above the 7.4–8.3 m/s the strategy uses.
- **They would with a rollover or comfort limit.** At a lateral limit of about 0.3 g (typical for a trike), T12 (7.0 m/s), T13 (7.3 m/s), T1 and T10 (8.9 m/s) start to bind.
- **The hard-coded turn table in the scripts drifts.** T12 is about 90 m late and T13 about 120 m late.

## 6. Where the energy goes (legacy Final model)

Measured with an energy-balance run of `Shell_Track_Profile_Final.m`. The battery supplies 144.8 kJ. The balance closes to −0.16 %, which is exactly the energy explicit Euler creates at dt = 0.1 s.

| Sink | kJ | % of battery energy |
|---|---:|---:|
| Rolling resistance | 70.3 | 48.5 |
| Aero drag | 32.9 | 22.8 |
| **kt/ke mismatch (non-physical, §7.1 #1)** | **13.0** | **9.0** |
| Drivetrain (η = 0.93) | 7.9 | 5.5 |
| Motor copper loss | 7.5 | 5.2 |
| Motor controller (η = 0.95) | 7.2 | 5.0 |
| Motor friction and windage | 3.7 | 2.5 |
| Kinetic energy left at the finish | 2.4 | 1.7 |

Time-step convergence is fine:

| dt (s) | mi/kWh | Time (min) |
|---|---|---|
| 0.2 | 236.35 | 33.00 |
| 0.1 | 236.42 | 33.07 |
| 0.01 | 236.15 | 33.09 |
| 0.001 | 236.13 | 33.10 |

## 7. Verified issues

Severity reflects the impact on a result used for decisions. The "effect" column gives the measured change to the headline 236.4 mi/kWh when the issue alone is corrected (in-memory MATLAB runs; no files changed). Line numbers link to the file on GitHub.

### 7.1 `Shell_Track_Profile_Final.m` (the latest legacy model)

`F` below means [`Shell_Track_Profile_Final.m`](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m).

| # | Sev. | Where | Issue | Effect |
|---|---|---|---|---|
| 1 | High | [F L797](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L797), [L799](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L799) (map copy [L699](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L699)) | **kt is derated by 0.9 but ke is not.** In SI units kt must equal ke for the same machine; otherwise 10 % of air-gap power vanishes with no physical mechanism, and motor efficiency can never exceed 90 %. Any commutation penalty should be an explicit, separate efficiency. | 9 % of all energy; kt = ke gives **264.1 mi/kWh (+11.7 %)** |
| 2 | High | [F L602–604](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L602), [L620](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L620) | **Coasting assumes the motor is fully decoupled** (rpm = 0, no drag) for the 79.5 % of the race spent coasting. Nothing in the repo mentions a freewheel or clutch. If the belt drive back-drives the motor, its friction adds about 1.3 N (28 % of rolling resistance). The whole pulse-and-glide benefit rests on this assumption. | Motor back-driven while coasting: **206.0 mi/kWh (−12.9 %)** |
| 3 | Medium | [F L33](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L33) | `rho = 1.004 % Air density in Colorado` for an Indianapolis race (≈ 220 m altitude: 1.15–1.19 kg/m³). Also lengthens every coast distance. | ρ = 1.18 kg/m³ gives **224.3 (−5.1 %)** |
| 4 | Medium | [F L800–801](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L800), comment [L693](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L693) | **No-load current halved "because voltage doubled".** The winding is unchanged (kt kept), so friction torque does not halve; the model understates motor loss torque about 2.3×. Calibrate at the datasheet's 30 V / 2080 rpm point. | **226.7 (−4.1 %)** |
| 5 | Medium | [F L72](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L72), [L636](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L636) | `AccessoryPower=0` is never used, but the joulemeter counts controller standby, telemetry, etc. Battery power is only `P_elec/0.95`, so pack voltage has no effect on anything. | 2 W: **230.1 (−2.7 %)**; 5 W: 221.3 (−6.4 %) |
| 6 | Medium | [F L54](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L54), [L620](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L620) | **No grade force** ("no grade, no wind"), although §5's elevation data sits in the same folder. | −1.5 % to +0.8 % and 32.8–33.6 min, depending on track alignment |
| 7 | Medium | [F L266](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L266), [L94](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L94), [L583](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L583) | **Unbounded `while true` loop** with no time-limit check. If a parameter set needs more than 2100 s, `t(i)` indexes past the end and errors. If the car stops in a coast zone, MATLAB hangs forever. A DNF is never reported. | Parameter sweeps crash or hang |
| 8 | Medium | [F L351](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L351) (and every "hold" block) | **Only one throttle level:** the drive force is always the fixed `T_out/r_t` = 34.4 N, or 0. "Hold speed" is 10 Hz on/off switching that no driver can do, so partial-throttle cruise and pulse-torque choice (the main strategy variables) can't be studied. | About 3 % of the result is this artefact |
| 9 | Medium | [F L290](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L290) (limiter [L285–289](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L285)); [L830–833](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L830); [L37](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L37) | **Limits computed, then bypassed.** The traction/power limiter is overwritten by `F = T_out / r_t;`. The duty cycle `D` and `physically_possible` are never checked, and `I_max` is never enforced. In this run nothing would bind (max 8.17 A, duty 0.755, traction limit 249 N), but any study raising torque or gear ratio, or lowering voltage, will silently produce impossible operating points. (Feb9Model actually exceeds its own `I_max` 8×.) | None today; a latent trap |
| 10 | Medium | [F L306–310](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L306), [L334](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L334), [L441](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L441), [L530](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L530) | **Corner coast-down logic is partly dead.** `v_1` = 8 and `v_7` = 8.3 are *above* `v_st` = 7.82 m/s, so `s_1` = −17.6 m and `s_7` = −47.8 m and the "coast into turn 1/7" windows are empty. `s_13` = 79.9 m is computed but unused; a fixed 3101–3256 m coast is used instead, so the car enters T13 at 5.7–6.2 m/s instead of the 7 m/s target. | Wasted time; strategy not what the comments say |
| 11 | Low | [F L146](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L146), [L174–176](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L174), [L198](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L198) | **Corner-radius arithmetic errors** in the track notes that set `v_1`, `v_7`, `v_13`. T13 uses 3π/2 (270°) for a "135°" turn, so R is 19.1 m, not 9.55 m. T1: 20/(π/2) = 12.7, not 25.5. T7 uses 40 m for a 35 m arc. See §5 for measured values. | Corner targets unfounded |
| 12 | Low | [F L28](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L28), [L204–208](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L204), [L557](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L557), [L575–577](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L575) | **Lap length defined several ways.** Code uses 3825 m. Comments say 3826 m, then "3926 m / 15,704 m / must average 19.5 mph" (stale, from Feb9). Official is 3832.5 m; GPS gives 3849 m. | 0.2–0.6 % of time and energy |
| 13 | Low | [F L653](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L653), [L659](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L659), [L577](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L577), [L640](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L640) | **Reported distance is hard-coded** (`15304`, while 15,300.6 m were driven), and `x(end) = 15304` is forced. The displacement plot is a per-lap sawtooth followed by 116 s of zeros and a spike; the velocity plot drops to 0 at the finish. The efficiency histogram's 0-bin includes the 1,160 unsimulated samples after the finish. | Output can't reveal distance bugs; plots misleading |
| 14 | Low | [F L575](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L575) | Lap wrap happens *after* the strategy decision, so on each lap-crossing step no segment matches and full drive is applied. The default when nothing matches is full throttle. | < 0.1 % |
| 15 | Low | [F L91](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L91), [L63](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L63) | Equivalent mass divides by rim radius `r_w` instead of rolling radius `r_t`, models the rim as a solid disc, and omits the motor rotor inertia (J_rotor·GR²/r_t², about 0.45–0.75 kg equivalent for 3,060–5,100 g·cm²; check the datasheet for our winding). | +0.02 % |
| 16 | Low | [F L47](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L47), [L601](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L601) | `T_motor = 0.9` is really the wheel-referred torque with no drivetrain loss; the motor actually runs at 0.9677 N·m. Choices made "at the map peak" (0.903 N·m) are off by 7.5 % in torque. | Mislabel |
| 17 | Low | [F L594–599](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L594) | Gear-shift branch is dead (both branches give `GR`). Re-enabling `GR/0.7` would *not* raise wheel force, because force is computed from the fixed `T_out`. | Misleading if re-enabled |
| 18 | Low | [F L790](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L790), [L751–753](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L751) | A local function sits mid-script with more script code after it. R2026a accepts this; releases before R2024a probably refuse the file (not tested). The motor-map loop reuses the sim's `i` and `V`. The map (I0 = 0.493/2, `3.14`) and the sim's function (0.490/2, `pi`) use slightly different constants. | Portability; confusion |
| 19 | Low | [F L891–900](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L891), [L41](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L41), [L638](Old%20Stuff/Vehicle%20Model/Shell_Track_Profile_Final.m#L638) | Legends list three series ("Low Gear", "High Gear", "Hyper Accel") but only one is plotted. Mass has an unexplained `-1.2` kg. There are many unused variables (`rad`, `isCoasting`, `P_drag`, `F_inertia`, `EffMotorTot`, most `v_use`). No parameter has a source; it isn't stated whether `m` includes driver and ballast. | Traceability |

### 7.2 Parameters disagree across files

No two models share one parameter set. Values, with the file line where each is set:

| Parameter | Final | Feb9 | Improved / Hypermiling | Sensitivity | Shell_Eco | vehiclemodel | Evidence / realistic |
|---|---|---|---|---|---|---|---|
| Air density ρ (kg/m³) | 1.004 "Colorado" | 1.004 | 1.004 | 1.004 | 1.004 | **1.2 "Indianapolis"** | ≈ 1.15–1.19 at IMS |
| C_rr | 0.0056 | 0.0056 | 0.0056 | 0.0056 | 0.004 | 0.004 (× v, wrong) | Unsourced; published top-car values 0.0008–0.0015, 0.0015–0.003 plausible on IMS asphalt (estimate); **measure by coast-down** |
| C_d × A_f (m²) | 0.1 × 0.71 = 0.071 | 0.071 | 0.071 | 0.071 | 0.2 × 0.5 = 0.10 | 0.10 | A_f = 0.71 m² is large for a prototype (PAC-Car II: 0.25 m²); published CdA 0.019–0.088 |
| Mass m (kg) | sum(Wd) − 1.2 = 83.6 | 83.6 | 84.8 / 83.8 (−1) | 84.8 | 84.8 | 85 | Unexplained offsets; driver/ballast status undocumented |
| Gear ratio | 9.23 | 9.23 (+ low 9.23/0.7) | 8 / 8 | (12, commented) | 12 (unused) | 5 / 9.12 | 9.23 = 120/13 is the raced ratio |
| Motor torque cmd (N·m) | 0.9 | 0.91 | 1.49 | n/a (F = 131.97 N) | n/a | stall-line | |
| Battery V / I_max | 60 V / 10 A | 60 / **1 A** | 48 / **1 A** | 60 / 10 | 60 / 35 | 48 / 56.9 (stall) | 60 V is the rule maximum |
| Drivetrain / controller η | 0.93 / 0.95 | 0.93 / 0.95 | **1.0** / none | 0.90 (can exceed 1) / none | 0.95 "RANDOM" / none | 0.85 × ad hoc | |
| Lap length (m) | 3825 (comments 3826, 3926) | 3926 | 3926 | none (fixed time) | none | 15,700 total | Official 3832.5; GPS 3849 |
| Time horizon | 2100 s (35 min) | 1800 s | 1800 s | 1800 s | 1800 s | 1800 s | Rule: 35 min |
| dt (s) | 0.1 | 0.1 | 0.1 | **1** | **1** | 0.01 | 0.1 is converged (§6) |

### 7.3 `Shell_Sensitivity_With_Hypermiling.m`: do not use its results

`S` below means [`Shell_Sensitivity_With_Hypermiling.m`](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m).

| Sev. | Where | Issue |
|---|---|---|
| High | [S L120](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L120), [L286](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L286) | "Energy" is wheel work (`P_x = F_req*v`). Drivetrain and motor efficiency never enter the kWh, so the drivetrain-efficiency sensitivity comes out *backwards*. With no motor-efficiency map, pulse-and-glide can only look worse than steady cruise in this model. |
| High | [S L279](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L279), [L226](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L226) | The perturbed `m_eq_p` is computed but the baseline `m_eq` is used, so the wheel/tyre rows (7–10) are exactly flat (286.91). |
| Medium | [S L98](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L98) | The baseline overrides the limiter with an unsourced `F = 131.9675` N, but the sweep uses the limiter. The "Baseline" line (285.06) doesn't match the sweep's own 0 % column (286.91). |
| Medium | [S L52](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L52), [L114](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L114) | dt = 1 s explicit Euler. One step (1.46 m/s) is larger than the whole 0.92 m/s pulse band, so speed overshoots to 11.6 m/s. Also, every case runs a fixed 1800 s instead of the race distance, so cases cover different distances. |
| Low | [S L230–245](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L230) | `isPulsing` is not reset between cases, so results depend on run order and the first case starts gliding from rest. |
| Low | [S L218–222](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L218) | The `T_eff ≤ 1` clamp is commented out, so **four** cases (+15 % … +30 %) use drivetrain efficiency > 100 %. |
| Low | [S L180](Old%20Stuff/Vehicle%20Model/Shell_Sensitivity_With_Hypermiling.m#L180) | `effifs` is preallocated 11×10 but filled 12×13 (MATLAB grows it silently); comments are stale. |

### 7.4 Earlier track models (`Feb9Model.m`, `Improved_Shell_track_profile.m`, `Shell_track_Hypermiling.m`)

- **Feb9Model**
  - [`I_max = 1` A](Old%20Stuff/Vehicle%20Model/Feb9Model.m#L33) is exceeded about 8× (the motor draws 5.8–8.3 A). The limiter is bypassed at [L275](Old%20Stuff/Vehicle%20Model/Feb9Model.m#L275).
  - Its time vector is fixed at [1800 s](Old%20Stuff/Vehicle%20Model/Feb9Model.m#L85). With Final's slower speed band the race takes 32 min and the script **crashes** on `t(i)` instead of reporting a DNF.
  - The low gear ([L567](Old%20Stuff/Vehicle%20Model/Feb9Model.m#L567)) changes only the motor operating point, not the wheel force.
  - Everything in §7.1 #1–#4 applies to it too: the kt fix gives +11.6 %, a coupled motor −10.1 %, and ρ = 1.18 −6 %.
- **Improved / Hypermiling** count energy at the wheel with `T_eff = 1` and no motor model ([Improved L57](Old%20Stuff/Vehicle%20Model/Improved_Shell_track_profile.m#L57), [L489](Old%20Stuff/Vehicle%20Model/Improved_Shell_track_profile.m#L489); [Hypermiling L62](Old%20Stuff/Vehicle%20Model/Shell_track_Hypermiling.m#L62), [L487](Old%20Stuff/Vehicle%20Model/Shell_track_Hypermiling.m#L487)). Their 285–289 mi/kWh corresponds to about 210 at the battery.
- **Hypermiling**
  - It had the *correct* turn-13 coast, based on `s_13` ([L429](Old%20Stuff/Vehicle%20Model/Shell_track_Hypermiling.m#L429)). That fix was lost because Feb9/Final descend from Improved, which uses the fixed [3101 m window](Old%20Stuff/Vehicle%20Model/Improved_Shell_track_profile.m#L424).
  - One of its six copy-pasted pulse blocks uses a magic `F = 11.92 / r_t` ([L257](Old%20Stuff/Vehicle%20Model/Shell_track_Hypermiling.m#L257)).
- **All three** report a hard-coded 15,704 m whether or not the car finished, and share Final's plot problems (§7.1 #13).

### 7.5 Motor and gearing tools

| Sev. | Where | Issue |
|---|---|---|
| Critical | [`motorgearstuff.m` L43](Old%20Stuff/Vehicle%20Model/motorgearstuff.m#L43) | `W_wheel = sCar/(2*3.14*r_t)` is in **rev/s but used as rad/s**. The true motor speed (3,522 rpm) exceeds the motor's 1,960 rpm no-load speed, so GR = 10 is infeasible and the overspeed check is defeated. "990 mi/kWh" is inflated about 6.3×. [L71](Old%20Stuff/Vehicle%20Model/motorgearstuff.m#L71) also sets `Kt = 1/Ke` (should be `Kt = Ke`). |
| High | [`EC90_30V_plot.m` L112](Old%20Stuff/Vehicle%20Model/EC90_30V_plot.m#L112) | `t_coast = (21.5-19.5)/(12/85)` divides **mph by m/s²**: 14.2 s instead of 6.3 s, so the number of pulse cycles and the race energy are wrong. |
| High | [`EC90_30V_plot.m` L171](Old%20Stuff/Vehicle%20Model/EC90_30V_plot.m#L171) | `Pelec = V_input * I` with a fixed 60 V (no PWM step-down; back-EMF is computed but unused). This overstates electrical power 1.1–2.1×. Corrected for this and the L112 bug, the file gives about 150 mi/kWh, not 232. (Final correctly uses `V_eff*I`.) |
| High | [`EC90_lastyear_estimation.m` L233](Old%20Stuff/Vehicle%20Model/EC90_lastyear_estimation.m#L233) | Declares [48 V](Old%20Stuff/Vehicle%20Model/EC90_lastyear_estimation.m#L8) but evaluates the motor at a hard-coded 60 V. It also scales by 1.5 instead of 48/30 = 1.6 ([L13–14](Old%20Stuff/Vehicle%20Model/EC90_lastyear_estimation.m#L13)). At 48 V its cruise speed exceeds the motor's no-load speed. |
| Medium | [`EC90_30V_plot.m` L10–14](Old%20Stuff/Vehicle%20Model/EC90_30V_plot.m#L10), [L213](Old%20Stuff/Vehicle%20Model/EC90_30V_plot.m#L213) | Same kt/ke and I0/2 problems as §7.1 #1 and #4. Road load is a constant 12 N at every speed (the comment says 8 N). Efficiency maps extend past stall with non-physical current. |

### 7.6 Oldest models (`vehiclemodel*.m`, `Shell_Eco_Model.m`)

- **`vehiclemodel.m` and `vehiclemodel (1).m` have wrong physics throughout; don't use them for decisions.**
  - [`m_eq = 3*m + ...`](Old%20Stuff/Vehicle%20Model/vehiclemodel.m#L32) gives 256 kg instead of 86 kg.
  - [`F_rr = v*RR*m*g`](Old%20Stuff/Vehicle%20Model/vehiclemodel.m#L96) makes rolling resistance ∝ speed.
  - Motor "efficiency" [scales force, not power](Old%20Stuff/Vehicle%20Model/vehiclemodel.m#L82), giving 2.1 N at launch.
  - Electrical power is [back-EMF × current](Old%20Stuff/Vehicle%20Model/vehiclemodel.m#L105).
  - Coasting never triggers ([`v_peak = 9`](Old%20Stuff/Vehicle%20Model/vehiclemodel.m#L35) is above the 8.9 m/s cap).
  - The car never finishes, yet an efficiency is reported.
  - The power plot is labelled W but the data is kW ([L147](Old%20Stuff/Vehicle%20Model/vehiclemodel.m#L147)).
- **`vehiclemodel (1).m`**
  - It is not a valid MATLAB file name, so `run()` refuses it.
  - Its [motor constants](Old%20Stuff/Vehicle%20Model/vehiclemodel%20%281%29.m#L45) give Kt = 0.231 N·m/A but Ke = 0.122 V·s/rad.
  - Its `eff_m_in` table goes down to −103 ([L85](Old%20Stuff/Vehicle%20Model/vehiclemodel%20%281%29.m#L85)).
  - (The earlier review said that at start-up "full current/torque accelerates the car for zero energy". In fact that lasts one 10 ms step and the force is only 2.1 N; the force-scaling bug above is the real start-up defect.)
- **`Shell_Eco_Model.m`**
  - It crashes at [L129](Old%20Stuff/Vehicle%20Model/Shell_Eco_Model.m#L129), `average(P_x)`; there is no such function, use `mean`.
  - `g_r` and `kv` ([L36–37](Old%20Stuff/Vehicle%20Model/Shell_Eco_Model.m#L36)) are never used, so there is no motor speed limit.
  - dt = 1 s ([L48](Old%20Stuff/Vehicle%20Model/Shell_Eco_Model.m#L48)) overshoots the 9.16 m/s cap to 11.2 m/s, which is then held for 30 min ([L81–88](Old%20Stuff/Vehicle%20Model/Shell_Eco_Model.m#L81)).
  - Energy is wheel work only.

### 7.7 Repository-level problems

- There is no single source of truth: parameters, lap length and time limit are copy-pasted and diverge (§7.2).
- There are no tests, no functions and no documented entry point. Every script starts with `clear`, so scripts can't be composed or looped.
- The GPS track data is present but no script reads it.
- There are duplicate and near-duplicate files: two `vehiclemodel`s, two EC90 scripts, and three generations of track model.

## 8. What changed from the previous review

The previous version of this README (commit `f814f6a`) was a useful first pass. Checked against the code:

- **Accurate:**
  - the limiter bypass (#1);
  - the conflicting lap lengths (#2, lines correct);
  - sensitivity 3a, 3b, 3c and 3e;
  - accessory power (#4);
  - Colorado air density and unused GPS (#5);
  - the `average()` crash (#6);
  - `3*m` and velocity-proportional rolling resistance (#7a, 7b);
  - "no authoritative file" (#8);
  - the runtime numbers (33.07 min / 0.0402 kWh / 236.4 mi/kWh reproduce exactly).
- **Overstated or misdirected:**
  - The limiter bypass is real but has **no effect** at the current parameters.
  - The lap-length mismatch inside Final is only 0.02 %.
  - These were given as the main reasons to distrust the result. The bigger effects are §7.1 #1–#5.
  - Claim 7c (start-up energy) is overstated; see §7.6.
- **Understated:** 3d. Four sensitivity cases exceed 100 % drivetrain efficiency, not just +30 %.
- **Missed:**
  - the kt/ke inconsistency;
  - the decoupled-motor coasting assumption;
  - I0 halving;
  - negative coast distances;
  - corner-radius errors and where the tight corners really are;
  - the unit bugs in the motor tools;
  - wheel-only energy in three models;
  - the unbounded loop.
- **Broken links:** all its links pointed to `C:/Paul Stuff/...`, which went stale when the files moved to `Old Stuff/`.

## 9. Recommended next steps

1. **Resolve the two hardware questions:**
   - Is there a freewheel or clutch in the drive?
   - Is kt really derated 0.9 under sinusoidal drive, and if so, is it counted separately from the 0.95 controller efficiency?
2. **Measure the car.** Do a coast-down test for C_rr and CdA, and weigh the car, driver and ballast. These dominate the result and none is sourced today.
3. **Build the new lapsim as MATLAB functions**, with one parameter struct (units and source per value), a track loaded from the GPS file (grade and curvature), explicit limits, a race ended by distance, and `matlab.unittest` tests (energy balance, distance, time limit, limits respected). See the proposed layout in [`AGENTS.md`](AGENTS.md#proposed-lapsim-architecture).
4. **Keep `Old Stuff/` as read-only reference.** Use `Shell_Track_Profile_Final.m` with its legacy parameters as a regression benchmark (236.4 mi/kWh) while porting.

## 10. How this review was done

This review was done on 2026-09-23.

- **Line-by-line reading:** every script was read line by line by independent reviewers (one per file group, plus cross-cutting passes for parameters, physics/numerics, track data and the previous README's claims).
- **Refutation checks:** most findings were then re-checked by two further reviewers asked to *refute* them. Severities were adjusted where they disagreed. Findings that were not re-checked independently were confirmed by re-reading the cited lines.
- **Measured figures:** every numeric effect quoted above comes from running the scripts in MATLAB R2026a Update 5 (unmodified, or modified in memory only).
- **Track analysis:** done in Python from `sem_2023_us.csv`.
- **Rules:** taken from the official SEM 2024–2026 rulebooks.
