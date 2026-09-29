# Model definition and calibration

All calculations use SI units. The modeled vehicle has two front wheels and one central driven rear wheel. Each simulation starts with fresh controller state. Parameters and the track table determine the outputs; no legacy workspace variables are used.

## Dynamics and center of mass

Total mass is `chassis_mass_kg + driver_mass_kg`. Chassis mass includes battery, wheels and drivetrain. Equivalent accelerating mass is `mass + wheel_inertia_kgm2 / wheel_radius_m^2`, where the inertia is the sum for all wheels. The loaded tire radius is used for inertia, gearing and torque conversion. Motor/gear inertia is omitted because the motor disconnects during coast.

At local grade angle `theta = atan(grade)`, the opposing forces are:

- Rolling: `(crr + crr_speed_per_mps * v) * mass * g * cos(theta)`.
- Aerodynamic: `0.5 * rho * Cd * area * (v + headwind) * abs(v + headwind)`.
- Grade: `mass * g * sin(theta)`.

Acceleration is `(wheel drive - brake - road loads - coast drag) / equivalent mass`. Aero and grade work are signed: a faster following wind or downhill segment can supply energy. Headwind is a local along-travel value, not a compass wind vector. All segments use the same configured local wind. Grade and radius are constant within a segment; subdivide the track to model changing geometry.

Rear normal load includes static weight distribution plus longitudinal acceleration, grade and aero force-height moments. Drive is constrained by rear grip, axle lift, and a friction circle that reserves lateral grip. Braking assumes ideal distribution over all wheels, subject to grip and `max_brake_force_N`. Residual coast drag is constrained by rear grip and shares the braking allowance.

The static lateral acceleration cap uses the smaller of tire grip and support-triangle rollover acceleration, multiplied by `corner_utilization`. The triangle narrows toward the single rear wheel, so longitudinal CG location, height and lateral offset all matter. Drive force is also constrained by the dynamic front normal load available to resist roll. The feasibility check tests this roll moment during every interval. This is a quasi-static screening approximation: suspension, camber, tire slip transients, banking, aero side force and downforce are omitted.

The road-load and load-transfer basis is described in [MathWorks Vehicle Body](https://www.mathworks.com/help/sdl/ref/vehiclebody.html) and [Longitudinal Vehicle](https://www.mathworks.com/help/sdl/ref/longitudinalvehicle.html). This implementation is independent MATLAB code and does not require Simscape.

## Electrical energy and operating limits

The motor is an effective DC equivalent. Its torque and back-EMF constants are equal in coherent SI units, removing the old model's separately scaled constants. See [maxon's explanation of motor constants](https://support.maxongroup.com/hc/en-us/articles/360005873794-Motor-constants). Effective winding current is distinct from battery current. Calibrate the equivalent constants together for the actual BLDC controller and commutation convention.

For motor speed `omega = v * gear_ratio / wheel_radius_m`:

```text
shaft torque = wheel force * wheel radius / (gear ratio * transmission efficiency)
loss torque = constant loss torque + viscous coefficient * omega
motor current = (shaft torque + loss torque) / kt
motor voltage = kt * omega + motor resistance * motor current
terminal power = motor voltage * motor current / controller efficiency + auxiliaries
terminal power = (open-circuit voltage - battery resistance * battery current) * battery current
chemical power = open-circuit voltage * battery current
```

The stable, lower-current battery root is used. Limits include effective winding current, usable shaft torque/power, engaged RPM, PWM duty at loaded battery voltage, pack current, terminal power, and minimum terminal voltage. A request above these limits reduces delivered force. Startup consumes copper-loss energy even at zero speed. Accessories that cannot be powered stop the run explicitly.

Battery voltage and resistance are constant approximations. Capacity is a usable **chemical energy** budget, distinct from delivered terminal energy used for mi/kWh. Battery SOC/temperature effects, motor thermal accumulation, detailed efficiency maps, switching dynamics and regenerative braking are omitted. A disconnected motor incurs no motor losses on coast; configure residual wheel drag separately. Auxiliaries remain powered throughout.

## Strategy, integration and feasibility

Cruise requests enough force to reach and hold its speed, subject to limits. Pulse/coast alternates between two speeds using configurable pulse torque. Both controllers anticipate future lower limits, including the next lap. They attempt natural coasting before slower corners and can apply mechanical brakes. `braking_deceleration_mps2` sets the **approach plan**; the brake/tire force is the hard physical limit. Severe descents or insufficient brakes can produce speed-limit violations, making the result infeasible.

The integrator evaluates speed-dependent forces at the interval midpoint. Displacement is mean speed times actual duration. Steps shorten at segment boundaries, zero speed, battery depletion and the finish. All work and power terms use that same interval:

```text
chemical battery energy = battery + motor + controller + transmission losses
                        + auxiliary + aero + rolling + grade + brake + coast-drag work
                        + kinetic energy change
```

`balance_residual_Wh` reports energy closure. The solver rejects a step that fails to converge instead of quietly reporting inconsistent force and energy. No post-finish zeros are integrated, and distance never resets between laps. Finish speed is retained and leftover kinetic energy is counted; no free braking or energy recovery occurs at the finish.

Trace rows contain interval-end time, distance and speed. Forces, current, RPM and power are interval-midpoint values. Integrate trace power with `sum(power .* dt_s)`, not `trapz` of the endpoint timestamps. `mode` is 1 driving, 0 coasting, or -1 braking. RPM is the engaged-equivalent speed even when the motor is freewheeling.

`feasible` requires full distance, the time and average-speed requirements, minimum combined mass, usable capacity, and the modeled physical constraints. `target_met` also requires the terminal-energy goal. Defaults use the user's 15.33 km/four laps/35 minutes, 16.6 mph minimum average, 50 kg minimum total mass and 300 mi/kWh goal. These are user-supplied requirements, not independently verified rules. Average speed permits a standing start and slower corners. Setting `minimum_average_speed_mps = 0` removes the additional planning-speed constraint while retaining the time limit.

## Provenance and calibration

The old 83.61 kg scale sum, Cd 0.1, area 0.71 m², Crr 0.0056, wheelbase 1.397 m, rolling radius 0.2413 m, wheel-inertia construction, CG height 0.375 m, ratio 9.23, transmission efficiency 0.93 and controller efficiency 0.95 come from the archived final script. Their accuracy is unknown. A **70 kg driver is added as an explicit placeholder**, because the old readings did not establish driver inclusion. Replace both mass inputs and the combined CG with measured values. If your scale reading includes the driver, enter that combined reading as chassis mass and set driver mass to zero.

Additional illustrative placeholders include front track 0.8 m, 5 W auxiliaries, 0.15 ohm pack resistance, 150 Wh capacity, 45 V minimum bus, and torque/current/power/RPM limits. Air density is 1.2 kg/m³. The effective SI motor constant is 0.136, resistance is the old 0.275 ohm, and friction is approximated from the old no-load calibration. None is a new measurement. Default pulse bands retain 16.5–18.5 mph.

The sample course uses the old slow-corner position with a corrected interpretation of 135 degrees for its approximate radius. Its final segment is adjusted so that one lap is 3,832.5 m. Other curvature and all grade are unknown and treated as straight/flat. The archived 2023 GPS data is a different course description and is not automatically consumed.

Suggested calibration sequence:

1. Weigh the complete car and driver and measure each wheel load. Determine combined CG, loaded tire radius and front track.
2. Fit coastdown measurements to rolling drag and CdA on a surveyed surface, using runs in both directions to account for wind/grade. Separate Cd from frontal area using independent aero evidence.
3. Log battery voltage/current, accessory load, wheel speed and torque in steady and pulsed operation. Fit motor/controller losses and pack sag.
4. Replace the course with measured segment lengths, curvature, grades and speed limits. Check a closed lap's net elevation change with `sum(length_m .* sin(atan(grade)))`.
5. Compare measured attempt time, terminal Wh and speed trace with predictions. Halve `dt_s` to check numerical stability, then vary uncertain inputs to build target ranges.

Sensitivity ranges describe scenarios, not statistical confidence intervals. Strategy sweeps identify the best feasible case on the supplied grid, not a globally optimal design or controller.
