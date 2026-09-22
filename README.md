Current Code Status:

Highest-priority findings
1. The main model calculates operating limits and then bypasses them.
   [Shell_Track_Profile_Final.m (line 284)](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Track_Profile_Final.m:284) selects the lower traction/power force, but line 290 immediately overwrites it with the geared motor torque. Consequently, I_max, maximum wheel power, and traction limits do not constrain acceleration. The motor’s physically_possible result is also calculated but never enforced.
2. Track distance has several conflicting definitions.
   The main file references 3,825 m, 3,826 m, and 3,926 m per lap, while reporting 15,304 m. See [Shell_Track_Profile_Final.m (line 204)](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Track_Profile_Final.m:204), [line 206 (line 206)](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Track_Profile_Final.m:206), and [line 575 (line 575)](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Track_Profile_Final.m:575). This directly affects race time and mi/kWh.
3. The sensitivity analysis is not reliable yet.
   In [Shell_Sensitivity_With_Hypermiling.m](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Sensitivity_With_Hypermiling.m):
   - m_eq_p is correctly calculated at line 226, but the simulation uses baseline m_eq at line 279.
   - isPulsing is not reset before each parameter run, making results dependent on test order.
   - The baseline uses a hard-coded 131.9675 N force at line 98, while sensitivity runs use the calculated limiter.
   - Drivetrain efficiency can exceed 100% during the +30% case because the clamp is commented out.
   - The result array is initialized as 11×10 but loops over 12×13 cases. MATLAB silently expands it, hiding the mismatch.
4. The reported energy excludes known loads and some feasibility constraints.
   AccessoryPower is explicitly set to zero and never used at [Shell_Track_Profile_Final.m (line 72)](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Track_Profile_Final.m:72). Controller/comms power, battery losses, current limits, and impossible duty-cycle points therefore do not consistently affect the reported efficiency.
5. Several environmental assumptions conflict with the modeled event.
   The final script describes an Indianapolis competition but uses “Air density in Colorado” at [Shell_Track_Profile_Final.m (line 33)](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Track_Profile_Final.m:33). The included GPS/elevation data is not consumed by any script, so grade is also omitted.
6. One model has a confirmed runtime failure.
   [Shell_Eco_Model.m (line 129)](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/Shell_Eco_Model.m:129) calls nonexistent MATLAB function average; it should conceptually use mean. I verified this failure in MATLAB R2026a.
7. The older vehicle model contains major physics errors.
   In [vehiclemodel.m](C:/Paul Stuff/SeniorDesignShellEco/Vehicle Model/vehiclemodel.m):
   - Equivalent mass uses 3*m at line 32.
   - Rolling-resistance force is incorrectly proportional to velocity at line 96.
   - At startup, voltage is zero while full current/torque accelerates the car, meaning initial mechanical work consumes zero modeled energy.
   I would not use this script for design decisions.
8. It is unclear which files are authoritative.
   The repository contains multiple generations of substantially duplicated models, including Feb9Model, Improved_*, Shell_track_*, two EC90 versions, and vehiclemodel (1).m. The latter is also an invalid MATLAB filename according to Code Analyzer. There is no README, test suite, or documented entry point.
Main-script runtime result
The apparent current entry point, Shell_Track_Profile_Final.m, runs successfully and reports:
- Completion time: 33.07 minutes
- Energy: 0.0402 kWh
- Efficiency: 236.4 mi/kWh
I would treat those as provisional because of the bypassed force limits and inconsistent race distance. Its plots are also misleading after completion: the simulation retains zero-filled preallocated samples, resets position every lap, and finally writes x(end)=15304 to force the displayed endpoint.
Changes I would make first
1. Choose one authoritative model and archive legacy scripts.
2. Establish a single track-length constant and one parameter set with units and sources.
3. Enforce torque, current, voltage, power, traction, and speed limits in one place.
4. Convert the scripts into functions such as simulateVehicle(params, track, strategy).
5. Fix the sensitivity analysis so every case starts from identical state and uses perturbed derived values.
6. Add validation tests for energy balance, dimensional consistency, distance, completion time, and physical bounds.
7. Add a short README covering the entry point, MATLAB version, assumptions, and expected benchmark output.