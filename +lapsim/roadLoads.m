function loads = roadLoads(p, speed, grade, radius)
%ROADLOADS Forces along direction of travel; positive values oppose motion.
mass = p.vehicle.chassis_mass_kg+p.vehicle.driver_mass_kg;
equivalentMass = mass+p.vehicle.wheel_inertia_kgm2/p.vehicle.wheel_radius_m^2;
theta = atan(grade); g=p.environment.gravity_mps2;
normal = mass*g*cos(theta);
relativeAir = speed+p.environment.headwind_mps;
loads.aero_N = 0.5*p.environment.air_density_kgpm3*p.aero.cd*p.aero.frontal_area_m2*relativeAir*abs(relativeAir);
loads.rolling_N = (p.tires.crr+p.tires.crr_speed_per_mps*speed)*normal;
loads.grade_N = mass*g*sin(theta);
loads.total_N = loads.aero_N+loads.rolling_N+loads.grade_N;
loads.mass_kg = mass;
loads.equivalent_mass_kg = equivalentMass;
loads.normal_N = normal;
loads.lateral_acceleration_mps2 = speed^2/radius;
% Approximate friction circle with lateral force distributed by normal load.
muLong = sqrt(max(0,p.tires.friction_coefficient^2-(speed^2/radius/(g*cos(theta)))^2));
L=p.vehicle.wheelbase_m; h=p.vehicle.cg_height_m;
% Nrear = rearBase + alpha*(drive-brake-coastDrag).
loads.rear_base_N = (normal*p.vehicle.cg_from_front_m + mass*g*sin(theta)*h ...
    + loads.aero_N*p.aero.force_height_m - mass/equivalentMass*loads.total_N*h)/L;
loads.transfer_per_force = mass/equivalentMass*h/L;
alpha=loads.transfer_per_force;
% Two front contacts supply the roll-restoring moment; the rear is central.
loads.roll_moment_Nm=mass*loads.lateral_acceleration_mps2*h ...
    + normal*abs(p.vehicle.cg_lateral_offset_m);
maxRearForRoll=normal-2*loads.roll_moment_Nm/p.vehicle.front_track_m;
if 1-muLong*alpha>0
    drive = max(0,muLong*loads.rear_base_N/(1-muLong*alpha));
else
    drive = muLong*normal;
end
% Include tire grip and axle-lift constraints, with no downforce model.
if alpha>0
    drive=min(drive,max(0,(maxRearForRoll-loads.rear_base_N)/alpha));
    brakeLift=max(0,loads.rear_base_N/alpha);
else
    brakeLift=Inf;
end
loads.max_drive_N=min(drive,muLong*normal);
loads.max_brake_N=min([p.vehicle.max_brake_force_N,muLong*normal,brakeLift]);
loads.longitudinal_friction=muLong;
end
