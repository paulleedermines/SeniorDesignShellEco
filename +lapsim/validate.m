function validate(p, track)
%VALIDATE Reject ambiguous/nonphysical inputs before running a case.
template = lapsim.defaultParameters();
checkFields(p,template,'p');
positive = {'vehicle.chassis_mass_kg','vehicle.wheel_radius_m','vehicle.wheelbase_m', ...
    'vehicle.front_track_m','motor.torque_constant_Nm_A','motor.resistance_ohm', ...
    'motor.max_current_A','motor.max_torque_Nm','motor.max_power_W','motor.max_rpm', ...
    'drivetrain.gear_ratio','battery.open_circuit_voltage_V','battery.max_current_A', ...
    'battery.max_terminal_power_W','battery.usable_capacity_Wh','environment.air_density_kgpm3', ...
    'environment.gravity_mps2','race.time_limit_s','race.target_mi_per_kWh', ...
    'sim.dt_s','sim.max_time_s','strategy.cruise_speed_mps','strategy.pulse_min_speed_mps', ...
    'strategy.pulse_max_speed_mps','strategy.pulse_motor_torque_Nm', ...
    'strategy.braking_deceleration_mps2'};
for i = 1:numel(positive)
    assert(getfieldpath(p,positive{i})>0,'lapsim:InvalidParameter','%s must be positive.',positive{i});
end
signed = {'environment.headwind_mps','vehicle.cg_lateral_offset_m'};
checkNonnegative(p,'',signed);
fractions = {'drivetrain.efficiency','controller.efficiency','controller.max_duty_cycle','tires.corner_utilization'};
for i=1:numel(fractions)
    x=getfieldpath(p,fractions{i});
    assert(x>0 && x<=1,'lapsim:InvalidParameter','%s must be in (0,1].',fractions{i});
end
assert(p.tires.corner_utilization<1,'lapsim:InvalidParameter','corner_utilization must be below 1 to reserve braking grip.');
assert(p.tires.friction_coefficient>0,'lapsim:InvalidParameter','Tire friction must be positive.');
assert(p.vehicle.cg_from_front_m>0 && p.vehicle.cg_from_front_m<p.vehicle.wheelbase_m, ...
    'lapsim:InvalidParameter','Combined CG must lie between axles.');
halfWidth = p.vehicle.front_track_m/2*(1-p.vehicle.cg_from_front_m/p.vehicle.wheelbase_m);
assert(abs(p.vehicle.cg_lateral_offset_m)<halfWidth,'lapsim:InvalidParameter','CG lies outside the three-wheel support triangle.');
assert(p.battery.min_terminal_voltage_V<p.battery.open_circuit_voltage_V, ...
    'lapsim:InvalidParameter','Minimum voltage must be below open-circuit voltage.');
assert(p.race.laps>=1 && p.race.laps==fix(p.race.laps),'lapsim:InvalidParameter','laps must be a positive integer.');
assert(p.strategy.pulse_min_speed_mps<p.strategy.pulse_max_speed_mps, ...
    'lapsim:InvalidParameter','Pulse minimum must be below pulse maximum.');
assert(any(strcmp(p.strategy.mode,{'cruise','pulse_coast'})),'lapsim:InvalidParameter','Unknown strategy.mode.');
assert(p.sim.dt_s<=2,'lapsim:InvalidParameter','Use dt_s <= 2 s; 0.25 s or less is recommended.');
assert(istable(track) && height(track)>0,'lapsim:InvalidTrack','Track must be a nonempty table.');
cols = {'length_m','grade','radius_m','speed_limit_mps'};
assert(all(ismember(cols,track.Properties.VariableNames)),'lapsim:InvalidTrack','Missing required track columns.');
for i=1:numel(cols)
    a=track.(cols{i});
    assert(isnumeric(a) && isreal(a) && iscolumn(a) && ~any(isnan(a)), ...
        'lapsim:InvalidTrack','Track column %s must be a real numeric column without NaNs.',cols{i});
end
assert(all(isfinite(track.length_m) & track.length_m>0),'lapsim:InvalidTrack','Segment lengths must be finite and positive.');
assert(all(isfinite(track.grade) & abs(track.grade)<1),'lapsim:InvalidTrack','Grade is rise/run and must have magnitude <1.');
assert(all(track.radius_m>0) && all(track.speed_limit_mps>0),'lapsim:InvalidTrack','Radius and limits must be positive (Inf allowed).');
end

function checkFields(p,t,path)
assert(isstruct(p) && isscalar(p),'lapsim:InvalidParameter','%s must be a scalar struct.',path);
names=fieldnames(t);
assert(isempty(setdiff(fieldnames(p),names)),'lapsim:InvalidParameter','Unknown field in %s (possible typo).',path);
for i=1:numel(names)
    f=names{i}; key=[path '.' f];
    assert(isfield(p,f),'lapsim:InvalidParameter','Missing %s.',key);
    if isstruct(t.(f))
        checkFields(p.(f),t.(f),key);
    elseif isnumeric(t.(f))
        assert(isnumeric(p.(f)) && isreal(p.(f)) && isscalar(p.(f)) && isfinite(p.(f)), ...
            'lapsim:InvalidParameter','%s must be a finite real scalar.',key);
    else
        assert((ischar(p.(f)) && isrow(p.(f))) || (isstring(p.(f)) && isscalar(p.(f))), ...
            'lapsim:InvalidParameter','%s must be text.',key);
    end
end
end

function checkNonnegative(p,path,signed)
names=fieldnames(p);
for i=1:numel(names)
    f=names{i}; key=f; if ~isempty(path), key=[path '.' f]; end
    x=p.(f);
    if isstruct(x), checkNonnegative(x,key,signed);
    elseif isnumeric(x) && ~ismember(key,signed)
        assert(x>=0,'lapsim:InvalidParameter','%s must be nonnegative.',key);
    end
end
end

function x=getfieldpath(p,path)
parts=strsplit(path,'.'); x=p;
for i=1:numel(parts), x=x.(parts{i}); end
end
