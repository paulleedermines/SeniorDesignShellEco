function result = simulate(p, track)
%SIMULATE Time-domain, three-wheel RWD electric efficiency LapSim.
% result.trace contains interval-end states and interval-average power.
% Power integration uses each actual dt_s, including boundary/finish steps.
lapsim.validate(p,track);
lapLength=sum(track.length_m); totalDistance=p.race.laps*lapLength;
lengths=repmat(track.length_m,p.race.laps,1);
ends=cumsum(lengths); starts=[0;ends(1:end-1)];
grades=repmat(track.grade,p.race.laps,1);
radii=repmat(track.radius_m,p.race.laps,1);
physicalCaps=cornerCaps(p,grades,radii,repmat(track.speed_limit_mps,p.race.laps,1));
if strcmp(p.strategy.mode,'cruise'), topSpeed=p.strategy.cruise_speed_mps;
else, topSpeed=p.strategy.pulse_max_speed_mps; end
caps=min(physicalCaps,topSpeed);
% Each future lower limit constrains approach speed, including lap transitions.
planBrake=min(p.strategy.braking_deceleration_mps2, ...
    0.7*p.tires.friction_coefficient*p.environment.gravity_mps2);
segment=1; t=0; x=0; v=p.sim.initial_speed_mps; pulse=true;
energy=zeros(1,12); violation=max(0,v-physicalCaps(1)); electricalOK=true; axleOK=true; rollOK=true;
reason='max_time'; count=0;
names={'time_s','distance_m','speed_mps','dt_s','interval_speed_mps','acceleration_mps2', ...
    'segment','lap','speed_limit_mps','drive_force_N','brake_force_N','aero_force_N', ...
    'rolling_force_N','grade_force_N','rear_normal_N','motor_current_A','battery_current_A', ...
    'battery_voltage_V','duty_cycle','motor_rpm','terminal_power_W','battery_power_W', ...
    'terminal_energy_Wh','battery_energy_Wh','mode'};
data=zeros(ceil(p.sim.max_time_s/p.sim.dt_s)+numel(ends)+10,numel(names));
mass=p.vehicle.chassis_mass_kg+p.vehicle.driver_mass_kg;
meq=mass+p.vehicle.wheel_inertia_kgm2/p.vehicle.wheel_radius_m^2;
initialKE=0.5*meq*v^2;
while t<p.sim.max_time_s-1e-9 && x<totalDistance-1e-8
    while segment<numel(ends) && x>=ends(segment)-1e-8, segment=segment+1; end
    dt=min(p.sim.dt_s,p.sim.max_time_s-t);
    if v<=p.strategy.pulse_min_speed_mps, pulse=true;
    elseif v>=p.strategy.pulse_max_speed_mps-1e-6, pulse=false; end
    % Shorten the interval to end exactly at a segment, stop, or capacity event.
    for eventIteration=1:12
        [step, vNext] = solveStep(p,v,x,dt,pulse,segment,starts,caps,grades,radii,planBrake);
        dx=0.5*(v+vNext)*dt;
        remaining=ends(segment)-x;
        capacityJ=max(0,p.battery.usable_capacity_Wh*3600-energy(1));
        ratio=min(1,capacityJ/max(step.pt.battery_power_W*dt,eps));
        newdt=dt;
        if dx>remaining+1e-9
            % Constant-acceleration event time, stable even when acceleration is tiny.
            vend=sqrt(max(0,v^2+2*step.acceleration*remaining));
            newdt=min(newdt,2*remaining/max(v+vend,eps));
        end
        if vNext<0, newdt=min(newdt,-v/step.acceleration); end
        newdt=min(newdt,dt*ratio);
        if abs(newdt-dt)<1e-9, break; end
        dt=max(newdt,1e-10);
    end
    if ~step.pt.electrical_feasible
        electricalOK=false; reason='auxiliary_power_infeasible'; break;
    end
    if dt<1e-8, reason='battery_capacity'; break; end
    % Re-evaluate the final shortened interval so force and energy agree.
    [step,vNext]=solveStep(p,v,x,dt,pulse,segment,starts,caps,grades,radii,planBrake);
    if vNext < -1e-7, error('lapsim:Integration','Stop event failed to converge; reduce dt_s.'); end
    vNext=max(0,vNext); dx=0.5*(v+vNext)*dt;
    if dx>ends(segment)-x+1e-6
        error('lapsim:Integration','Segment event failed to converge; reduce dt_s.');
    end
    midSpeed=0.5*(v+vNext); pt=step.pt;
    % Each term is work over precisely the same interval. Signed aero/grade.
    increments=[pt.battery_power_W,pt.terminal_power_W,pt.battery_loss_W, ...
        pt.auxiliary_power_W,pt.motor_loss_W,pt.controller_loss_W,pt.drivetrain_loss_W, ...
        step.loads.aero_N*midSpeed,step.loads.rolling_N*midSpeed,step.loads.grade_N*midSpeed, ...
        step.brake*midSpeed,step.coastDrag*midSpeed]*dt;
    energy=energy+increments;
    x=x+dx; t=t+dt; v=vNext;
    violation=max(violation,max(0,v-physicalCaps(segment)));
    if segment<numel(ends) && abs(x-ends(segment))<1e-6
        violation=max(violation,max(0,v-physicalCaps(segment+1)));
    end
    rear=step.loads.rear_base_N+step.loads.transfer_per_force*(pt.wheel_force_N-step.brake-step.coastDrag);
    axleOK=axleOK && rear>=-1e-6 && rear<=step.loads.normal_N+1e-6;
    rollOK=rollOK && step.loads.roll_moment_Nm <= (step.loads.normal_N-rear)*p.vehicle.front_track_m/2+1e-6;
    count=count+1;
    data(count,:)=[t,x,v,dt,midSpeed,step.acceleration,mod(segment-1,height(track))+1, ...
        ceil(segment/height(track)),physicalCaps(segment),pt.wheel_force_N,step.brake, ...
        step.loads.aero_N,step.loads.rolling_N,step.loads.grade_N,rear,pt.motor_current_A, ...
        pt.battery_current_A,pt.battery_voltage_V,pt.duty_cycle,pt.motor_rpm,pt.terminal_power_W, ...
        pt.battery_power_W,energy(2)/3600,energy(1)/3600,step.mode];
    if energy(1)>=p.battery.usable_capacity_Wh*3600-1e-5, reason='battery_capacity'; break; end
    if v<1e-8 && dx<1e-8, reason='stalled'; break; end
end
completed=x>=totalDistance-1e-6;
if completed, reason='completed'; end
result.parameters=p; result.track=track;
result.trace=array2table(data(1:count,:),'VariableNames',names);
energyNames={'battery_Wh','terminal_Wh','battery_loss_Wh','auxiliary_Wh','motor_loss_Wh', ...
    'controller_loss_Wh','drivetrain_loss_Wh','aero_Wh','rolling_Wh','grade_Wh','braking_Wh','coast_drag_Wh'};
for k=1:numel(energyNames), result.energy.(energyNames{k})=energy(k)/3600; end
result.energy.kinetic_change_Wh=(0.5*meq*v^2-initialKE)/3600;
result.energy.balance_residual_Wh=result.energy.battery_Wh-sum(energy(3:12))/3600-result.energy.kinetic_change_Wh;
s.completed=completed; s.stop_reason=reason; s.distance_m=x; s.planned_distance_m=totalDistance;
s.lap_length_m=lapLength; s.time_s=t; s.average_speed_mps=x/max(t,eps);
s.terminal_energy_Wh=energy(2)/3600; s.battery_energy_Wh=energy(1)/3600;
s.Wh_per_km=NaN; s.km_per_kWh=NaN; s.mi_per_kWh=NaN;
if x>0 && energy(2)>0
    s.Wh_per_km=s.terminal_energy_Wh/(x/1000);
    s.km_per_kWh=1000/s.Wh_per_km; s.mi_per_kWh=s.km_per_kWh/1.609344;
end
s.within_time_limit=completed && t<=p.race.time_limit_s;
s.within_capacity=completed && s.battery_energy_Wh<=p.battery.usable_capacity_Wh+1e-8;
s.max_speed_limit_violation_mps=violation;
s.physically_feasible=electricalOK && axleOK && rollOK && violation<=p.sim.speed_tolerance_mps;
s.within_speed_requirement=completed && s.average_speed_mps>=p.race.minimum_average_speed_mps;
s.within_mass_requirement=mass>=p.race.minimum_total_mass_kg;
s.feasible=completed && s.within_time_limit && s.within_capacity && s.physically_feasible ...
    && s.within_speed_requirement && s.within_mass_requirement;
s.energy_budget_Wh=totalDistance/(p.race.target_mi_per_kWh*1.609344);
s.energy_margin_Wh=s.energy_budget_Wh-s.terminal_energy_Wh;
s.target_met=s.feasible && s.terminal_energy_Wh<=s.energy_budget_Wh;
s.time_margin_s=p.race.time_limit_s-t;
s.required_average_speed_mps=max(totalDistance/p.race.time_limit_s,p.race.minimum_average_speed_mps);
s.equivalent_mass_kg=meq; s.total_mass_kg=mass;
result.summary=s;
end

function caps=cornerCaps(p,grades,radii,limits)
gNormal=p.environment.gravity_mps2*cos(atan(grades));
halfWidth=p.vehicle.front_track_m/2*(1-p.vehicle.cg_from_front_m/p.vehicle.wheelbase_m) ...
    -abs(p.vehicle.cg_lateral_offset_m);
if p.vehicle.cg_height_m==0, roll=Inf(size(grades));
else, roll=gNormal*halfWidth/p.vehicle.cg_height_m; end
lateral=p.tires.corner_utilization*min(p.tires.friction_coefficient*gNormal,roll);
caps=min(limits,sqrt(lateral.*radii));
end

function [step,vNext]=solveStep(p,v,x,dt,pulse,segment,starts,caps,grades,radii,planBrake)
% Implicit midpoint iteration gives work = force*distance and delta kinetic energy.
vNext=v;
initialLoads=lapsim.roadLoads(p,v,grades(segment),radii(segment));
coast=strcmp(p.strategy.mode,'pulse_coast') && ~pulse;
upcoming=find(starts>x & starts-x<=p.strategy.coast_lookahead_m & caps<caps(segment));
if ~isempty(upcoming) && initialLoads.total_N>0
    coast=coast || any(v^2-2*initialLoads.total_N/initialLoads.equivalent_mass_kg*(starts(upcoming)-x)>caps(upcoming).^2);
end
converged=false;
for iter=1:35
    vm=max(0,(v+vNext)/2);
    loads=lapsim.roadLoads(p,vm,grades(segment),radii(segment));
    endGuess=x+max(v,vm)*dt;
    ahead=find(starts>x & starts<=endGuess+max(300,p.strategy.coast_lookahead_m));
    target=caps(segment);
    if ~isempty(ahead)
        target=min(target,min(sqrt(caps(ahead).^2+2*planBrake*max(0,starts(ahead)-endGuess))));
    end
    desiredAcceleration=(target-v)/dt;
    requiredForce=loads.total_N+loads.equivalent_mass_kg*desiredAcceleration;
    brake=max(0,min(loads.max_brake_N,-requiredForce));
    request=max(0,requiredForce);
    % Taper drive to the engaged RPM ceiling before reaching the discontinuity.
    motorSpeedLimit=p.motor.max_rpm*2*pi/60*p.vehicle.wheel_radius_m/p.drivetrain.gear_ratio;
    request=min(request,max(0,loads.total_N+loads.equivalent_mass_kg*(motorSpeedLimit-v)/dt));
    if strcmp(p.strategy.mode,'pulse_coast')
        request=min(request,p.strategy.pulse_motor_torque_Nm*p.drivetrain.gear_ratio*p.drivetrain.efficiency/p.vehicle.wheel_radius_m);
    end
    if coast, request=0; end
    request=min(request,loads.max_drive_N);
    if brake>0, request=0; end
    pt=lapsim.powertrain(p,vm,request);
    coastDrag=0;
    if pt.wheel_force_N==0 && vm>0
        rearBrakeGrip=loads.longitudinal_friction*max(0,loads.rear_base_N-loads.transfer_per_force*brake) ...
            /(1+loads.longitudinal_friction*loads.transfer_per_force);
        coastDrag=min(p.drivetrain.coast_drag_N,rearBrakeGrip);
        brake=min(brake,max(0,loads.max_brake_N-coastDrag));
    end
    acceleration=(pt.wheel_force_N-brake-loads.total_N-coastDrag)/loads.equivalent_mass_kg;
    % At rest, static road resistance cannot propel the car backwards.
    if v==0 && acceleration<0, acceleration=0; end
    candidate=v+acceleration*dt;
    if abs(candidate-vNext)<1e-9, vNext=candidate; converged=true; break; end
    vNext=candidate;
end
if ~converged, error('lapsim:Integration','Midpoint solver did not converge; reduce dt_s.'); end
step.pt=pt; step.loads=loads; step.brake=brake; step.coastDrag=coastDrag;
step.acceleration=acceleration; step.mode=double(pt.wheel_force_N>0)-double(brake>0);
end
