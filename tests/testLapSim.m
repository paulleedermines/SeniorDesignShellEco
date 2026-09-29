function tests = testLapSim
%TESTLAPSIM Physics and mission regressions, independent of a fitted car.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
repository=fileparts(fileparts(mfilename('fullpath')));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture(repository));
end

function testConstantCruiseMatchesClosedForm(testCase)
[p,track] = cruiseCase(100,5);
r = lapsim.simulate(p,track);
v=p.sim.initial_speed_mps; m=p.vehicle.chassis_mass_kg+p.vehicle.driver_mass_kg;
force=0.5*p.environment.air_density_kgpm3*p.aero.cd*p.aero.frontal_area_m2*v^2 ...
    +p.tires.crr*m*p.environment.gravity_mps2;
omega=v*p.drivetrain.gear_ratio/p.vehicle.wheel_radius_m;
torque=force*p.vehicle.wheel_radius_m/(p.drivetrain.gear_ratio*p.drivetrain.efficiency);
current=(torque+p.motor.loss_torque_Nm+p.motor.viscous_loss_Nm_per_rad_s*omega) ...
    /p.motor.torque_constant_Nm_A;
motorVoltage=p.motor.torque_constant_Nm_A*omega+p.motor.resistance_ohm*current;
terminalPower=p.auxiliary.power_W+motorVoltage*current/p.controller.efficiency;
voc=p.battery.open_circuit_voltage_V; resistance=p.battery.internal_resistance_ohm;
batteryCurrent=2*terminalPower/(voc+sqrt(voc^2-4*resistance*terminalPower));
duration=100/v;
verifyTrue(testCase,r.summary.feasible);
verifyEqual(testCase,r.summary.time_s,duration,'AbsTol',1e-8);
verifyEqual(testCase,r.trace.speed_mps,repmat(v,height(r.trace),1),'AbsTol',1e-8);
verifyEqual(testCase,r.energy.terminal_Wh,terminalPower*duration/3600,'RelTol',1e-8);
verifyEqual(testCase,r.energy.battery_Wh,voc*batteryCurrent*duration/3600,'RelTol',1e-8);
verifyEqual(testCase,r.energy.auxiliary_Wh,p.auxiliary.power_W*duration/3600,'AbsTol',1e-10);
verifyEqual(testCase,r.energy.aero_Wh, ...
    0.5*p.environment.air_density_kgpm3*p.aero.cd*p.aero.frontal_area_m2*v^2*100/3600,'RelTol',1e-8);
verifyEqual(testCase,r.summary.energy_budget_Wh,100/(300*1.609344),'AbsTol',1e-12);
end

function testIdealStartupEnergyEqualsKineticEnergy(testCase)
[p,track]=cruiseCase(20,20);
p.sim.initial_speed_mps=0;
p.aero.cd=0; p.tires.crr=0; p.auxiliary.power_W=0;
p.drivetrain.efficiency=1; p.controller.efficiency=1;
p.battery.internal_resistance_ohm=0;
p.motor.loss_torque_Nm=0; p.motor.viscous_loss_Nm_per_rad_s=0;
% Validation requires positive resistance. This tends to the ideal motor.
p.motor.resistance_ohm=1e-12;
r=lapsim.simulate(p,track);
force=p.motor.max_torque_Nm*p.drivetrain.gear_ratio/p.vehicle.wheel_radius_m;
expectedSpeed=sqrt(2*force*20/r.summary.equivalent_mass_kg);
verifyTrue(testCase,r.summary.completed);
verifyEqual(testCase,r.trace.speed_mps(end),expectedSpeed,'AbsTol',1e-7);
verifyEqual(testCase,r.energy.battery_Wh,r.energy.kinetic_change_Wh,'AbsTol',1e-9);
verifyLessThan(testCase,abs(r.energy.balance_residual_Wh),1e-9);
end

function testStartupCopperAndFreewheel(testCase)
p=lapsim.defaultParameters();
start=lapsim.powertrain(p,0,Inf);
verifyGreaterThan(testCase,start.wheel_force_N,0);
verifyEqual(testCase,start.wheel_power_W,0);
verifyGreaterThan(testCase,start.motor_loss_W,0);
verifyGreaterThan(testCase,start.battery_power_W,p.auxiliary.power_W);
verifyEqual(testCase,start.motor_loss_W,p.motor.resistance_ohm*start.motor_current_A^2,'AbsTol',1e-12);
coast=lapsim.powertrain(p,8,0);
verifyEqual(testCase,coast.motor_current_A,0);
verifyEqual(testCase,coast.wheel_force_N,0);
verifyEqual(testCase,coast.terminal_power_W,p.auxiliary.power_W);
verifyGreaterThan(testCase,coast.battery_power_W,coast.terminal_power_W);
end

function testPowertrainEnforcesLimitsAndConservesEnergy(testCase)
baseline=lapsim.defaultParameters();
for scenario=1:4
    p=baseline;
    if scenario==2
        p.battery.max_current_A=0.5; p.battery.max_terminal_power_W=15;
    elseif scenario==3
        p.battery.internal_resistance_ohm=1.5;
        p.battery.min_terminal_voltage_V=58; p.controller.max_duty_cycle=0.55;
    elseif scenario==4
        p.motor.max_current_A=2; p.motor.max_torque_Nm=0.1; p.motor.max_power_W=15;
        p.battery.internal_resistance_ohm=0;
    end
    for speed=[0 1 5 10 12 20]
        for request=[0 0.01 10 Inf]
            q=lapsim.powertrain(p,speed,request);
            verifyGreaterThanOrEqual(testCase,q.wheel_force_N,0);
            verifyLessThanOrEqual(testCase,q.wheel_force_N,request+1e-8);
            verifyLessThanOrEqual(testCase,q.motor_current_A,p.motor.max_current_A+1e-8);
            verifyLessThanOrEqual(testCase,q.motor_torque_Nm,p.motor.max_torque_Nm+1e-8);
            verifyLessThanOrEqual(testCase,q.motor_torque_Nm*q.motor_rpm*2*pi/60,p.motor.max_power_W+1e-8);
            verifyLessThanOrEqual(testCase,q.battery_current_A,p.battery.max_current_A+1e-8);
            verifyLessThanOrEqual(testCase,q.terminal_power_W,p.battery.max_terminal_power_W+1e-8);
            verifyGreaterThanOrEqual(testCase,q.battery_voltage_V,p.battery.min_terminal_voltage_V-1e-8);
            verifyLessThanOrEqual(testCase,q.duty_cycle,p.controller.max_duty_cycle+1e-8);
            if q.motor_rpm>p.motor.max_rpm, verifyEqual(testCase,q.wheel_force_N,0); end
            dissipated=q.motor_loss_W+q.controller_loss_W+q.drivetrain_loss_W ...
                +q.battery_loss_W+q.auxiliary_power_W;
            verifyEqual(testCase,q.battery_power_W,q.wheel_power_W+dissipated,'AbsTol',1e-8);
        end
    end
end
end

function testAuxiliaryInfeasibilityStopsRun(testCase)
[p,track]=cruiseCase(100,5);
p.auxiliary.power_W=1000;
q=lapsim.powertrain(p,5,Inf);
verifyFalse(testCase,q.electrical_feasible);
verifyEqual(testCase,q.wheel_force_N,0);
r=lapsim.simulate(p,track);
verifyEqual(testCase,r.summary.stop_reason,'auxiliary_power_infeasible');
verifyFalse(testCase,r.summary.completed);
verifyFalse(testCase,r.summary.target_met);
verifyEqual(testCase,r.summary.distance_m,0);
end

function testEveryLapAndSegmentEndsAtPhysicalDistance(testCase)
[p,~]=cruiseCase(100,5);
track=makeTrack([37;23;41]); p.race.laps=3;
r=lapsim.simulate(p,track);
ends=cumsum(repmat(track.length_m,p.race.laps,1));
verifyTrue(testCase,r.summary.completed);
verifyEqual(testCase,r.summary.distance_m,303,'AbsTol',1e-7);
verifyEqual(testCase,r.trace.distance_m(end),303,'AbsTol',1e-7);
verifyEqual(testCase,r.summary.lap_length_m,101);
verifyEqual(testCase,r.trace.lap(end),3);
for boundary=ends'
    verifyLessThan(testCase,min(abs(r.trace.distance_m-boundary)),1e-7);
end
verifyTrue(testCase,all(diff(r.trace.distance_m)>0));
verifyEqual(testCase,sum(r.trace.dt_s),r.summary.time_s,'AbsTol',1e-10);
end

function testMassDragAndRollingResistanceIncreaseCruiseEnergy(testCase)
[p,track]=cruiseCase(100,5);
base=lapsim.simulate(p,track);
heavy=p; heavy.vehicle.chassis_mass_kg=p.vehicle.chassis_mass_kg+30;
draggy=p; draggy.aero.cd=p.aero.cd*2;
rolling=p; rolling.tires.crr=p.tires.crr*1.5;
for variant={heavy,draggy,rolling}
    r=lapsim.simulate(variant{1},track);
    verifyTrue(testCase,r.summary.feasible);
    verifyEqual(testCase,r.summary.time_s,base.summary.time_s,'AbsTol',1e-8);
    verifyGreaterThan(testCase,r.energy.terminal_Wh,base.energy.terminal_Wh);
end
end

function testBatteryDepletionIsPartialAndCannotMeetTarget(testCase)
[p,track]=cruiseCase(100,5);
p.battery.usable_capacity_Wh=0.05;
r=lapsim.simulate(p,track);
verifyFalse(testCase,r.summary.completed);
verifyFalse(testCase,r.summary.target_met);
verifyEqual(testCase,r.summary.stop_reason,'battery_capacity');
verifyGreaterThan(testCase,r.summary.distance_m,0);
verifyLessThan(testCase,r.summary.distance_m,100);
verifyEqual(testCase,r.energy.battery_Wh,p.battery.usable_capacity_Wh,'AbsTol',1e-8);
verifyLessThan(testCase,abs(r.energy.balance_residual_Wh),1e-8);
end

function testTimeoutDoesNotInventDistance(testCase)
[p,track]=cruiseCase(100,5);
p.sim.max_time_s=1.2;
r=lapsim.simulate(p,track);
verifyFalse(testCase,r.summary.completed);
verifyFalse(testCase,r.summary.feasible);
verifyFalse(testCase,r.summary.target_met);
verifyEqual(testCase,r.summary.stop_reason,'max_time');
verifyEqual(testCase,r.summary.time_s,1.2,'AbsTol',1e-10);
verifyEqual(testCase,r.summary.distance_m,6,'AbsTol',1e-8);
verifyEqual(testCase,r.summary.planned_distance_m,100);
end

function testCornerBrakingIncludesNextLap(testCase)
[p,~]=cruiseCase(120,8);
p.sim.initial_speed_mps=3; p.race.laps=2;
p.strategy.coast_lookahead_m=0;
track=makeTrack([20;100]);
track.radius_m=[12;Inf]; track.speed_limit_mps=[3;8];
r=lapsim.simulate(p,track);
verifyTrue(testCase,r.summary.completed);
verifyTrue(testCase,r.summary.physically_feasible);
verifyGreaterThan(testCase,max(r.trace.brake_force_N),0);
[offset,index]=min(abs(r.trace.distance_m-120));
verifyLessThan(testCase,offset,1e-7);
verifyLessThanOrEqual(testCase,r.trace.speed_mps(index),3+p.sim.speed_tolerance_mps);
verifyLessThan(testCase,abs(r.energy.balance_residual_Wh),1e-7);
end

function testGripAndCenterOfMassAffectTractionAndCornerLimits(testCase)
[p,track]=cruiseCase(20,9);
normal=lapsim.roadLoads(p,0,0,Inf);
lowGrip=p; lowGrip.tires.friction_coefficient=0.03;
limited=lapsim.roadLoads(lowGrip,0,0,Inf);
rearward=lowGrip; rearward.vehicle.cg_from_front_m=0.75*p.vehicle.wheelbase_m;
rearLoaded=lapsim.roadLoads(rearward,0,0,Inf);
verifyLessThan(testCase,limited.max_drive_N,normal.max_drive_N);
verifyGreaterThan(testCase,rearLoaded.max_drive_N,limited.max_drive_N);
p.sim.initial_speed_mps=0; track.radius_m=10;
p.vehicle.cg_height_m=0.25;
lowCG=lapsim.simulate(p,track);
p.vehicle.cg_height_m=0.75;
highCG=lapsim.simulate(p,track);
verifyLessThan(testCase,highCG.trace.speed_limit_mps(1),lowCG.trace.speed_limit_mps(1));
verifyTrue(testCase,highCG.summary.physically_feasible);
verifyLessThan(testCase,abs(highCG.energy.balance_residual_Wh),1e-7);
end

function testRoadLoadsHaveExpectedSignsAndUnits(testCase)
p=lapsim.defaultParameters(); m=p.vehicle.chassis_mass_kg+p.vehicle.driver_mass_kg;
rest=lapsim.roadLoads(p,0,0,Inf);
moving=lapsim.roadLoads(p,5,0,Inf);
verifyEqual(testCase,rest.rolling_N,p.tires.crr*m*p.environment.gravity_mps2,'AbsTol',1e-10);
verifyEqual(testCase,moving.rolling_N,rest.rolling_N,'AbsTol',1e-10);
uphill=lapsim.roadLoads(p,5,0.1,Inf);
downhill=lapsim.roadLoads(p,5,-0.1,Inf);
verifyGreaterThan(testCase,uphill.grade_N,0);
verifyEqual(testCase,uphill.grade_N,-downhill.grade_N,'AbsTol',1e-10);
p.environment.headwind_mps=-10;
tailwind=lapsim.roadLoads(p,5,0,Inf);
verifyLessThan(testCase,tailwind.aero_N,0);
end

function testUphillDownhillEnergyAccountsForBraking(testCase)
[p,~]=cruiseCase(80,5);
p.motor.max_current_A=30; p.motor.max_torque_Nm=3;
p.motor.max_power_W=2000; p.battery.max_current_A=30;
p.battery.max_terminal_power_W=1800;
track=makeTrack([40;40]); track.grade=[0.05;-0.05];
r=lapsim.simulate(p,track);
verifyTrue(testCase,r.summary.feasible);
verifyEqual(testCase,r.energy.grade_Wh,0,'AbsTol',1e-8);
verifyGreaterThan(testCase,r.energy.braking_Wh,0);
verifyTrue(testCase,all(r.trace.battery_current_A>=0));
verifyLessThan(testCase,abs(r.energy.balance_residual_Wh),1e-7);
end

function testInvalidPhysicalInputsAreRejected(testCase)
[p,track]=cruiseCase(100,5);
bad=p; bad.aero.cd=-0.1;
verifyError(testCase,@()lapsim.simulate(bad,track),'lapsim:InvalidParameter');
bad=p; bad.drivetrain.efficiency=1.01;
verifyError(testCase,@()lapsim.simulate(bad,track),'lapsim:InvalidParameter');
bad=p; bad.vehicle.cg_from_front_m=p.vehicle.wheelbase_m;
verifyError(testCase,@()lapsim.simulate(bad,track),'lapsim:InvalidParameter');
bad=p; bad.vehicle.cg_lateral_offset_m=1;
verifyError(testCase,@()lapsim.simulate(bad,track),'lapsim:InvalidParameter');
bad=p; bad.motor.max_current_A=NaN;
verifyError(testCase,@()lapsim.simulate(bad,track),'lapsim:InvalidParameter');
bad=p; bad.aero.Cd=0.1;
verifyError(testCase,@()lapsim.simulate(bad,track),'lapsim:InvalidParameter');
badTrack=track; badTrack.length_m=0;
verifyError(testCase,@()lapsim.simulate(p,badTrack),'lapsim:InvalidTrack');
badTrack=track; badTrack.radius_m=-1;
verifyError(testCase,@()lapsim.simulate(p,badTrack),'lapsim:InvalidTrack');
end

function testMissionRequirementsGateEfficiencyTargets(testCase)
[p,track]=cruiseCase(100,5);
p.race.target_mi_per_kWh=1;
baseline=lapsim.simulate(p,track);
verifyTrue(testCase,baseline.summary.target_met);
p.race.minimum_average_speed_mps=6;
slow=lapsim.simulate(p,track);
verifyFalse(testCase,slow.summary.within_speed_requirement);
verifyFalse(testCase,slow.summary.target_met);
p.race.minimum_average_speed_mps=0; p.race.minimum_total_mass_kg=1000;
light=lapsim.simulate(p,track);
verifyFalse(testCase,light.summary.within_mass_requirement);
verifyFalse(testCase,light.summary.target_met);
end

function testSensitivityIsRepeatableAndIndependentOfOrder(testCase)
[p,track]=cruiseCase(100,5);
p.strategy.mode='pulse_coast'; p.strategy.pulse_min_speed_mps=4;
p.strategy.pulse_max_speed_mps=6; p.sim.initial_speed_mps=0;
first=lapsim.sensitivity(p,track,{'aero.cd','tires.crr'},[0.8 1 1.2]);
repeat=lapsim.sensitivity(p,track,{'aero.cd','tires.crr'},[0.8 1 1.2]);
reversed=lapsim.sensitivity(p,track,{'tires.crr','aero.cd'},[1.2 1 0.8]);
keys={'parameter_path','factor'};
verifyEqual(testCase,first,repeat);
verifyEqual(testCase,sortrows(first,keys),sortrows(reversed,keys));
verifyTrue(testCase,all(first.feasible));
baseline=first(first.factor==1,:);
verifyEqual(testCase,baseline.terminal_energy_Wh(1),baseline.terminal_energy_Wh(2),'AbsTol',1e-12);
end

function testTargetAndStrategyStudiesPreserveFeasibility(testCase)
[p,track]=cruiseCase(40,5);
targets=lapsim.targetSweep(p,track,'aero.cd',[0.05 -0.1 0.2]);
verifyEqual(testCase,targets.requested_value,[0.05;-0.1;0.2]);
verifyFalse(testCase,targets.valid(2));
verifyFalse(testCase,targets.target_met(2));
verifyGreaterThan(testCase,targets.terminal_energy_Wh(3),targets.terminal_energy_Wh(1));
strategies=lapsim.strategySweep(p,track,[4 5],[0 1]);
verifyTrue(testCase,all(strategies.valid));
verifyTrue(testCase,all(strategies.feasible));
verifyEqual(testCase,sum(strategies.is_best_feasible),1);
selected=strategies.terminal_energy_Wh(strategies.is_best_feasible);
verifyEqual(testCase,selected,min(strategies.terminal_energy_Wh(strategies.feasible)),'AbsTol',1e-12);
end

function testMotorSpeedCeilingIsConsistentAtTransition(testCase)
[p,track]=cruiseCase(100,9);
p.motor.max_rpm=3000;
limit=p.motor.max_rpm*2*pi/60*p.vehicle.wheel_radius_m/p.drivetrain.gear_ratio;
p.sim.initial_speed_mps=limit-0.01;
r=lapsim.simulate(p,track);
verifyTrue(testCase,r.summary.completed);
verifyLessThanOrEqual(testCase,max(r.trace.speed_mps),limit+1e-8);
expectedRPM=r.trace.interval_speed_mps*p.drivetrain.gear_ratio/p.vehicle.wheel_radius_m*60/(2*pi);
verifyEqual(testCase,r.trace.motor_rpm,expectedRPM,'AbsTol',1e-6);
verifyLessThan(testCase,abs(r.energy.balance_residual_Wh),1e-8);
end

function testDynamicRollMomentConstrainsRearDrive(testCase)
p=lapsim.defaultParameters(); p.vehicle.cg_height_m=0.6;
radius=20;
halfWidth=p.vehicle.front_track_m/2*(1-p.vehicle.cg_from_front_m/p.vehicle.wheelbase_m);
speed=sqrt(0.98*p.environment.gravity_mps2*halfWidth/p.vehicle.cg_height_m*radius);
loads=lapsim.roadLoads(p,speed,0,radius);
rear=loads.rear_base_N+loads.transfer_per_force*loads.max_drive_N;
availableMoment=(loads.normal_N-rear)*p.vehicle.front_track_m/2;
verifyLessThanOrEqual(testCase,loads.roll_moment_Nm,availableMoment+1e-8);
straight=lapsim.roadLoads(p,speed,0,Inf);
verifyLessThan(testCase,loads.max_drive_N,straight.max_drive_N);
end

function testFullRaceConvergesWithSmallerTimeStep(testCase)
p=lapsim.defaultParameters(); track=lapsim.exampleTrack();
coarse=lapsim.simulate(p,track);
p.sim.dt_s=p.sim.dt_s/2;
fine=lapsim.simulate(p,track);
verifyTrue(testCase,coarse.summary.feasible);
verifyTrue(testCase,fine.summary.feasible);
verifyEqual(testCase,coarse.summary.distance_m,15330,'AbsTol',1e-6);
verifyEqual(testCase,coarse.summary.terminal_energy_Wh,fine.summary.terminal_energy_Wh,'RelTol',0.01);
verifyEqual(testCase,coarse.summary.time_s,fine.summary.time_s,'RelTol',0.005);
verifyLessThan(testCase,abs(fine.energy.balance_residual_Wh),1e-7);
end

function testTinySegmentRemainderIsNotBatteryDepletion(testCase)
p=lapsim.defaultParameters(); track=lapsim.exampleTrack();
% These pulse/coast phases previously left ~1e-8 m at a segment boundary.
p.tires.crr=0.001;
r=lapsim.simulate(p,track);
verifyTrue(testCase,r.summary.completed);
verifyEqual(testCase,r.summary.stop_reason,'completed');
verifyLessThan(testCase,r.summary.battery_energy_Wh,50);
p=lapsim.defaultParameters(); p.vehicle.chassis_mass_kg=70; p.vehicle.driver_mass_kg=50;
r=lapsim.simulate(p,track);
verifyTrue(testCase,r.summary.completed);
verifyLessThan(testCase,r.summary.battery_energy_Wh,p.battery.usable_capacity_Wh);
end

function [p,track]=cruiseCase(distance,speed)
p=lapsim.defaultParameters();
p.strategy.mode='cruise'; p.strategy.cruise_speed_mps=speed;
p.sim.initial_speed_mps=speed; p.sim.dt_s=0.2; p.sim.max_time_s=120;
p.race.laps=1; p.race.minimum_average_speed_mps=0;
p.strategy.coast_lookahead_m=0;
track=makeTrack(distance);
end

function track=makeTrack(lengths)
lengths=lengths(:); count=numel(lengths);
track=table(lengths,zeros(count,1),Inf(count,1),Inf(count,1), ...
    'VariableNames',{'length_m','grade','radius_m','speed_limit_mps'});
end
