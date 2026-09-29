function p = exampleTargetParameters()
%EXAMPLETARGETPARAMETERS Illustrative combined design candidate, not measured.
% Compare this candidate against the default baseline on the same mission.
% These values are goals to investigate, not guaranteed hardware capabilities.
p=lapsim.defaultParameters();
p.vehicle.chassis_mass_kg=35;
p.vehicle.driver_mass_kg=50;
p.aero.cd=0.10;
p.aero.frontal_area_m2=0.60;
p.tires.crr=0.003;
p.vehicle.cg_height_m=0.30;
p.vehicle.cg_from_front_m=0.50;
p.auxiliary.power_W=3;
end
