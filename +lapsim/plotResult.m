function fig = plotResult(r, visible)
%PLOTRESULT Race trace and delivered-energy breakdown, suitable for export.
if nargin<2, visible='on'; end
fig=figure('Name','Shell Eco-marathon LapSim','Color','w','Visible',visible, ...
    'Position',[100 100 1200 760]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
tr=r.trace; s=r.summary;
nexttile;
plot(tr.distance_m/1000,tr.speed_mps/0.44704,'LineWidth',1.3); hold on;
yline(r.parameters.race.minimum_average_speed_mps/0.44704,'--','Minimum average');
xlabel('Distance (km)'); ylabel('Speed (mph)'); grid on;
title(sprintf('%.2f min | %.1f mph average',s.time_s/60,s.average_speed_mps/0.44704));
nexttile;
plot(tr.time_s/60,tr.terminal_energy_Wh,'LineWidth',1.5); hold on;
yline(s.energy_budget_Wh,'--',sprintf('%.0f mi/kWh budget',r.parameters.race.target_mi_per_kWh));
xlabel('Time (min)'); ylabel('Delivered battery energy (Wh)'); grid on;
title(sprintf('%.1f mi/kWh | %.2f Wh',s.mi_per_kWh,s.terminal_energy_Wh));
nexttile;
plot(tr.time_s/60,tr.terminal_power_W,'LineWidth',0.9);
xlabel('Time (min)'); ylabel('Battery terminal power (W)'); grid on;
title('Motor/controller + auxiliaries, including startup');
nexttile;
values=[r.energy.rolling_Wh,r.energy.aero_Wh,r.energy.motor_loss_Wh, ...
    r.energy.drivetrain_loss_Wh,r.energy.controller_loss_Wh,r.energy.auxiliary_Wh, ...
    r.energy.braking_Wh,r.energy.coast_drag_Wh,r.energy.grade_Wh,r.energy.kinetic_change_Wh];
barh(values); yticks(1:numel(values));
yticklabels({'Rolling','Aero','Motor','Transmission','Controller','Auxiliary', ...
    'Braking','Coast drag','Grade','Kinetic change'});
xlabel('Energy (Wh), signed for aero/grade/kinetic'); grid on;
title('Delivered energy allocation');
sgtitle(sprintf('Shell Eco-marathon | %.2f kg | feasible: %d | target met: %d', ...
    s.total_mass_kg,s.feasible,s.target_met));
end
