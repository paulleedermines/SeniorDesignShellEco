function out = powertrain(p, speed_mps, requested_force_N)
%POWERTRAIN Enforce motoring limits and account for every electrical loss.
%   OUT = lapsim.powertrain(P, SPEED_MPS, REQUESTED_FORCE_N) returns the
%   requested positive wheel force or the largest physically available force.
%   Use Inf to query the available force. Inputs and parameters are scalar;
%   lapsim.validate validates parameter ranges before simulation.
%
%   The motor is an effective DC equivalent: torque constant equals back-EMF
%   constant in SI units, Vmotor = kt*omega + Rmotor*Imotor. Motor current is
%   winding/effective phase current, distinct from DC battery current. Torque
%   and shaft-power limits apply after Coulomb/viscous motor losses. Controller
%   efficiency is a power efficiency; its duty is Vmotor/Vbattery_loaded.
%
%   Battery current uses the stable, lower-current root of
%       Pterminal = (Voc - Rbattery*Ibattery)*Ibattery.
%   Battery power is chemical power Voc*Ibattery, including internal loss.
%   No regeneration is modeled. At zero force the motor freewheels without
%   electrical or mechanical load; accessories remain powered. motor_rpm is
%   the engaged, drive-equivalent speed even while freewheeling.
%
%   If auxiliaries alone exceed the battery envelope, OUT serves only the
%   feasible auxiliary power and sets electrical_feasible=false. The caller
%   must terminate the simulation rather than count that as a valid run.

radius = p.vehicle.wheel_radius_m;
ratio = p.drivetrain.gear_ratio;
etaDrive = p.drivetrain.efficiency;
etaControl = p.controller.efficiency;
kt = p.motor.torque_constant_Nm_A;
rm = p.motor.resistance_ohm;
voc = p.battery.open_circuit_voltage_V;
rb = p.battery.internal_resistance_ohm;
omega = speed_mps * ratio / radius;
emf = kt * omega;
lossTorque = p.motor.loss_torque_Nm + ...
    p.motor.viscous_loss_Nm_per_rad_s * omega;

% Restrict the battery to its stable branch and configured I/V/P envelope.
if rb > 0
    batteryCurrentCap = min([p.battery.max_current_A, voc / (2 * rb), ...
        max(0, (voc - p.battery.min_terminal_voltage_V) / rb)]);
    terminalCap = min(p.battery.max_terminal_power_W, ...
        (voc - rb * batteryCurrentCap) * batteryCurrentCap);
elseif voc >= p.battery.min_terminal_voltage_V
    terminalCap = min(p.battery.max_terminal_power_W, ...
        voc * p.battery.max_current_A);
else
    terminalCap = 0;
end
auxiliary = min(p.auxiliary.power_W, terminalCap);
feasible = p.auxiliary.power_W <= terminalCap && ...
    voc >= p.battery.min_terminal_voltage_V;
[batteryCurrent, batteryVoltage] = batteryPoint(voc, rb, auxiliary);

out = struct('wheel_force_N', 0, 'motor_current_A', 0, ...
    'battery_current_A', batteryCurrent, 'battery_voltage_V', batteryVoltage, ...
    'motor_voltage_V', 0, 'duty_cycle', 0, ...
    'motor_rpm', omega * 60 / (2 * pi), 'motor_torque_Nm', 0, ...
    'battery_power_W', voc * batteryCurrent, 'terminal_power_W', auxiliary, ...
    'wheel_power_W', 0, 'motor_loss_W', 0, 'controller_loss_W', 0, ...
    'drivetrain_loss_W', 0, 'battery_loss_W', rb * batteryCurrent^2, ...
    'auxiliary_power_W', auxiliary, 'electrical_feasible', feasible);

if requested_force_N <= 0 || ~feasible || ...
        out.motor_rpm > p.motor.max_rpm
    return
end

% Analytic current bounds enforce winding current, usable shaft torque,
% shaft power, and battery terminal power before checking loaded-bus duty.
currentCap = min(p.motor.max_current_A, ...
    (p.motor.max_torque_Nm + lossTorque) / kt);
if omega > 0
    currentCap = min(currentCap, ...
        (p.motor.max_power_W / omega + lossTorque) / kt);
end
motorPowerCap = max(0, (terminalCap - auxiliary) * etaControl);
if rm > 0
    denominator = emf + sqrt(emf^2 + 4 * rm * motorPowerCap);
    if denominator > 0
        powerCurrentCap = 2 * motorPowerCap / denominator;
    else
        powerCurrentCap = 0;
    end
elseif emf > 0
    powerCurrentCap = motorPowerCap / emf;
else
    powerCurrentCap = Inf;
end
currentCap = min(currentCap, powerCurrentCap);
requestedCurrent = (requested_force_N * radius / (ratio * etaDrive) + ...
    lossTorque) / kt;
current = min(requestedCurrent, currentCap);
noLoadCurrent = lossTorque / kt;
if current <= noLoadCurrent
    return
end

motorVoltage = emf + rm * current;
terminalPower = auxiliary + motorVoltage * current / etaControl;
[batteryCurrent, batteryVoltage] = batteryPoint(voc, rb, terminalPower);
if motorVoltage > p.controller.max_duty_cycle * batteryVoltage
    % The voltage residual increases monotonically with winding current on
    % the stable battery branch. A small fixed bisection also handles Rb=0.
    low = noLoadCurrent;
    high = current;
    lowVoltage = emf + rm * low;
    lowPower = auxiliary + lowVoltage * low / etaControl;
    [~, lowBus] = batteryPoint(voc, rb, lowPower);
    if lowVoltage >= p.controller.max_duty_cycle * lowBus
        return
    end
    for iteration = 1:32
        mid = (low + high) / 2;
        midVoltage = emf + rm * mid;
        midPower = auxiliary + midVoltage * mid / etaControl;
        [~, midBus] = batteryPoint(voc, rb, midPower);
        if midVoltage <= p.controller.max_duty_cycle * midBus
            low = mid;
        else
            high = mid;
        end
    end
    current = low;
    motorVoltage = emf + rm * current;
    terminalPower = auxiliary + motorVoltage * current / etaControl;
    [batteryCurrent, batteryVoltage] = batteryPoint(voc, rb, terminalPower);
end

shaftTorque = max(0, kt * current - lossTorque);
wheelForce = shaftTorque * ratio * etaDrive / radius;
shaftPower = shaftTorque * omega;
motorPower = motorVoltage * current;
wheelPower = wheelForce * speed_mps;

out.wheel_force_N = wheelForce;
out.motor_current_A = current;
out.battery_current_A = batteryCurrent;
out.battery_voltage_V = batteryVoltage;
out.motor_voltage_V = motorVoltage;
out.duty_cycle = motorVoltage / batteryVoltage;
out.motor_torque_Nm = shaftTorque;
out.battery_power_W = voc * batteryCurrent;
out.terminal_power_W = terminalPower;
out.wheel_power_W = wheelPower;
out.motor_loss_W = rm * current^2 + lossTorque * omega;
out.controller_loss_W = motorPower * (1 / etaControl - 1);
out.drivetrain_loss_W = shaftPower - wheelPower;
out.battery_loss_W = rb * batteryCurrent^2;
end

function [current, voltage] = batteryPoint(voc, resistance, terminalPower)
% Stable quadratic form avoids cancellation for small accessory loads.
if resistance == 0
    current = terminalPower / voc;
    voltage = voc;
else
    discriminant = max(0, voc^2 - 4 * resistance * terminalPower);
    current = 2 * terminalPower / (voc + sqrt(discriminant));
    voltage = voc - resistance * current;
end
end
