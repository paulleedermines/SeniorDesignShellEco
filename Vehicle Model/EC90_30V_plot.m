%% 3D Speed-Torque-Efficiency map with PWM + speed-dependent losses
%This is for 30V motor with 60V input
%Dominic Padula
%AI was used to assist the development of this model
clear;clc;clf;
close all
% Motor data (30 V column values other than supply)
V_max = 60;        % max supply voltage (V)
R  = 0.275;         % terminal resistance (ohm)
kt = 0.136*0.9;        % torque constant (Nm/A) adjusted for sinusoidal
ks=70.2;          %speed constant rpm/V
ke = 1/(ks*2*3.14/60);        % back-EMF constant (V/(rad/s))
I0 = 0.490/2;        % no-load current (A)
w0_rpm = 2080*2;     % no-load speed (rpm)
T_stall=10; %Stall torque Nm

% Derived no-load values (for loss calibration)
omega0 = 2*pi*w0_rpm/60;        % rad/s
P_elec_noload = V_max * I0;     % W
P_cu_noload   = I0^2 * R;       % copper loss at no-load
P_other_noload = P_elec_noload - P_cu_noload;  % remainder (iron+windage+friction)

% Split mechanical loss into coulomb-like (torque*omega) and quadratic (windage)
frac_quad = 0.20;   % fraction of mechanical loss that is quadratic (tweakable)
P_quad = P_other_noload * frac_quad;
P_coulomb = P_other_noload * (1 - frac_quad);

% Coulomb-like torque (constant torque loss)
T_fric_coulomb = P_coulomb / omega0;   % Nm

% Viscous/windage torque coefficient (gives torque = B*omega, so power = B*omega^2)
B_visc = P_quad / (omega0^2);         % Nms/rad (units such that power = B*omega^2)

% Display calibrated loss pieces
fprintf('Calibrated loss params:\n');
fprintf('  Coulomb friction torque Tf0 = %.5f N*m\n', T_fric_coulomb);
fprintf('  Viscous coefficient B = %.6e (N*m / (rad/s))\n', B_visc);
fprintf('  (these reproduce no-load input power roughly)\n\n');

%% Sweep definitions
V_vec = linspace(5, V_max, 25);       % sweep supply voltage (simulate PWM) (V)
T_load = linspace(0, T_stall, 300);      % load torque sweep (Nm) up to stall (Nm)
[Tv, Vv] = meshgrid(T_load, V_vec);   % 2D grid of (T, V)

% Preallocate
omega_grid = zeros(size(Tv));    % rad/s
speed_rpm_grid = zeros(size(Tv));
I_grid = zeros(size(Tv));
Pmech_grid = zeros(size(Tv));
Pelec_grid = zeros(size(Tv));
eff_grid = zeros(size(Tv));

% For each (T_load, V) solve for steady-state omega from:
%   Electromagnetic torque T_e = T_load + Tf0 + B*omega
%   Current I = T_e / kt
%   Voltage: V = I*R + ke*omega
% -> combine: V = R/kt*(T_load + Tf0 + B*omega) + ke*omega
% -> omega * ( ke + R*B/kt ) = V - R/kt*(T_load + Tf0)
% -> omega = ( V - R/kt*(T_load + Tf0) ) / ( ke + R*B/kt )

den = ke + (R * B_visc / kt);
for i = 1:numel(Tv)
    T = Tv(i);
    V = Vv(i);

    rhs = V - (R/kt) * (T + T_fric_coulomb);
    omega = rhs / den;

    if omega < 0
        omega = 0;
    end

    % electromagnetic torque (includes friction & viscous)
    T_e = T + T_fric_coulomb + B_visc * omega;

    % current
    I = T_e / kt;

    % electrical/mech power
    Pelec = V * I;
    Pmech = T * omega;

    % safety: avoid tiny or negative Pelec
    if Pelec <= 1e-9
        eff = 0;
    else
        eff = max(0, Pmech / Pelec);
    end

    omega_grid(i) = omega;
    speed_rpm_grid(i) = omega * 60 / (2*pi);
    I_grid(i) = I;
    Pmech_grid(i) = Pmech;
    Pelec_grid(i) = Pelec;
    eff_grid(i) = eff;
end
%% New Code
%using out=G_A_Calcs(GR,t,smin,smax) %t in sec, s in mph
GR=9.8;
t_init_low=15;
t_init_high=9;
t_hyper=3;
init_accel_low=G_A_Calcs(GR/0.7,t_init_low,0,15);
init_accel_high=G_A_Calcs(GR,t_init_high,15,19.5);
hyper_accel=G_A_Calcs(GR,t_hyper,19.5,21.5);


% Find total power usage for the run using a resistive force of 8  N at 20.5
% mph (avg cruise speed)

%v=v0+at -> t=(v-v0)/a = (v-v0)/(F/m)
t_coast=(21.5-19.5)/(12/85);
t_init=t_init_low+t_init_high;
t_hypercycle=t_hyper+t_coast;

n_hyper_cycles=((60*30)-t_init)/t_hypercycle;

E_init_low=init_accel_low.E;
E_init_high=init_accel_high.E;
E_hyper_cycle=hyper_accel.E;

E_used=E_init_low+E_init_high+(n_hyper_cycles*E_hyper_cycle);
Eff_mikWh=10/E_used

%% Chats Fn
function out = motor_efficiency(T_load, speed_rpm, V_input)
% MOTOR_EFFICIENCY
% Computes motor efficiency for a PMDC motor with:
%  - back-EMF
%  - copper loss
%  - Coulomb friction
%  - viscous/windage loss (quadratic)
% Model parameters are based on your V datasheet column.

%% Motor parameters (30 V column,modified)
R  = 0.275;       % terminal resistance (ohm)
kt = 0.136*0.9;      % torque constant (Nm/A)
ks=70.2;
ke = 1/(ks*2*3.14/60);      % back-EMF constant (V/(rad/s))
I0 = 0.490/2;      % no-load current (A)
w0_rpm = 2080*2;

%% Derived no-load quantities
omega0 = 2*pi*w0_rpm/60;
P_elec_nl = 60 * I0;
P_cu_nl   = I0^2 * R;
P_other   = P_elec_nl - P_cu_nl;

% Split mechanical losses (these match the 3D map model)
frac_quad = 0.20;
P_quad = P_other * frac_quad;
P_coulomb = P_other * (1 - frac_quad);

T_fric = P_coulomb / omega0;    % coulomb friction torque
B_visc = P_quad / (omega0^2);   % viscous coefficient

%% Convert speed to rad/s
omega = speed_rpm * 2*pi/60;

%% Compute electromagnetic torque required
% Total load torque = useful load + friction + windage
T_e = T_load + T_fric + B_visc * omega;

%% Current draw
I = T_e / kt;

%% Back-EMF
E = ke * omega;

%% Electrical power
Pelec = V_input * I;

%% Mechanical output power
Pmech = T_load * omega;

%% Efficiency
if Pelec <= 1e-9
    eff = 0;
else
    eff = max(0, Pmech / Pelec);
end

%% Package outputs
out.efficiency     = eff;
out.eff_percent    = eff * 100;
out.current        = I;
out.P_mech         = Pmech;
out.P_elec         = Pelec;
out.T_electromag   = T_e;
out.T_load         = T_load;
out.T_coulomb      = T_fric;
out.T_viscous      = B_visc * omega;
out.back_EMF       = E;
out.omega          = omega;
out.speed_rpm      = speed_rpm;
out.V_input        = V_input;
end

%% Gearing and Accel Fn
function out=G_A_Calcs(GR,t,smin,smax) %t in sec, s in mph
%take a gear ratio and a time (seconds) to do the acceleration 
%Calculate power, energy used and efficiency
%Assume const accel
smin=smin/2.237; %m/s
smax=smax/2.237; %m/s
a=(smax-smin)/t;

M=85; %kg
r_t = .2413;                % Tire radius [m]

%P_req=TW
%F_req=ma
F_req=(M*a)+12;
T_wheel=F_req*r_t;
T_motor=T_wheel/GR;

if t<10
    n=t*10; %points to calc
else
    n=200;
end
Speed_sweep=linspace(smin,smax,n);
W_wheel_sweep=Speed_sweep*(1/r_t); %rad/s
W_motor_sweep=W_wheel_sweep*GR; %rad/s

rpm_motor_max=W_motor_sweep(n)*9.5493;
rpm_motor_sweep=W_motor_sweep*9.5493;

dt=t/n;
Pmechtotal=0;
Pelectotal=0;
for i=1:n
    outputstuff=motor_efficiency(T_motor,rpm_motor_sweep(i),60);
    Pmechtotal=Pmechtotal+outputstuff.P_mech;
    Pelectotal=Pelectotal+outputstuff.P_elec;
end
if rpm_motor_max>4160
    %print("Too fast")
end
out.E=(Pelectotal*dt/1000)/3600; %Energy used, kWh
out.EffTotal=Pmechtotal/Pelectotal;
out.F_req=F_req;
out.T_motor=T_motor;
out.rpm_motor_sweep=rpm_motor_sweep;
end
%% End of new code

%% Plot the 3D surface for eff
f1=figure('Position',[100 100 900 600]);
%surf(Tv, speed_rpm_grid, eff_grid*100, 'EdgeColor','none');
contourf(Tv, speed_rpm_grid, eff_grid*100);
xlabel('Torque (Nm)');
ylabel('Speed (rpm)');
zlabel('Efficiency (%)');
title('Efficiency Map');
colormap turbo; colorbar;
%view(30, -45);
grid on;
hold on
figure(f1)
plot(init_accel_low.T_motor,init_accel_low.rpm_motor_sweep,'.','color',[0 1 1],'MarkerSize', 7)
plot(init_accel_high.T_motor,init_accel_high.rpm_motor_sweep,'.','color',[1 1 0.2],'MarkerSize', 7)
plot(hyper_accel.T_motor,hyper_accel.rpm_motor_sweep,'.','color',[1 0.1 1],'MarkerSize', 7)
hold off

%% Plot the 3D surface for power
f2=figure('Position',[100 100 900 600]);
%surf(Tv, speed_rpm_grid, eff_grid*100, 'EdgeColor','none');
contourf(Tv, speed_rpm_grid, Pmech_grid);
xlabel('Torque (Nm)');
ylabel('Speed (rpm)');
zlabel('Mech Power ');
title('Power');
colormap turbo; colorbar;
%view(30, -45);
grid on;
hold on
figure(f2)
plot(init_accel_low.T_motor,init_accel_low.rpm_motor_sweep,'.','color',[0 1 1],'MarkerSize', 7)
plot(init_accel_high.T_motor,init_accel_high.rpm_motor_sweep,'.','color',[1 1 0.2],'MarkerSize', 7)
plot(hyper_accel.T_motor,hyper_accel.rpm_motor_sweep,'.','color',[1 0.1 1],'MarkerSize', 7)
hold off
%% Find global maximum efficiency and where it occurs
[eta_max, idx] = max(eff_grid(:));
[i_row, i_col] = ind2sub(size(eff_grid), idx);
T_at_max = Tv(idx);
V_at_max = Vv(idx);
speed_at_max = speed_rpm_grid(idx);

fprintf('\nGlobal max efficiency = %.2f %%\n', eta_max*100);
fprintf('  Occurs at: Torque = %.3f N*m, Speed = %.1f rpm, Supply V = %.1f V\n', ...
    T_at_max, speed_at_max, V_at_max);
