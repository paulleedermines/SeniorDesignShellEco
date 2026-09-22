%% 3D Speed-Torque-Efficiency map with PWM + speed-dependent losses
%Dominic Padula
%AI was used to assist the development of this model
clear;clc;clf;
% Motor data (60 V column baseline)
V_max = 60;        % max supply voltage (V)
R  = 1.26;         % terminal resistance (ohm)
kt = 0.286;        % torque constant (Nm/A)
ke = 0.286;        % back-EMF constant (V/(rad/s))
I0 = 0.227;        % no-load current (A)
w0_rpm = 1980;     % no-load speed (rpm)

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
T_load = linspace(0, 9.57, 300);      % load torque sweep (Nm) up to stall (~9.57 Nm)
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

%% Plot the 3D surface
figure('Position',[100 100 900 600]);
%surf(Tv, speed_rpm_grid, eff_grid*100, 'EdgeColor','none');
contourf(Tv, speed_rpm_grid, eff_grid*100);
xlabel('Torque (Nm)');
ylabel('Speed (rpm)');
zlabel('Efficiency (%)');
%title('3D Speed–Torque–Efficiency Map (PWM + viscous losses)');
colormap turbo; colorbar;
view(30, -45);
grid on;

%% Find global maximum efficiency and where it occurs
[eta_max, idx] = max(eff_grid(:));
[i_row, i_col] = ind2sub(size(eff_grid), idx);
T_at_max = Tv(idx);
V_at_max = Vv(idx);
speed_at_max = speed_rpm_grid(idx);

fprintf('\nGlobal max efficiency = %.2f %%\n', eta_max*100);
fprintf('  Occurs at: Torque = %.3f N*m, Speed = %.1f rpm, Supply V = %.1f V\n', ...
    T_at_max, speed_at_max, V_at_max);
