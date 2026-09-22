% SHELL ECOMARATHON Vehicle Model
% By Jordan Jeong and co.

%GOAL: AS FUEL EFFICIENT AS POSSIBLE FOR 10 MILES IN 30 MINUTES, about 16.09km

% Constants

clear; clc; format short;

rho = 1.004; % [kg/m^3] Air density in Colorado
rad = (2*pi/60); % Conversion factor for RMP to rad/sec
g = 9.81; % [m/s^s] gravity
V = 60; % [V] Voltage of car
I_max = 35; % [A] Current of car
C_d = .2 ; % Drag Coeff
C_rr = .004; % Rolling Coeff.
Wd = [61,61,65] ./ 2.205; % [kg] Weight Distribution [FL,FR,R]
m = sum(Wd); % [kg] total weight of car
h_aero = .375 ; % [m] height of aero 
h_cg = h_aero; % assume CoM height is same as aero height
% no grade, no wind, 3 wheels

% Tire jawn

r_w = .203; % [m] radius of rim
r_t = .2413; % [m] outer radius of tire
m_t = .35; % [kg] tire mass (of one)
m_w = .65; % [kg] wheel mass (of one)
m_as = m_t + m_w; % [kg] wheel assembly mass
l_wb = 1.397; % [m] distance between two axles
I_w = .5 * m_w * r_w^2; % [kg*m^2] MOI of Wheel
I_t = .5 * m_t * (r_t^2 + r_w^2); % [kg/m^2] MOI of tire
I_tot = I_t + I_w; % [kg/m^2] TOTAL MOI
mu_t = .7; % coeff. friction of tire

g_r = 12; % gear ratio
kv = 33.4; % [rpm/V] @ 60 Volts
T_eff = .95; % transmission efficiency RANDOM VALUE
A_f = .5; %[m^2] frontal area
P_max = V * I_max; % [W] 
P_maxwh = T_eff * P_max; % P_max wheels

RWD = Wd(3) / m; % [Weight fraction on rear axle]
F_maxf = (mu_t*RWD*m*g*1) / (1 - ((mu_t*h_cg)/l_wb)); % [N] Rear axle tractive force

m_eq = m + 3 * (I_w * (1/r_w)^2); % [kg] mass equivalent

t = 0:1:1800;

%Pre allocating arrays
F_drag = zeros(1, length(t));
P_drag = zeros(1, length(t));
F_rr = zeros(1, length(t));
P_rr = zeros(1, length(t));
F_inertia = zeros(1, length(t));
P_inertia = zeros(1, length(t));
P_maxf = zeros(1, length(t));
F_maxwh = zeros(1, length(t));
F_req = zeros(1,length(t));
P_x = zeros(1, length(t));
v = zeros(1, length(t));
a = zeros(1, length(t));
x = zeros(1, length(t));

for i = 1:length(t) - 1
    F_drag(i) = 1/2* rho * C_d * A_f * v(i)^2; % aerodynamic force (N)
    P_drag(i) = F_drag(i) * v(i); % aero power (W)
    % Rolling Resist. force and power
    F_rr(i) = C_rr * m *g; 
    P_rr(i) = F_rr(i) * v(i);
    % No grade
    F_maxwh(i) = P_maxwh / v(i); % F_max wheels

    % Determine which is your limiter (Powertrain [wh] or Traction [f])
    if F_maxwh(i) < F_maxf
        F = F_maxwh(i); % Use Max Powertrain force
    else
        F = F_maxf; % Use maximum tractive force if F_maxw is greater
    end

    if v(i) < 9.16 % 9.16 m/s = 20.5 mph
        a(i) = (F - F_rr(i) - F_drag(i)) / m_eq; % Calculate acceleration
        v(i+1) = v(i) + a(i); % Update velocity
        x(i+1) = x(i) + v(i) * (t(i+1) - t(i)); % Update position
    else
        a(i) = 0; % No acceleration if speed exceeds limit
        v(i+1) = v(i); % Maintain current velocity
        x(i+1) = x(i) + v(i) * (t(i+1) - t(i)); % Update position
    end
    F_inertia(i) = m_eq * a(i); % inertial force (N)
    P_inertia(i) = F_inertia(i) * v(i); % inertial power (W)

    F_req(i)=F_inertia(i)+F_rr(i)+F_drag(i);
    P_x(i)=F_req(i)*v(i);
end

%Energy Used
Energy_kWh = trapz(t, max(P_x, 0)) / (3600 * 1000);

figure('Name', 'Displacement vs Time'); 
set(gca,'FontSize',16); 
set(0, 'DefaultLineLineWidth', 1.5);
hold on; grid on; box on; 
plot(t, x ,'DisplayName', 'Displacement');
xlabel('Time [s]'); 
ylabel('Displacement  [m]'); 
legend
eff = x(end) / Energy_kWh / 1.609 / 1000; % efficiency in mi/kWh [from m/kWh]

% Power Loss Vs Time
figure('Name', 'PowerLoss'); 
set(gca,'FontSize',16); 
set(0, 'DefaultLineLineWidth', 1.5);
grid on; box on; 
plot(t, P_drag , t, P_rr, t, P_inertia );
xlabel('Time [s]'); 
ylabel('Power Loss [W]');
title('Power Loss [W] vs Time [s]')
legend('Drag','Rolling Resistance', 'Interia')

% power required
figure('Name', 'PowerRequired'); 
set(gca,'FontSize',16); 
set(0, 'DefaultLineLineWidth', 1.5);
hold on; grid on; box on; 
plot(t, P_x);
xlabel('Time [s]')
ylabel('Power [W]')
average(P_x)

% SENSITIVITY ANALYSIS
%{
param = 0;
param(1) = m; % mass
param(2) = h_aero; % height of aero pressure
param(3) = RWD; % RWD percent mass dist.
param(4) = V; % voltage
param(5) = mu_t; % coeff. of friction tire
param(6) = C_rr; % rolling resist. coeff.
param(7) = m_w; % mass wheel assem.
param(8) = m_t; % mass tire assem.
param(9) = r_w; % radius wheel
param(10) = r_t; % radius tire
param(11) = C_d; % coeff. of aero
%}










