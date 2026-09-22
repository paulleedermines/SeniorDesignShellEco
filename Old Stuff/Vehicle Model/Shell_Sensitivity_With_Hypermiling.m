% SHELL MODEL W/ CHAT (HYPERMILING SENSITIVITY TEST)
% SHELL ECOMARATHON Vehicle Model (cleaned + robust)
% By Jordan Jeong and co. (edited)

% THIS CODE IS TO PERFORM A SENSITIVTY TEST OF THE SHELL ECO-MARATHON CAR.
% THE CAR IS HYPERMILING WITH V_AVG BEING 20.5 MPH, AT A 5 PERCENT
% INCREASES AND DECREASE. 
% NOTE: THIS IS NOT THE HYPERMILING SPEED TEST, THAT IS IN A DIFFERENT CODE


clear; clc; format short;

rho = 1.004;           % [kg/m^3] Air density in Colorado
rad = (2*pi/60);      % Conversion factor for RPM to rad/sec (not used currently)
g = 9.81;             % [m/s^2] gravity
V = 60;               % [V] Voltage of car
I_max = 10;           % [A] Current of car
C_d = 0.1;            % Drag Coeff
C_rr = 0.0056;         % Rolling Coeff.
Wd = [61,61,65] ./ 2.205; % [kg] Weight Distribution [FL,FR,R]
m = sum(Wd);          % [kg] total weight of car
h_aero = 0.375;       % [m] height of aero 
h_cg = h_aero;        % assume CoM height same as aero height
% no grade, no wind, 3 wheels

% Tire jawn
r_w = 0.203;          % [m] radius of rim
r_t = 0.2413;         % [m] outer radius of tire
m_t = 0.35;           % [kg] tire mass (one)
m_w = 0.65;           % [kg] wheel mass (one)
m_as = m_t + m_w;     % [kg] wheel assembly mass
l_wb = 1.397;         % [m] distance between two axles
I_w = 0.5 * m_w * r_w^2;                       % [kg*m^2] MOI wheel
I_t = 0.5 * m_t * (r_t^2 + r_w^2);             % [kg*m^2] MOI tire
I_tot = I_t + I_w;                             % [kg*m^2] TOTAL MOI
mu_t = 0.7;                                     % coeff. friction tire


T_eff = 0.90;          % transmission efficiency (estimate)
A_f = 0.71;             % [m^2] frontal area
P_max = V * I_max;     % [W] motor input
P_maxwh = T_eff * P_max; % [W] available at wheels (approx)

RWD = Wd(3) / m;       % [Weight fraction on rear axle]
% Traction-limited force (static formula); keep denominator safe
F_maxf = (mu_t*RWD*m*g) / (1 - ((mu_t*h_cg)/l_wb)); % [N] Rear axle tractive force

% Equivalent mass including rotating inertia
m_eq = m + 3 * (I_tot * (1/r_w)^2); % [kg] mass equivalent

% Time vector (seconds)
t = 0:1:1800; % 30 minutes at 1-second steps

% Pre-allocate arrays
N = length(t);
F_drag = zeros(1,N);
P_drag = zeros(1,N);
F_rr = zeros(1,N);
P_rr = zeros(1,N);
F_inertia = zeros(1,N);
P_inertia = zeros(1,N);
F_maxwh = zeros(1,N);
F_req = zeros(1,N);
P_x = zeros(1,N);
v = zeros(1,N);
a = zeros(1,N);
x = zeros(1,N);

% Initial condition: start from rest (explicit)
v(1) = 0;
x(1) = 0;
dt = 1;
v_max = 20.5 / 2.237 + 0.05 * (20.5 / 2.237);
v_min = 20.5 / 2.237 - 0.05 * (20.5 / 2.237);
isPulsing = true;
for i = 1:N-1
    % aerodynamic force (N)
    F_drag(i) = 0.5 * rho * C_d * A_f * v(i)^2;
    P_drag(i) = F_drag(i) * v(i);
    % Rolling resistance (N) and power
    F_rr(i) = C_rr * m * g;
    P_rr(i) = F_rr(i) * v(i);

    % Avoid division by zero for v == 0: at zero speed power cannot
    % be used directly to compute force (P/v), so treat as not power-limited.
    if v(i) <= 0
        F_maxwh(i) = Inf;
    else
        F_maxwh(i) = P_maxwh / v(i);
    end

    % Determine limiter: powertrain or traction
    if F_maxwh(i) < F_maxf
        F = F_maxwh(i);
    else
        F = F_maxf;
    end
    F = 131.9675;

     if isPulsing == true % 10.992 m/s = 24.6 mph, this scenario is to speed up to top end speed
        if v(i) >= v_max
            isPulsing = false;
        end
    else   % 7.33 m/s = 16.4 mph, low end speed,
        F = 0;
        if v(i) <= v_min % 7.786 m/s = 9.16 - 15 percent
            isPulsing = true;
        end
    end % end of pulsing if else

    % position update (explicit Euler)

            a(i) = (F  - F_rr(i) - F_drag(i)) / m_eq; % Calculate acceleration
            v(i+1) = v(i) + a(i); % Update velocity
            x(i+1) = x(i) + v(i) * (t(i+1) - t(i)); % Update position
            F_inertia(i) = m_eq * a(i); % inertial force (N)
            P_inertia(i) = F_inertia(i) * v(i); % inertial power (W)
        
            F_req(i)=F_inertia(i)+F_rr(i)+F_drag(i);
            P_x(i)=F_req(i)*v(i);
end


% Energy used (integrate positive power only)
Energy_kWh = trapz(t, max(P_x, 0)) / (3600 * 1000); % [kWh]

% Convert displacement to miles correctly and compute efficiency mi/kWh
m_to_mi = 1/1609.344;
distance_miles = x(end) * m_to_mi;
if Energy_kWh > 0
    eff_mi_per_kWh = distance_miles / Energy_kWh;
else
    eff_mi_per_kWh = NaN;
end

fprintf('Distance [m]: %.1f  Distance [mi]: %.4f  Energy [kWh]: %.4f  Eff [mi/kWh]: %.4f\n',...
    x(end), distance_miles, Energy_kWh, eff_mi_per_kWh)

% --- Plots ---
figure('Name', 'Displacement vs Time');
set(gca,'FontSize',16);
set(0, 'DefaultLineLineWidth', 1.5);
hold on; grid on; box on;
plot(t, x ,'DisplayName', 'Displacement [m]');
xlabel('Time [s]');
ylabel('Displacement  [m]');
legend

figure('Name', 'PowerLoss');
set(gca,'FontSize',16);
set(0, 'DefaultLineLineWidth', 1.5);
grid on; box on;
plot(t, P_drag, t, P_rr, t, P_inertia);
xlabel('Time [s]');
ylabel('Power Loss [W]');
title('Power Loss [W] vs Time [s]')
legend('Drag','Rolling Resistance','Inertia')

figure('Name', 'PowerRequired');
set(gca,'FontSize',16);
set(0, 'DefaultLineLineWidth', 1.5);
hold on; grid on; box on;
plot(t, P_x, t , P_maxwh, t ,F_maxf);
xlabel('Time [s]')
ylabel('Power [W]')

figure('Name', 'F Plot');
set(gca,'FontSize',16);
set(0, 'DefaultLineLineWidth', 1.5);
hold on; grid on; box on;
plot(t,F_req );
xlabel('Time [s]')
ylabel('Force output [N]')



% -----------------------------
% SENSITIVITY ANALYSIS (fixed and robust)
% -----------------------------
effifs = zeros(11, 10); % [paramIndex x modifierIndex]

for j = 1:12
    % modifiers to test ( 70% to 130%, steps of 5% of baseline)
    mods = [0.7,0.75,0.8, 0.85, 0.9,0.95, 1.0, 1.05, 1.1, 1.15, 1.20,1.25,1.3];
    for k = 1:length(mods)
        mod = mods(k);

        % Reset baseline parameters and apply single parameter perturbation
        param = zeros(1,12);
        param(1) = m;       % mass
        param(2) = h_aero;  % height of aero / CoM
        param(3) = RWD;     % rear weight fraction
        param(4) = C_d;     % drag coeff
        param(5) = mu_t;    % tire friction
        param(6) = C_rr;    % rolling resistance coeff
        param(7) = m_w;     % wheel mass
        param(8) = m_t;     % tire mass
        param(9) = r_w;     % wheel radius
        param(10)= r_t;     % tire radius
        param(11)= A_f;     % frontal area
        param(12)= T_eff;   % Drivetrain eff.

        param(j) = param(j) * mod; % change only parameter j

        % Recompute derived quantities
        m_p = param(1);
        h_aero_p = param(2);
        RWD_p = param(3);
        C_d_p = param(4);
        mu_t_p = param(5);
        C_rr_p = param(6);
        m_w_p = param(7);
        m_t_p = param(8);
        r_w_p = param(9);
        r_t_p = param(10);
        A_f_p = param(11);
        T_eff_p = param(12);
%{
        if T_eff_p > 1
            T_eff_p = 1;
        end
%}
        I_w_p = 0.5 * m_w_p * r_w_p^2;
        I_t_p = 0.5 * m_t_p * (r_t_p^2 + r_w_p^2);
        I_tot_p = I_t_p + I_w_p;
        m_eq_p = m_p + 3 * (I_tot_p * (1/r_w_p)^2);
        F_maxf_p = (mu_t_p * RWD_p * m_p * g) / (1 - (mu_t_p * h_aero_p) / l_wb);
        P_maxwh = P_max * T_eff_p;

        % reset dynamic arrays
        F_drag = zeros(1,N);
        P_drag = zeros(1,N);
        F_rr = zeros(1,N);
        P_rr = zeros(1,N);
        F_inertia = zeros(1,N);
        P_inertia = zeros(1,N);
        F_maxwh = zeros(1,N);
        F_req = zeros(1,N);
        P_x = zeros(1,N);
        v = zeros(1,N);
        a = zeros(1,N);
        x = zeros(1,N);

        v(1) = 0;
        x(1) = 0;

        for i = 1:N-1
            F_drag(i) = 0.5 * rho * C_d_p * A_f_p * v(i)^2;
            P_drag(i) = F_drag(i) * v(i);
            F_rr(i) = C_rr_p * m_p * g;
            P_rr(i) = F_rr(i) * v(i);

            if v(i) <= 0
                F_maxwh(i) = Inf;
            else
                F_maxwh(i) = P_maxwh / v(i);
            end

            if F_maxwh(i) < F_maxf_p
                F = F_maxwh(i);
            else
                F = F_maxf_p;
            end
            %F = 125;

             if isPulsing == true % 10.992 m/s = 24.6 mph, this scenario is to speed up to top end speed
                if v(i) >= v_max
                    isPulsing = false;
                end
            else   % 7.33 m/s = 16.4 mph, low end speed,
                F = 0;
                if v(i) <= v_min % 7.786 m/s = 9.16 - 15 percent
                    isPulsing = true;
                end
            end % end of pulsing if else
        
            % position update (explicit Euler)
        
                    a(i) = (F  - F_rr(i) - F_drag(i)) / m_eq; % Calculate acceleration
                    v(i+1) = v(i) + a(i); % Update velocity
                    x(i+1) = x(i) + v(i) * (t(i+1) - t(i)); % Update position
                    F_inertia(i) = m_eq * a(i); % inertial force (N)
                    P_inertia(i) = F_inertia(i) * v(i); % inertial power (W)
                
                    F_req(i)=F_inertia(i)+F_rr(i)+F_drag(i);
                    P_x(i)=F_req(i)*v(i);
                    dt = t(i+1) - t(i);
                    x(i+1) = x(i) + v(i) * dt;
        end

        Energy_kWh_p = trapz(t, max(P_x, 0)) / (3600 * 1000);
        distance_miles_p = x(end) * m_to_mi;
        if Energy_kWh_p > 0
            effifs(j,k) = distance_miles_p / Energy_kWh_p;
        else
            effifs(j,k) = NaN;
        end
    end
end

% Show sensitivity table (rows: param index 1..11, cols: modifiers 0.9/1.0/1.1)
%{
disp('Sensitivity results (mi/kWh) for params 1..11, modifiers [0.9 1.0 1.1]:')
disp(effifs)

%}

%{
%% Plotting the Bar graph
Parameters = {'Mass', 'H CG', 'RWD', 'Drag Coeff.', 'Tire friction', 'RR Coeff.', 'Wheel Mass', 'Tire Mass', 'Wheel Radius', 'Tire Radius', 'Frontal Area', 'Drivetrain Efficiency'};
figure('Name', 'Sensitivity');
bar(Parameters, effifs);
title('Sensitivity Analysis of Car Simulation');
ylabel('Vehicle Efficiency [mi/kWh]');
xlabel('Variable Changed');
legend('-30%','-25%','-20%','-15%','-10%','-5%', '0%', '+5%', '+10%', '+15%', '+20%','+25','+30%');

figure('Name', 'Sensitivity Graph');
plot((mods - 1)* 100,effifs);
title('Sensitivity Analysis of Car Simulation');
ylabel('Vehicle Efficiency [mi/kWh]');
xlabel('Variable Changed [%]');
legend(Parameters);
%}
p = [effifs(1,:) ; effifs(4,:) ; effifs(6,:)];

New_p = {'Mass', 'Aero coefficient', 'Rolling resistance'};
figure('Name', 'New Sensitivity');
bar(New_p,p);
title('Sensitivity Analysis of Car Simulation');
ylabel('Vehicle Efficiency [mi/kWh]');
xlabel('Variable Changed');
legend('-30%','-25%','-20%','-15%','-10%','-5%', '0%', '+5%', '+10%', '+15%', '+20%','+25','+30%');

E = ones(1,length(mods));

%NEW PLOT


figure('Name', 'Updated Sensitivity Graph')
plot((mods - 1)* 100,p, (mods - 1)* 100, E * eff_mi_per_kWh);
title('Sensitivity Analysis of Car Simulation');
ylabel('Vehicle Efficiency [mi/kWh]');
xlabel('Variable Changed [%]');
legend('Mass', 'C_d * A_f', 'RR', 'Baseline');
p_percent = ( p - p(1,7)) / p(1,7) * 100;


%{
% -------------------------------------
% For the Gear Ratio
% -------------------------------------

% MOTOR MAX EFFIF = 86.6 %

r = 10 / 39.37; % Tire radius [in to m]
kv = 33.4; % Speed Constant [rpm/V]
I_max = 10; % [A]
g_p = 10; % Pinion gear teeth
g_s = 120; % Spur gear teach
g_r = g_s / g_p; % GEAR RATIO
w_max = kv * V; % max rpm AT THE MOTOR
T_max = I_max * V / (w_max * rad); % Max torque at given power FOR THE MOTOR[Nm]
v_max = 9.16; % Max speed [m/s]

% FOR CURRENT MODEL

T_model = T_max * T_eff * g_r; % The model's max torque
w_model = w_max / g_r; % The model's max rpm



% Time vector (seconds)
t = 0:1:1800; % 30 minutes at 1-second steps

% Pre-allocate arrays
N = length(t);
F_drag = zeros(1,N);
P_drag = zeros(1,N);
F_rr = zeros(1,N);
P_rr = zeros(1,N);
F_inertia = zeros(1,N);
P_inertia = zeros(1,N);
F_motor = zeros(1,N);
P_motor = zeros(1,N);
F_maxwh = zeros(1,N);
F_req = zeros(1,N);
P_x = zeros(1,N);
v = zeros(1,N);
a = zeros(1,N);
x = zeros(1,N);
w_m = zeros(1,N);
%}
