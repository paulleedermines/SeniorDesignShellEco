% SHELL ECOMARATHON Vehicle Model
% By Jordan Jeong and Lucas Razanauskas

%GOAL: AS FUEL EFFICIENT AS POSSIBLE FOR 10 MILES IN 30 MINUTES, about 16.09km

% Constants
clc; clear all; close all; 

%% Vehicle parameters

C_d = 0.20;                 % Drag coefficient [unitless] - estimated value given known values of similar vehicles
A = 0.5;                    % Frontal area [m^2]
m = 85;                     % Vehicle mass [kg]
h_CG = 0.375;               % Vehicle center of gravity height [m]
h_aero = 0.375;             % Vehicle center of pressure [m] (assume same height as CG)
theta = 0;                  % Grade angle [degrees]
rho = 1.2;                  % Air density in Indianapolis [kg/m^3]
l_wb = 1.397;               % Wheel base [m]
g = 9.81;                   % Acceleration due to gravitya [m/s^2]
Wff = 55.45*g;              % Weight on front axle [N]
Wfr = 29.55*g;              % Weight on rear axle [N]
RR = 0.004;                 % Rolling resistance coefficient [unitless]
r_w = .203;                 % Wheel radius [m]
r_t = .2413;                % Tire radius [m]
m_t = .35;                  % Tire mass (single) [kg]
m_w = .65;                  % Wheel mass (single) [kg]
m_as = m_t + m_w;           % Wheel assembly mass (single) [kg]
I_t = 1/2*(m_t*(r_t)^2);    % Inertia of tire [kg*m^2]
I_w = 1/2*(m_w*(r_w)^2);    % Inertia of wheel [kg*m^2]
I_c = 3*(I_t+I_w);          % Inertia of all three wheels and tires [kg*m^2]
mu=0.7;                     % Tire friction coefficient [unitless]
m_eq=3*m+I_c/r_t^2;         % Equivalent mass [kg]


v_peak = 9.3;                 % Speed at which to start coasting [m/s]
v_min = 8.6;                  % Speed at which to stop coasting [m/s]
coast = false;              % Boolean for turning coasting on/off

% Motor Properties
% Must enter the max RPM (no load speed) and max Torque (stall torque) for
% accurate results
V_max = 48; % Motor Voltage [V]
I_max = 56.9; % Stall Current [A]
I_nl = 0.2; % No load current [A]
RPM_max = 3750; % Max Motor Speed [RPM] aka No Load Speed
rad_sec = RPM_max*2*pi()/60;
GR = 9.12; % Max gear ratio
tq_stall = 13.1; % Stall Torque [N-m]
p_max = tq_stall*rad_sec*0.25;
v_max = rad_sec/GR*r_t % Max linear speed
if v_max < 9
    disp("Shit's fucked, not fast enough top speed.")
end


% Drivetrain Properties
eff_d = 0.85;              % Assume 85% drivetrain efficiency
                           % Assume this efficiency is constant at all speeds
                           % Technically gear ratio should go in here but
                           % whatever
% Initialising vectors
dt = 0.01;
t=0:dt:1800;             % Time vector [s] goes to 1800 seconds (30 minutes), duration of race
lt = length(t);
a=zeros(1,lt);                      % define initial acceleration as 0 [m/s^2]
v=zeros(1,lt);                      % define initial velocity as 0 [m/s]
x=zeros(1,lt);                      % define initial position as 0 [m]
Fx=zeros(1,lt);                     % Force at the wheel/road surface [N]
Faero=zeros(1,lt);                  % Force of aerodynamic drag [N]
F_rr=zeros(1,lt);                   % Force of rolling resistance [N]
F_grade=zeros(1,lt);                % Force of grade/incline [N]
eff_m=zeros(1,lt);                  % Efficiency of motor
eff_max = 0.86;                     % Peak motor efficiency as advertised
V = zeros (1,lt);                   % Voltage at specific time
I = zeros (1,lt);                   % Current at specific time
I(1) = I_max;                       % Car will always start at stall current
P = zeros (1,lt);                   % Power at specific time
E = zeros (1,lt);                   % Total Energy at specific time [kWh] 
goal = zeros(1,lt);                 % This is what we want to be 200+ [mi/kWh]
eff_m_in = zeros(1,101);
for j=1:80
    eff_m_in(j) = eff_max*j/86+0.01;
end
for j=81:101
    eff_m_in(j) = eff_max*(1-(j-90)^2);
end
rpm = 0:1:100;
eff_m_in

for j=2:length(t)
    speed_percent = v(j-1)/v_max;                   % How fast the motor is spinning compared to its maximum speed
    
    eff_m(j) = 0.85*(speed_percent/0.9)+0.01;       % Efficiency is a function of motor speed, where peak is at 90% of peak RPM
    %replace this with an array and use interp1
    tq = tq_stall*(1-speed_percent);                % The faster you are spinning, the less torque you have
    Fx(j)= mu*Wfr*cosd(theta)/(1-mu*h_CG/l_wb);     % Max tractive force of rear wheel [N]
    
    if Fx(j)*r_t > (tq*GR)                          % Tractive force times tire radius cannot exceed provided torque
        Fx(j)= tq*GR/r_t*eff_d*eff_m(j);                  
    else
        Fx(j)=Fx(j)*eff_d*eff_m(j);                       
    end

    V(j) = V_max*(speed_percent);                   % Probably have to use kV here rather than speed percent but oh well
    I(j) = (I_max-I_nl)*(1-speed_percent)+I_nl;     % Not sure about this
    
    Faero(j) = 0.5*rho*C_d*A*v(j-1)^2;              % Aero force [N]
    F_rr(j)=v(j-1)*RR*m*g*cosd(theta);              % Rolling resistance [N]
    F_grade(j)=m*g*sind(theta);                     % Grade force (always zero since the track is flat) [N]
    if coast
        V(j) = 0;
        I(j) = 0;
        Fx(j) = 0;
    end   
    a(j)=(Fx(j)-Faero(j)-F_rr(j)-F_grade(j))/m_eq;  % Resultant acceleration[m/s^2]
    v(j)=a(j)*(t(j)-t(j-1))+v(j-1);                 % Velocity [m/s]
    P(j) = V(j)*I(j)/1000;                          % Power [kW]
    E(j) = E(j-1)+P(j)*dt/3600;                     % Energy [kWh]
    if v(j)>v_max*0.9                               % For simplicity, we limit motor speed to 90% of peak
        v(j)=v_max*0.9;
        a(j)=0;
    end
    x(j)=v(j)*(t(j)-t(j-1))+x(j-1);                 % Position [m]
    if x(j) > 15700
        x(j) = 15700;
        v(j)= 0;
    end
    goal(j) = x(j)./E(j)/1610;
    if v(j) >= v_peak
        coast = true;
    end
    if v(j) <= v_min
        coast = false;
    end
end
goal(2) = 0; % prevents infinite spike because E(2) is zero because V(2) is zero because v(1) is zero


figure
plot(t,v)
xlabel("Time (s)")
ylabel("Velocity (m/s)")
yline(v_max)
sgtitle("Velocity over Time")

figure
plot(t,x)
xlabel("Time (s)")
ylabel("Distance (m)")
yline(15700/4)
yline(15700/2)
yline(15700*3/4)
yline(15700)
sgtitle("Distance over Time")

figure
plot(t,P)
xlabel("Time (s)")
ylabel("Power (kW)")
%yline(600)
sgtitle("Power")

figure
plot(t,E)
xlabel("Time (s)")
ylabel("Energy (kWh)")
sgtitle("Total Energy")

figure
plot(t,goal)
xlabel("Time (s)")
ylabel("Efficiency (mi/kWh)")
sgtitle("Efficiency")

figure
plot(t,eff_m)
xlabel("Time (s)")
ylabel("Efficiency")
sgtitle("Motor Efficiency")

figure
plot(t,V)
xlabel("Time (s)")
ylabel("Voltage (V)")
sgtitle("Voltage")

figure
plot(t,I)
xlabel("Time (s)")
ylabel("Current (A)")
sgtitle("Current")


% Need to make an array or equation of efficiency of the motor
% E_motor = [] is a function of rpm 
% since we don't have a given motor curve, we assume peak efficiency is at
% 90% of peak RPM and a linear decrease to 0% effiency at 0% RPM
% Therefore efficiency = 0.9*(RPM/Peak RPM)*Peak efficiency

% Assume we can corner at full speed
% Track Info
% Total length: 3925m (official)
% Below distances measured with google maps
% Starting line to first corner: 636m
% Turn 1/2: 129m (coasting) to 765m
% turn 3 is full speed
% turn 4 entry at 1000m, 235m of possible acceleration
% turn 4 is 95m
% then 114m of straight
% then 56m coasting through chicanes
% then 714m straight
% then 91m turn
% then 55m straight
% then 129m of s turns
% then 78m before turn
% then 30m turn
% then 678m of straight/mild curve
% then 175 m of turns
% 99m straight
% 116m turn
% 410 m straight
% google measurements are 3840, or 85m less than official; maybe small car
% cuts corners