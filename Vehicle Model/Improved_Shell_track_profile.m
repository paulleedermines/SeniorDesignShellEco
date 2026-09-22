% Improved track profile for SHELL ECOMARATHON (updated 2.4.26)
% By Jordan Jeong and Co.
% Mines 2026 Team

% This is to basically convert the track profile, and make the velocity
% profile more smoother and would coast down to specific speeds instead of
% braking to a certain speed

% This requires to understand where and how far the throttle would have to
% be let go to coast down to the correct speed 
%

% FOR TURNS 1, 7 and 13

% TRACK LENGTH = 3.926 km per lap (2.439 mi), or 15.704 total four laps (9.756 mi).

clear; clc; format short;

rho = 1.004;           % [kg/m^3] Air density in Colorado
rad = (2*pi/60);      % Conversion factor for RPM to rad/sec (not used currently)
g = 9.81;             % [m/s^2] gravity
V = 48;               % [V] Voltage of car
I_max = 1;           % [A] Current of car
C_d = 0.1;            % Drag Coeff
C_rr = 0.0056;         % Rolling Coeff.
Wd = [61,61,65] ./ 2.205; % [kg] Weight Distribution [FL,FR,R]
m = sum(Wd);          % [kg] total weight of car
h_aero = 0.375;       % [m] height of aero 
h_cg = h_aero;        % assume CoM height same as aero height
RWD = Wd(3) / m;       % [Weight fraction on rear axle]

T_motor = 1.49; % [N-m] Torque output from the motor
gr = 8; % Gear Ratio
T_out = T_motor * gr; % [N-m] Torque output after gearing

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

% For the coasting before entering turns
isCoasting = false;
mode = "drive";   % drive | coast | hold



% FOR EFFICIENCY VALUE, REPLACE THIS -------------------------------------
T_eff = 1.0;          % transmission efficiency

A_f = 0.71;             % [m^2] frontal area
P_max = V * I_max;     % [W] motor input
P_maxwh = T_eff * P_max; % [W] available at wheels (approx)

% Traction-limited force (static formula); keep denominator safe
F_maxf = (mu_t*RWD*m*g) / (1 - ((mu_t*h_cg)/l_wb)); % [N] Rear axle tractive force

% Equivalent mass including rotating inertia
m_eq = m + 3 * (I_tot * (1/r_w)^2); % [kg] mass equivalent

% Time vector (seconds)
t = 0:.1:1800; % 30 minutes at .1-second steps

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
dt = .1;
%{
% FOR HYPERMILING, Hypermiling will probably only apply for straights
and MAYBE for turns big enough to go at the straight speeds
v_max =;
v_min =;

%}

% BELOW IS ALL TRACK VALUES AND DATA FROM ME, NO CODE, BUT IS GOOD TO READ
% TO UNDERSTAND HOW CERTAIN NUMBERS WERE PRODUCED, IMPORTANTCE OF THESE
% NUMBERS, ETC.

% Velocity Values of turns, st = straight , numbers are the turn or straight numbers,
% if it has n, it is both turn numbers (1 and 2, 5 and 6, etc)
% R = Length of turn / turn angle
% v_max = sqrt(mu*g*R) *THIS IS THE MAX SPEED THE VEHICLE CAN GO THROUGH
% THE CORNER
% NOTE: SINCE THESE MEASUREMENTS ARE OFF OF SATELITE DATA ON GOOGLE MAPS
% AND EARTH, MEASURED FROM THE CENTER OF TRACK,
% I WILL ROUND DOWN AND MAYBE SUBTRACT 1 M/S TO ENSURE GRACE
% DOES NOT DIE :)

% note: will give turn number, or if it is straight for certain amount of
% distance, and its distance in the lap, parts of the track highlighted in
% red means no hypermiling, regardless what speed is allowed to drive

% START TO TURN = ABOUT 0 - 636 METERS

% TURN 1 = 20 meters, 90 degree turn STARTS AT POINT 636m - ENDS AT 656m
% R = 20m / (pi/2) = 25.477 meters
% v_max1 = 9.35 m/s
% will use 8 m/s

% STRAIGHT FOR 40 METERS - 656 TO 696


% TURN 2 = 69 METERS, 90 degree turn, 696 TO 765
% R = ROUGHLY 40 METERS
% v_max = v_st

% TURN 3 IS BIG ENOUGH TO GO NORMAL SPEED = 275 METERS, 765 TO 1040

% TURN 4 = 35 METERS, 95 METERS TOTAL,  60 degree turn FROM 1040 TO 1075
% 1075 TO 1135 IS STRAIGHT
% BIG ENOUGH TO GO STRAIGHT
% v_max = v_st

% STRAIGHT FOR = 114 METERS, FROM 1135 TO 1249

% TURN 5-6 CHICAINE = 56 METERS FROM 1249 TO 1305
% half = 28 meters, 90 degrees both
% 28m / (pi/2), r = 17.825 meters
% v_max = 11.06 m/s
% (BIG ENOUGH TO GO NORMAL SPEED, no hypermiling)

% STRAIGHT FOR = 714 METERS, FROM 1305 TO 2019

% TURN 7 = 35 METERS, 90 degree turn, FROM 2019 TO 2054
% R = 40 / (pi/2) = 25.477 meters
% v_max7 = 9.577 meters, use 8.3

% STRAIGHT FOR = 101 METERS, FROM 2054 TO 2155

% TURN 8-9 CHICAINE = 160 METERS, FROM 2155 TO 2315
% big enough to go normal

% STRAIGHT FOR = 78 METERS, FROM 2315 TO 2393

% TURN 10 = 30 METERS, 60 degrees, FROM 2393 TO 2423
% R = 30m / (pi/3) = 28.66m , v_max = 14 m/s GO NORMAL

% TURN 11 = 678 METERS (BIG ENOUGH TO GO NORMAL) FROM 2423 TO 3101

% TURN 12 = 35 meters, 90 degree turn, FROM 3101 TO 3136
% R = 35 / (pi/2) = 22.93 m
% v_max = 12.37 m/s

% Straight for 120 meters, FROM 3136 TO 3256 (might adjust this straight
% length to coast sooner for turn 13)

% TURN 13 = 45 METERS, about 135 degree turn, FROM 3256 TO 3301
% R = 45 / (3pi/2) = 9.554 meters
% v_max = 8.1 m/s, will use 7 m/s


% TURN 14 = 215 METERS (BIG ENOUGHT TO GO NORMAL) FROM 3301 TO 3516

% STRAIGHT FOR = 410 METERS, FROM 3516 TO 3926

% COMPLETION OF ONE LAP == 3926 METERS
% Four total laps = 15,704 meters 
% (FOR COMPLETION, MUST AVERAGE 19.5 MPH)


v_st = 20.5 / 2.237 ; % velocity on straights (20 mph, will be based on what the turn velocities are)

v_st1 = v_st; % STRAIGHT BETWEEN START AND 1

v_1 = 8; % velocity on turn 1 [m/s]

v_2 = v_st; % velocity on turn 2

v_3 = v_st; % turn 3 is found to be big enough to hypermile

v_4 = v_st; % TURN 4

v_st2 = v_st; % STRIAGHT BETWEEN 4 AND 5

v_5n6 = v_st; % 5 AND 6 CHICANE

v_st3 = v_st; % STRAIGHT BETWEEN 6 AND 7 (HAHAHA)

v_7 = 8.3; % TURN 7 [m/s]

v_st4 = v_st; % STRAIGHT BETWEEN 7 AND 8

v_8n9 = v_st; % 8 AND 9 CHICANE

v_st5 = v_st; % STRAIGHT BETWEEN 9 AND 10

v_10 = v_st; % TURN 10

v_11 = v_st; % TURN 11 (PRETTY MUCH A STRAIGHT)

v_12 = v_st; % TURN 12

v_st6 = v_st; % STRAIGHT BETWEEN 12 AND 13 (might delete since so small and going into toughest corner)

v_13 = 7; % TURN 13 [m/s]

v_14 = v_st; % TURN 14 (STRAIGHT)

v_st7 = v_st; % STRAIGHT BETWEEN 14 AND FINISH


lap_num = 1; % number of laps completed, once hits lap 5, race will end

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
    F = T_out / r_t; % this is because I am pretty sure we are always gearing limited, w a gear ratio of 8, nominal torque

    % THIS IS THE COASTDOWN DISTANCE SOLVING: BASCIALLY HAD TO DERIVE THE
    % VEL. PER TIME TO CHANGE TO DIST. PER VELOCITY. LMK IF YOU WANT THE
    % ACTUAL DERIVATION, BUT THE s_... IS THE DERIVATION AND SOLUTION FOR
    % IT, b AND c ARE FOR CONSTANT HOLDERS

    b = C_rr * m * g;
    c = 0.5 * rho * C_d * A_f;

    s_1 = m_eq / 2 / c * log(( b + c * v_st1^2)/( b + c * v_1^2));

    s_7 = m_eq / 2 / c * log(( b + c * v_st^2)/( b + c * v_7^2));

    s_13 = m_eq / 2 / c * log(( b + c * v_12^2)/( b + c * v_13^2));

    % FOR STRAIGHT 1 (Can hypermile for future code)
    if x(i) < 636 - s_1
        if v(i) < v_st1
            mode = "drive";
            F = T_out / r_t;
        else 
            mode = 'coast';
            F = F_drag(i) + F_rr(i);
        end
    end
    
    % COAST INTO TURN 1
    if x(i) >= 636 - s_1 && x(i) < 636
        mode = "coast";
        F = 0;
    end
    
    % TURN 1
    if x(i) >= 636 && x(i) < 656
        mode = "hold";
        F = F_drag(i) + F_rr(i);   % constant-speed corner
    end

    %For turn 2 (maintain v_2)
    if x(i)>= 656 && x(i) < 765
        v_use = v_2;
        mode = "hold";
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
            
    end
    
    % For turn 3 (can Hypermile)
    if x(i) >= 765 && x(i) < 1040
        v_use = v_3;
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end

    end

    % For turn 4
    if x(i) >= 1040 && x(i) < 1135
        v_use = v_4;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
    end

    % For straight 2 (can hypermile)
    if x(i) >= 1135 && x(i) < 1249
        v_use = v_st2;
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end

    end
    % For turn 5 and 6 chicane
    if x(i) >= 1249 && x(i) < 1305
        v_use = v_5n6;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
    end

    % For straight 3
    if x(i) >= 1305 && x(i) < 2019 - s_7
        v_use = v_st3;
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end

    end

    % For coasting before turn 7
    if x(i) >= 2019 - s_7 && x(i) < 2019
        mode = "coast";
        F = 0;
    end
    

    % For turn 7
    if x(i) >= 2019 && x(i) < 2054
        v_use = v_7;
        mode = "hold";
        F = F_drag(i) + F_rr(i);   % constant-speed corner
    end

    % For straight 4
    if x(i) >= 2054 && x(i) < 2155
        v_use = v_st4;
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end

    end

    % For turn 8 and 9 chicane
    if x(i) >= 2155 && x(i) < 2315
        v_use = v_8n9;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
    end

    % For straight 5
    if x(i) >= 2315 && x(i) < 2393
        v_use = v_st5;
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
    end

    % For turn 10
    if x(i) >= 2393 && x(i) < 2423
        v_use = v_10;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
    end

    % For turn 11
    if x(i) >= 2423 && x(i) < 3101
        v_use = v_11;
        if v(i) < v_use 
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
    end

    % For prep for turn 13 *has to coast through turn 12
    if x(i) >= 3101 && x(i) < 3256
        F = 0;
    end

    % For turn 13
    if x(i) >= 3256 && x(i) < 3301
        v_use = v_13;
        mode = "hold";
        F = F_drag(i) + F_rr(i);   % constant-speed corner
    end
    
    % For turn 14
    if x(i) >= 3301 && x(i) < 3516
        v_use = v_14;
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end
    
    end

    % For straight 7
    if x(i) >= 3516 && x(i) < 3926
        v_use = v_st7;
        if v(i) < v_use
            F = T_out / r_t;
        else
            F = F_drag(i) + F_rr(i);
        end

    end

    % Check if lap is complete
    if x(i) > 3926
        lap_num = lap_num + 1;
        x(i) = x(i) - 3926;
    end

    % Check if four laps has been completed (RACE HAS BEEN DONE)
    if lap_num == 5
        disp('Exiting loop because four laps have been completed!')
        Time_comp = t(i) / 60 % THIS IS THE COMPLETION TIME IN MINUTES
        break
    end

    %Real Force output, taking into account efficiencies
    F_real = F * T_eff;
        

    a(i) = (F_real - F_rr(i) - F_drag(i)) / m_eq;
    % protect against tiny negative accelerations causing small negative speeds
    v(i+1) = max(0, v(i) + a(i) * dt);

    % position update (explicit Euler)

    x(i+1) = x(i) + v(i) * dt;

    % inertia force and power
    F_inertia(i) = m_eq * a(i);
    P_inertia(i) = F_inertia(i) * v(i);

    % total required force and power (this uses the old F value b/c of
    % mechancical power output from motor, not onto the ground)
    F_req(i) = F;
    P_x(i) = F_req(i) * v(i);
end


% Energy used (integrate positive power only)
Energy_kWh = trapz(t, max(P_x, 0)) / (3600 * 1000); % [kWh]

% Convert displacement to miles correctly and compute efficiency mi/kWh
m_to_mi = 1/1609.344;
distance_miles = 15704 * m_to_mi;
if Energy_kWh > 0
    eff_mi_per_kWh = distance_miles / Energy_kWh;
else
    eff_mi_per_kWh = NaN;
end
x(end) = 15704;
fprintf('Distance [m]: %.1f  Distance [mi]: %.4f  Energy [kWh]: %.4f  Eff [mi/kWh]: %.4f\n',...
    x(end), distance_miles, Energy_kWh, eff_mi_per_kWh)

% Displacement plot
figure('Name', 'Displacement vs Time');
set(gca,'FontSize',16);
set(0, 'DefaultLineLineWidth', 1.5);
hold on; grid on; box on;
plot(t, x ,'DisplayName', 'Displacement [m]');
xlabel('Time [s]');
ylabel('Displacement  [m]');
legend

% velocity plot
figure('Name', 'Velocity vs Time');
set(gca,'FontSize',16);
set(0, 'DefaultLineLineWidth', 1.5);
hold on; grid on; box on;
plot(t, v ,'DisplayName', 'Velocity [m/s]');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend