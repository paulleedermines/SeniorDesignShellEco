% Improved track profile for SHELL ECOMARATHON FINAL (4.21.26)
% With hypermiling
% By Jordan Jeong and Dominic Padula
% Mines 2026 Team

% The final version was last edited April 11th, 2026.  This was in Indy and
% the track was slightly adjusted at competition. To combat with the
% shortening of the track, we shortened the final straightaway by 100
% meters, we declared that the first corner and 7th turn
% was no longer an issue (this may or may not be true for future years).
% Turn 13 was still needed to be considered. 

% 4.21.26 update: This was to clean up any code and remove and redundancy
% that next years team would not use and make sure they understand it. 
% There seems to be about a 13 mi/kWh improvement using this (slighly
% simplified) strategy over the earlier constant speed cornering

% This is to basically convert the track profile, and make the velocity
% profile more smoother and would coast down to specific speeds instead of
% braking to a certain speed

% This requires to understand where and how far the throttle would have to
% be let go to coast down to the correct speed 


% FOR TURNS 1, 7 and 13

% TRACK LENGTH = 3.826 km per lap, or 15.304 km total four laps.

clear; clc; format short;
close all

rho = 1.004;           % [kg/m^3] Air density in Colorado
rad = (2*pi/60);      % Conversion factor for RPM to rad/sec (not used currently)
g = 9.81;             % [m/s^2] gravity
V = 60;               % [V] Voltage of car
I_max = 10;           % [A] Current of car
C_d = 0.1;            % Drag Coeff
C_rr = 0.0056;         % Rolling Coeff.
Wd = [61,61,65] ./ 2.205; % [kg] Weight Distribution [FL,FR,R]
m = sum(Wd)-1.2;          % [kg] total weight of car

h_aero = 0.375;       % [m] height of aero 
h_cg = h_aero;        % assume CoM height same as aero height
RWD = Wd(3) / m;       % [Weight fraction on rear axle]

T_motor = 0.9; % [N-m] Torque output from the motor
GR = 9.23; % Gear Ratio
% note: this is the GR on the car with a 13T small pulley, ideal we found
% was 10 with a 12T but this pulley was slipping and it gets close to
% maxxing out the motor speed
T_out = T_motor * GR; % [N-m] Torque output after gearing

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

AccessoryPower=0; %ADD COMMS POWER ETC HERE

Drivetrain_eff=0.93; %CDR estimate
MotorControl_eff=0.95; %Good estimate from EE team. This is used later in code
% FOR EFFICIENCY VALUE, REPLACE THIS -------------------------------------
%it seems like the logic train this starts ends without being used
%so I am not implementing motor efficiency here
%motor_out=motor_efficiency((F*r_t),((v(i)*60)/(2*3.14159*r_t)),V );

%T_eff = MotorControl_eff * Drivetrain_eff;         % transmission efficiency e
%Teff does NOT include motor itself.
A_f = 0.71;             % [m^2] frontal area
P_max = V * I_max;     % [W] motor input
P_maxwh = Drivetrain_eff *MotorControl_eff* P_max; % [W] available at wheels (approx)

% Traction-limited force (static formula); keep denominator safe
F_maxf = (mu_t*RWD*m*g) / (1 - ((mu_t*h_cg)/l_wb)); % [N] Rear axle tractive force

% Equivalent mass including rotating inertia
m_eq = m + 3 * (I_tot * (1/r_w)^2); % [kg] mass equivalent

% Time vector (seconds)
t = 0:.1:2100; % 35 minutes at .1-second steps


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
MotorRPM=zeros(1,N);
MotorTorque=zeros(1,N);
MotorEff=zeros(1,N);

% Initial condition: start from rest (explicit)
v(1) = 0;
x(1) = 0;
dt = .1;


% BELOW IS ALL TRACK VALUES AND DATA FROM ME, NO CODE, BUT IS GOOD TO READ
% TO UNDERSTAND HOW CERTAIN NUMBERS WERE PRODUCED, IMPORTANTCE OF THESE
% NUMBERS, ETC.

% After comp note: This is all pretty much useless after the changes they
% implemented. This will stay in case they consider to change the track to
% the layout they displayed

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

% STRAIGHT FOR = 310 METERS, FROM 3516 TO 3826

% COMPLETION OF ONE LAP == 3926 METERS
% Four total laps = 15,704 meters 
% (FOR COMPLETION, MUST AVERAGE 19.5 MPH)

vh_min= 16.5 /2.237;
vh_max=18.5/2.237;  %getting hypermiling stuff. These values were adjusted from CDR numbers to adjust the 
%average speed lower, as at comp Shell changed the time from 30 to 35 mins

vmax_firstgear=15/2.237; % Note: This was not used in comp because the shifter battery died, GR change was adjusted in the code, so there is no gear change
%You can get a bit more efficiency by using the lower gear at initial
%acceleration up to speed. To add this back in, uncomment out the GR/0.7 line around line 590ish and comment out the GR/1 line
   % % if v(i)<vmax_firstgear  <~~search this if you cant find the line
   % %         %GRi=GR/0.7; <~~ add this back in
   % %         GRi=GR/1;  <~~comment this out

v_st = 17.5 / 2.237 ; % velocity on straights (17.5 mph, will be based on what the turn velocities are)

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

i=0;
j=0;
while true
    i=i+1;
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

    % Determine limiter: powertrain [f] or traction [wh]
    if F_maxwh(i) < F_maxf
        F = F_maxwh(i); % ~ At the lowest 70N
    else
        F = F_maxf; % ~Roughly 250N
    end
    F = T_out / r_t; 
    % this is because I am pretty sure we are always gearing limited, w a
    % gear ratio of 10, nominal torque ~ 37N

    % THIS IS THE COASTDOWN DISTANCE SOLVING: BASCIALLY HAD TO DERIVE THE
    % VEL. PER TIME TO CHANGE TO DIST. PER VELOCITY. LMK IF YOU WANT THE
    % ACTUAL DERIVATION, BUT THE s_... IS THE DERIVATION AND SOLUTION FOR
    % IT, b AND c ARE FOR CONSTANT HOLDERS. THE DERIVATION WAS IN 2026 CDR

    % After competition note: s_1 and s_7 are pretty much useless for the
    % course after changes, but good to keep in mind if they change it to
    % be how the road course is set up

    b = C_rr * m * g;
    c = 0.5 * rho * C_d * A_f;

    s_1 = m_eq / 2 / c * log(( b + c * v_st1^2)/( b + c * v_1^2));

    s_7 = m_eq / 2 / c * log(( b + c * v_st^2)/( b + c * v_7^2));

    s_13 = m_eq / 2 / c * log(( b + c * v_12^2)/( b + c * v_13^2));
    
    % This was also not used, but this is the case of going from
    % hypermiling (max speed) to coast to avg speed with no issues. This is used in
    % past versions of the code.
    %s_hype = m_eq / 2 / c * log(( b + c * v_max^2) / (b + c * v_st^2) );

    % FOR STRAIGHT 1 (Can hypermile for future code)
    if x(i) < 636 - s_1
        if v(i) < vh_max && mode ~= "coast"
            mode = "drive";
            F = T_out / r_t;
        elseif v(i) >= vh_max
            mode="coast";
            F=0;
        elseif v(i) < vh_min
            mode="drive";
            F = T_out / r_t;
        else 
            F=0;
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
        %F = F_drag(i) + F_rr(i);   % constant-speed corner
        %^test removal
        F=0; %testing a coast here
    end

    %For turn 2 (maintain v_2)
    if x(i)>= 656 && x(i) < 765
        v_use = v_2;
        mode = "hold";
        if v(i) < v_use
            F = T_out / r_t;
        else
            %F = F_drag(i) + F_rr(i);
             %^test removal
             F=0; %testing a coast here
        end
            
    end
    
    % For turn 3 (can Hypermile)
    if x(i) >= 765 && x(i) < 1040
        v_use = v_3;
        if v(i) < vh_max && mode ~= "coast"
            mode = "drive";
            F = T_out / r_t;
        elseif v(i) >= vh_max
            mode="coast";
            F=0;
        elseif v(i) < vh_min
            mode="drive";
            F = T_out / r_t;
        else 
            F=0;
        end

    end

    % For turn 4
    if x(i) >= 1040 && x(i) < 1135
        v_use = v_4;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            % F = F_drag(i) + F_rr(i);
             %^test removal
             F=0; %testing a coast here
        end
    end

    % For straight 2 (can hypermile)
    if x(i) >= 1135 && x(i) < 1249
        v_use = v_st2;
        if v(i) < vh_max && mode ~= "coast"
            mode = "drive";
            F = T_out / r_t;
        elseif v(i) >= vh_max
            mode="coast";
            F=0;
        elseif v(i) < vh_min
            mode="drive";
            F = T_out / r_t;
        else 
            F=0;
        end

    end
    % For turn 5 and 6 chicane
    if x(i) >= 1249 && x(i) < 1305
        v_use = v_5n6;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            %F = F_drag(i) + F_rr(i);
             %^test removal
        F=0; %testing a coast here
        end
    end

    % For straight 3
    if x(i) >= 1305 && x(i) < 2019 - s_7
        v_use = v_st3;
        if v(i) < vh_max && mode ~= "coast"
            mode = "drive";
            F = T_out / r_t;
        elseif v(i) >= vh_max
            mode="coast";
            F=0;
        elseif v(i) < vh_min
            mode="drive";
            F = T_out / r_t;
        else 
            F=0;
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
        %F = F_drag(i) + F_rr(i);   % constant-speed corner
         %^test removal
        F=0; %testing a coast here
    end

    % For straight 4
    if x(i) >= 2054 && x(i) < 2155
        v_use = v_st4;
        if v(i) < vh_max && mode ~= "coast"
            mode = "drive";
            F = T_out / r_t;
        elseif v(i) >= vh_max
            mode="coast";
            F=0;
        elseif v(i) < vh_min
            mode="drive";
            F = T_out / r_t;
        else 
            F=0;
        end

    end

    % For turn 8 and 9 chicane
    if x(i) >= 2155 && x(i) < 2315
        v_use = v_8n9;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            %F = F_drag(i) + F_rr(i);
             %^test removal
        F=0; %testing a coast here
        end
    end

    % For straight 5
    if x(i) >= 2315 && x(i) < 2393
        v_use = v_st5;
        if v(i) < vh_max && mode ~= "coast"
            mode = "drive";
            F = T_out / r_t;
        elseif v(i) >= vh_max
            mode="coast";
            F=0;
        elseif v(i) < vh_min
            mode="drive";
            F = T_out / r_t;
        else 
            F=0;
        end
    end

    % For turn 10
    if x(i) >= 2393 && x(i) < 2423
        v_use = v_10;
        mode = "hold";
        if v(i) < v_use 
            F = T_out / r_t;
        else
            %F = F_drag(i) + F_rr(i);
             %^test removal
        F=0; %testing a coast here
        end
    end

    % For turn 11
    if x(i) >= 2423 && x(i) < 3101
        v_use = v_11;
        if v(i) < v_use 
            F = T_out / r_t;
        else
            %F = F_drag(i) + F_rr(i);
             %^test removal
        F=0; %testing a coast here
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
        %F = F_drag(i) + F_rr(i);   % constant-speed corner
         %^test removal
        F=0; %testing a coast here
    end
    
    % For turn 14
    if x(i) >= 3301 && x(i) < 3516
        v_use = v_14;
        if v(i) < v_use
            F = T_out / r_t;
        else
            %F = F_drag(i) + F_rr(i);
             %^test removal
        F=0; %testing a coast here
        end
    
    end

    % For straight 7
    if x(i) >= 3516 && x(i) < 3825
        v_use = v_st7;
        if v(i) < vh_max && mode ~= "coast"
            mode = "drive";
            F = T_out / r_t;
        elseif v(i) >= vh_max
            mode="coast";
            F=0;
        elseif v(i) < vh_min
            mode="drive";
            F = T_out / r_t;
        else 
            F=0;
        end

    end

    % Check if lap is complete
    if x(i) > 3825
        lap_num = lap_num + 1;
        x(i) = x(i) - 3825;
    end

    % Check if four laps has been completed (RACE HAS BEEN DONE)
    if lap_num == 5
        disp('Exiting loop because four laps have been completed!')
        Time_comp = t(i) / 60 % THIS IS THE COMPLETION TIME IN MINUTES
        break
    end

    




    if F>0.1
        j=j+1;
       if v(i)<vmax_firstgear
           %GRi=GR/0.7;
           GRi=GR/1;
       else
           GRi=GR;
       end
        MotorRPM(i)=((v(i)*60)*GRi/(2*3.14159*r_t));
        MotorTorque(i)=(F/Drivetrain_eff)*r_t/GRi;
    else
        MotorRPM(i)=0;
        MotorTorque(i)=0;
    end
    
    motor_out=motor_efficiency_from_rpm_Tload_battV(MotorTorque(i),MotorRPM(i),V );
    
    motoreff=motor_out.efficiency;
    MotorEff(i)=motoreff;
    % if motoreff<0.1
    %     motoreff=0.1;
    % end
    %F is wheel force
    %F_motor = F / (T_eff);
    %^this was used to test if very low efficiencies at startup were
    %throwing off the simulation
        

    a(i) = (F - F_rr(i) - F_drag(i)) / m_eq;
    % protect against tiny negative accelerations causing small negative speeds
    v(i+1) = max(0, v(i) + a(i) * dt);

    % position update (explicit Euler)

    x(i+1) = x(i) + v(i) * dt;

    % inertia force and power
    F_inertia(i) = m_eq * a(i);
    P_inertia(i) = F_inertia(i) * v(i);

    % total required force and power (this uses the old F value b/c of
    % mechancical power output from motor, not onto the ground)
    %F_req(i) = F_motor;
    %P_x(i) = F_req(i) * v(i);
    P_x(i) = motor_out.P_elec/MotorControl_eff;
end
EffMotorTot=sum(MotorEff)/j;
figure
histogram(MotorEff, 'BinLimits', [0 1], 'Normalization', 'count') %This shows the 
%distribution of motor efficiency to see how good your driving strategy is.
%The large bar at 0 is fine bc it is just showing the points when the motor
%is off and the car is coasting.
xlabel('Efficiency Value')
ylabel('Count (# of 0.1s datapoints)')
title('Motor Efficiency Distribution')
grid on
% Energy used (integrate positive power only)
Energy_kWh = trapz(t, max(P_x, 0)) / (3600 * 1000); % [kWh]

% Convert displacement to miles correctly and compute efficiency mi/kWh
m_to_mi = 1/1609.344;
distance_miles = 15304 * m_to_mi;
if Energy_kWh > 0
    eff_mi_per_kWh = distance_miles / Energy_kWh;
else
    eff_mi_per_kWh = NaN;
end
x(end) = 15304;
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

%% Here begins the motor model section. This first section is for the mapping. 
%ChatGPT was very useful in helping write this code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Doms code below

% need arrays of torque (Nm) and speed (rpm): get using Jordans force (N) and velocity
% (m/s)
%Note: when doubling input voltage for a given motor, no load speed *2 and
%I0 /2
%MotorRPM=(v/r_t)*GR*9.5493;
% Motor data (30 V column values other than supply)
V_max = 60;        % max supply voltage (V)
R  = 0.275;         % terminal resistance (ohm)
kt = 0.136*0.9;        % torque constant (Nm/A) adjusted for sinusoidal
ks=70.2;          %speed constant rpm/V
ke = 1/(ks*2*3.14/60);        % back-EMF constant (V/(rad/s))
I0 = 0.493/2;        % no-load current (A)
w0_rpm = 2080*2;     % no-load speed (rpm)
T_stall=10; %Stall torque Nm

% Derived no-load values (for loss calibration)
omega0 = 2*pi*w0_rpm/60;        % rad/s
P_elec_noload = V_max * I0;     % W
P_cu_noload   = I0^2 * R;       % copper loss at no-load
P_other_noload = P_elec_noload - P_cu_noload;  % remainder (iron+windage+friction)

% Split mechanical loss into coulomb-like (torque*omega) and quadratic (windage)
frac_quad = 0.20;   % fraction of mechanical loss that is quadratic (tweakable, this is a guess)
P_quad = P_other_noload * frac_quad;
P_coulomb = P_other_noload * (1 - frac_quad);

% Coulomb-like torque (constant torque loss)
T_fric_coulomb = P_coulomb / omega0;   % Nm

% Viscous/windage torque coefficient (gives torque = B*omega, so power = B*omega^2)
B_visc = P_quad / (omega0^2);         % Nms/rad (units such that power = B*omega^2)

% Display calibrated loss pieces
% fprintf('Calibrated loss params:\n');
% fprintf('  Coulomb friction torque Tf0 = %.5f N*m\n', T_fric_coulomb);
% fprintf('  Viscous coefficient B = %.6e (N*m / (rad/s))\n', B_visc);
% fprintf('  (these reproduce no-load input power roughly)\n\n');

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



%% Motor Function: outputs key performance and efficiency numbers
function out = motor_efficiency_from_rpm_Tload_battV(T_load, speed_rpm, V_batt)
% Computes motor efficiency at a kinematic operating point (rpm, torque)
% using battery voltage. Internally computes the required PWM duty cycle
% and effective terminal voltage so results match the 3D efficiency map.

%% Motor parameters (same as in the mapping section)
R  = 0.275;
kt = 0.136*0.9; %Maxon has a sheet that shows that when using sinusoidal control kt is 0.9 of datasheet value
ks = 70.2;
ke = 1/(ks*2*pi/60);
I0 = 0.490/2;
w0_rpm = 2080*2;

%% Derived no-load quantities
omega0 = 2*pi*w0_rpm/60;
P_elec_nl = 60 * I0;
P_cu_nl   = I0^2 * R;
P_other   = P_elec_nl - P_cu_nl;

% Loss split (same as 3D map). This is a guess and can be changed.
frac_quad = 0.20; %This is a guess and can be changed.
P_quad = P_other * frac_quad;
P_coulomb = P_other * (1 - frac_quad);

T_fric = P_coulomb / omega0;
B_visc = P_quad / (omega0^2);

%% Convert speed to rad/s
omega = speed_rpm * 2*pi/60;

%% Electromagnetic torque required
T_e = T_load + T_fric + B_visc * omega;

%% Current
I = T_e / kt;

%% Required effective terminal voltage (PWM-averaged)
V_eff = I*R + ke*omega;

%% Duty cycle
D = V_eff / V_batt;

%% Check physical realizability
physically_possible = (D <= 1 + 1e-6);

%% Electrical & mechanical power
Pelec = V_eff * I;
Pmech = T_load * omega;

%% Efficiency
if Pelec <= 1e-9
    eff = 0;
else
    eff = max(0, Pmech / Pelec);
end

%% Package outputs. 
% %Not all of these are used, but they can be useful for troubleshooting by adding a line to see what any of them are if you need to
out.efficiency        = eff;
out.eff_percent       = eff*100;
out.current           = I;
out.P_mech            = Pmech;
out.P_elec            = Pelec;
out.omega             = omega;
out.speed_rpm         = speed_rpm;
out.T_electromag      = T_e;
out.T_load            = T_load;
out.T_coulomb         = T_fric;
out.T_viscous         = B_visc * omega;
out.back_EMF          = ke * omega;
out.V_eff             = V_eff;
out.V_batt            = V_batt;
out.duty_cycle        = D;
out.physically_possible = physically_possible;

end


%% End of motor function

%% Plot the 3D surface for motor efficiency (2D with color contours for 3rd dim)
f1=figure('Position',[100 100 900 600]);
%surf(Tv, speed_rpm_grid, eff_grid*100, 'EdgeColor','none');
contourf(Tv, speed_rpm_grid, eff_grid*100);
xlabel('Torque (Nm)');
ylabel('Speed (rpm)');
zlabel('Efficiency (%)');
title('Efficiency Map (%)');
colormap turbo; colorbar;
%view(30, -45);
grid on;
hold on
figure(f1)
plot(MotorTorque,MotorRPM,  '.', ...
     'Color',[0 1 1],'MarkerSize',9); %here the simulated data over the entire
%race run is plotted ontop of the map to see where all the points fall. Fig
%1 shows this as a histogram for efficiency. These points show up as the
%pink line on the graph



% --- dummy plots for legend only ---
h1 = plot(nan,nan,'.','Color',[0 1 1],'MarkerSize',15);
h2 = plot(nan,nan,'.','Color',[1 1 0.2],'MarkerSize',15);
h3 = plot(nan,nan,'.','Color',[1 0.1 1],'MarkerSize',15);

lgd=legend([h1 h2 h3], ...
       {'Initial Accel (Low Gear)', ...
        'Initial Accel (High Gear)', ...
        'Hyper Accel'}, ...
       'Location','best');
lgd.Color     = 'black';   % background
lgd.TextColor = 'white';   % readable text
lgd.EdgeColor = 'white';   % border (optional)
hold off


%% Plot the 3D surface for power
f2=figure('Position',[100 100 900 600]);
%surf(Tv, speed_rpm_grid, eff_grid*100, 'EdgeColor','none');
contourf(Tv(:,1:100), speed_rpm_grid(:,1:100), Pmech_grid(:,1:100));
xlabel('Torque (Nm)');
ylabel('Speed (rpm)');
zlabel('Mech Power ');
title('Power Map (W)');
colormap turbo; colorbar;
%view(30, -45);
grid on;
hold on
figure(f2)
plot(MotorTorque,MotorRPM,  '.', ...
     'Color',[0 1 1],'MarkerSize',9);



% --- dummy plots for legend only ---
h1 = plot(nan,nan,'.','Color',[0 1 1],'MarkerSize',15);
h2 = plot(nan,nan,'.','Color',[1 1 0.2],'MarkerSize',15);
h3 = plot(nan,nan,'.','Color',[1 0.1 1],'MarkerSize',15);

lgd=legend([h1 h2 h3], ...
       {'Initial Accel (Low Gear)', ...
        'Initial Accel (High Gear)', ...
        'Hyper Accel'}, ...
       'Location','best');
lgd.Color     = 'black';   % background
lgd.TextColor = 'white';   % readable text
lgd.EdgeColor = 'white';   % border (optional)
hold off
%% Find global maximum efficiency and where it occurs
%useful for seeing at what torque and rpms the motor is best at
[eta_max, idx] = max(eff_grid(:));
[i_row, i_col] = ind2sub(size(eff_grid), idx);
T_at_max = Tv(idx);
V_at_max = Vv(idx);
speed_at_max = speed_rpm_grid(idx);

fprintf('\nGlobal max efficiency = %.2f %%\n', eta_max*100);
fprintf('  Occurs at: Torque = %.3f N*m, Speed = %.1f rpm, Supply V = %.1f V\n', ...
    T_at_max, speed_at_max, V_at_max);
