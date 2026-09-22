%motorgearstuff
clear;clc;
% G=1:20;
% for i=1:20
%     E(i)=kWh(G(i));
% end
% plot(G,E)
% function output=kWh(G_RATIO)
%%nominal:
nTorque=1.49; %Nm
nSpeed=1520; %rpm
nCurrent=5.99; %A
nVoltage=48;
nW=nSpeed* 2 * 3.14 / 60;
%noload:
nl_W=1960 * 2 * 3.14 / 60;
nl_I=0.283;

%stall:
st_T=9.49;
st_I=57.8;



%goals--total power output? Power loss?
%efficiency=TW/AV

m=85; %kg
a_start=0.445; % m/s2, to get from 0 to 20 mph in 20s, just a guess

Tc=0.231; %Nm/A
Sc=41.3; %rpm/V
Wc=Sc*(2*3.14/60); %rad/sV


sCar=8.9; %m/s 20 mph
cPower=100; %W --mostly a guess
 
r_t = .2413; %outer tire rad,m
gRatio=10; %i think
%gRatio=G_RATIO;
%get W 
W_wheel = sCar/(2*3.14*r_t);
W_motor=W_wheel*gRatio;
if W_motor>nl_W
    W_motor=nl_W;
    disp("too large G")
end

%get T
%rough force calcs
speed=sCar;
Pneeded=speed*(cPower/sCar); %right now this just says power needed=100W

F_road=Pneeded/speed;
T_motor=(F_road*r_t)/gRatio;
%^both for cruise

F_acc=m*a_start + (F_road/3); %extremely roughly accounting for drag, friction, etc
T_motor_acc=(F_acc*r_t)/(gRatio/0.7); %assuming using lower gear when starting

%eff=(T*W)/(A*V); T*W needs to be mechPower=100W. AV? Depends on motor
%Eff eqn

%% From chat:\
% 1) electrical resistance from stall point (approx)
R = nVoltage / st_I;

% 2) motor back-EMF constant (Ke) from nominal point, then torque constant Kt
Ke = (nVoltage - nCurrent*R) / nW;  % V/(rad/s)
Kt = 1/Ke;                   % Nm/A  (SI)


%% 



T=linspace(0,st_T,100);

eff=@(T)((T * nl_W * (1 - T/st_T)) / (nVoltage * (nl_I + T/Tc)));

for i=1:length(T)
    effT(i)=eff(T(i));
end

effTmotor=eff(T_motor);
effTmotoracc=eff(T_motor_acc);

plot(T,effT)
hold on
plot(T_motor,eff(T_motor),'*')

plot(T_motor_acc,eff(T_motor_acc),'*')

hold off

effTotal=0; %in mi/kWh

% Wh_acc_ideal=(nl_I + T_motor_acc/Kt)*nVoltage*0.00555556;
% Wh_cruise_ideal=(nl_I+ T_motor/Kt)*nVoltage*0.4833;

Wh_acc_ideal=T_motor_acc*(W_motor/2)*0.00555556;%*avg acc W
Wh_cruise_ideal=T_motor*W_motor*0.4833;

Wh_ideal=Wh_cruise_ideal+Wh_acc_ideal;
effTotIdeal=10/(Wh_ideal/1000);

Wh_acc_real=Wh_acc_ideal/effTmotoracc;
Wh_cruise_real=Wh_cruise_ideal/effTmotor;
Wh_real=Wh_acc_real+Wh_cruise_real;
effTotReal=10/(Wh_real/1000);
output=effTotReal


%end