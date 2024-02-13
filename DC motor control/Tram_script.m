clc
clear all

%% DC machine parameters

%electrical parameters
%armature
Vdc = 600;
Van =Vdc;
Pmec = 21e3; %W
Pmec_eq = Pmec*4;
efficiency = 0.9;
Pel_eq = Pmec_eq/efficiency;
Ian = Pel_eq/Van; % Rotor nominal current
tau_a = 10e-3; %equivalent tao a
%Ra*Ian^2 = (1-efficiency)*Van*Ian
Ra = (1-efficiency)*Van/Ian; % Armature resistance [Ohm]
La = tau_a*Ra; % Armature inductance [H]
%excitation
Ven = 60;
Ien = 5;
tau_e = 0.1;
Re = Ven/Ien;
Le = Re*tau_e;
E = Van-Ra*Ian;

%mechanical parameters
speed_max = 42; %km/h
Or = 970; %rpm of motor
Ob = Or;
rated_speed_motor = Or*(2*pi)/60; %rad/s
d = 680e-3; %diameter in m
rho = 13/74; %gearbox ratio
speed_tram = rated_speed_motor*d/2*rho*3.6; %km/h
Tn = Pmec_eq/rated_speed_motor; % Nominal torque provided by the machine
K = Tn/(Ian*Ien); % Machine torque constant
m_noload = 15e3; %kg
n_passengers_max = 130;
average_weight_person = 70;
m_full_load = m_noload + n_passengers_max*average_weight_person;

J_eq = m_full_load*((speed_tram/3.6)^2)/(rated_speed_motor^2);
Tfriction = Tn/10; %at rated speed
distance = [0,1,1.001,3,3.001,4,4.001,6,6.001,8,8.001,9,9.001,10];
speed_ref = [speed_tram/2,speed_tram/2, speed_tram,speed_tram,speed_tram,speed_tram,speed_max,speed_max,speed_tram,speed_tram,speed_tram,speed_tram,speed_tram/2,speed_tram/2];
slope = [0,0,0,0,5,5,0,0,0,0,-5,-5,0,0];
theta = atan(slope./100);
Tslope = m_full_load*9.81*sin(theta)*d/2*rho;
Tload = Tslope + Tfriction;
B = Tfriction/rated_speed_motor; % friction coefficient < Or
tau_mec=J_eq/B;
speed_weak = [0, speed_tram/4,speed_tram/2, speed_tram,speed_tram*1.5, speed_tram*1.8,speed_tram*1.9 speed_tram*2];
T_weakening = [Tn, Tn, Tn, Tn, Tn/1.5, Tn/2, Tn/2.5, Tn/5];
%%  PI controller design parameters
s=tf('s');
%Gi
tau_a_desired=tau_a/10;
wc_a=2*pi/tau_a_desired;
%Ge
tau_e_desired=tau_e/10;
wc_e=2*pi/tau_e_desired;
%GO
tau_mec_desired=tau_mec/100;
wc_O=2*pi/tau_mec_desired;
%tf
Gi = 1/(Ra+La*s);
Ge = 1/(Re+Le*s);
GO = 1/(B+J_eq*s);

%% Zero Pole cancellation (90 phase margin)
% %PI parameters ia
% kp_a=wc_a*La;
% ki_a=wc_a*Ra;
% Regi=kp_a+ki_a/s
% Ti_a=kp_a/ki_a;
% %tf open loop
% Li=Regi*Gi;
% %tf close loop
% Fi=Li/(1+Li);
% % figure
% % bode(Li)
% % figure
% % bode(Fi)
% %PI parameters ie
% kp_e=wc_e*Le;
% ki_e=wc_e*Re;
% Rege=kp_e+ki_e/s
% Ti_e=kp_e/ki_e;
% %tf open loop
% L_e=Rege*Ge;
% %tf close loop
% Fe=L_e/(1+L_e);
% % figure
% % bode(L_e)
% % figure
% % bode(Fe)
% %PI parameters speed
% kp_O=wc_O*J_eq;
% ki_O=wc_O*B;
% RegO=kp_O+ki_O/s
% Ti_O=kp_O/ki_O;
% %tf open loop
% LO=RegO*GO;
% %tf close loop
% FO=LO/(1+LO);
% % figure
% % bode(LO)
% % figure
% % bode(FO)

%% Pidtool
%otherwise use pidtool
phase_m=90;
%pidtool(Gi)
opt=pidtuneOptions('PhaseMargin', phase_m);
par_regi=pidtune(Gi,'PI',wc_a,opt);

ki_a=par_regi.Ki;
kp_a=par_regi.Kp;

Regi=kp_a+ki_a/s
Ti_a=kp_a/ki_a;
%tf open loop
Li=Regi*Gi;
%tf close loop
Fi=Li/(1+Li);
% figure
% bode(Li);
% figure
% bode(Fi);
% figure
% margin(Li);

%pidtool(Ge)
opt=pidtuneOptions('PhaseMargin', phase_m);
par_rege=pidtune(Ge,'PI',wc_a,opt);

ki_e=par_rege.Ki;
kp_e=par_rege.Kp;

Rege=kp_e+ki_e/s
Ti_e=kp_e/ki_e;
%tf open loop
L_e=Rege*Ge;
%tf close loop
Fe=L_e/(1+L_e);
% figure
% bode(L_e);
% figure
% bode(Fe);
% figure
% margin(L_e);


%pidtool(GO)
par_reg_speed=pidtune(GO,'PI',wc_O,opt);
ki_O=par_reg_speed.Ki;
kp_O=par_reg_speed.Kp;

RegO=kp_O+ki_O/s
Ti_O=kp_O/ki_O;
%tf open loop
LO=RegO*GO;
%tf close loop
FO=LO/(1+LO);
% figure
% bode(LO);
% figure
% bode(FO);
% figure
% margin(LO);
%% saturation
SatUp_Va = Van; % [A]
SatLow_Va = -Van;
SatUp_T = Tn*3; % [V]
SatLow_T = -Tn*3;
SatUp_Ve = Ven*1.1;
SatLow_Ve = 0;
