%% 102B Load Cases
% Number of G's and Suspension Parameters are based on Berkeley Formula
% SAE's 2023 Design. All fixed loads like number of G's are off real-time
% testing data and an extensive Vehicle Dynamics Script.
% All loads are to be applied from the right side pneumatic trails
clc; close all; clear; 

%% Input Variables
% Suspension Parameters
CGz = 12.25; %CG Height (in)
WB = 62; %Wheel Base (in) 
CW = 550; %Car Weight + Driver (lbs)
WBias = 0.54; %Weight Bias(Rear/Total)
TrW = 47; %Track Width (in)
PT = 1; %Pneumatric Trail (in) - Assumption for estimating moment on tire (obsolete)
WCz = 7.7222; %Wheel Center Height from origin (in)
FBrCR = (7.175/2)-(1.63/2); %Front Brake Rotor Radius (GP320 - 1.63in offset/2)
RBrCR = (6.85/2)-(1.1/2); %Rear Brake Rotor Radius (GP200 - 1.1in offset/2)

%Aero Parameters
TDF = 190; % Total Downforce (lbf)
FDFP = 0.4;  % Front Downforce Percentage Along Wheelbase
LLTD = 0.4; %Lateral Load Transfer Distribution (front/total)
BB = 0.72; %Brake Bias (front/total)

% # of Gs
CO_G = 2; % Cornering Outside (# of Gs)
CI_G = 2; % Cornering Inside (# of Gs)
Br_G = 2.2; % Braking (# of Gs)
RB_G = 1.3; % Reverse Braking (# of Gs)
Bump_G = 3; % Bump (# of Gs)

%Compound
CP_G1 = [1.5 1 0]; % [  Outer_Corner_G  Braking_G  Bump_G  ]
CP_G2 = [2 0 1]; % [  Outer_Corner_G  Braking_G  Bump_G  ]

%% Calculated Variables
RDFP = 1-FDFP; % Rear Downforce Percentage
RearDF = (RDFP*TDF)/2; % Rear DF (1/4 Car)
FrontDF = (FDFP*TDF)/2; %Front DF (1/4 Car)
NetFgLoad = CW+2*(RearDF + FrontDF); %Net Load Caused from Car Weight and DownForce
RearFg = 0.5 * CW * WBias; %Force caused from 1/2 Rear Car Weight
FrontFg = 0.5 * CW * (1-WBias); %Force caused from 1/2 Rear Car Weight

%% Rear Forces Cornering

%Force caused from rear lateral load transfer
RLLT = (CW*CGz*CO_G/TrW)*(1-LLTD);

Rear_Cornering_Outside(3) = RearFg + RearDF + RLLT; % Z-Direction Load applied from PneuT
Rear_Cornering_Outside(2) = -1*(Rear_Cornering_Outside(3)/NetFgLoad)*CW*CO_G; % Y-Direction Load applied from PneuT
Rear_Cornering_Outside(1) = 0; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Rear_Cornering_Outside(4) = 0; % Caliper Load applied from caliper center between tabs
Rear_Cornering_Outside(5) = 0; % Rotor Braking Reaction Load through Bearing in Z-Direction

Rear_Cornering_Inside(3) = RearFg + RearDF - RLLT; % Rear Cornering Inside Load in Z-Direction
Rear_Cornering_Inside(2) = (Rear_Cornering_Inside(3)/NetFgLoad)*CW*CO_G; % Y-Direction Load applied from PneuT
Rear_Cornering_Inside(1) = 0; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Rear_Cornering_Inside(4) = 0; % Caliper Load in Z-Direction
Rear_Cornering_Inside(5) = 0; % Rotor Braking Reaction Load through Bearing in Z-Direction

%% Front Forces Cornering

%Force caused from rear lateral load transfer
FLLT = (CW*CGz*CO_G/TrW)*(LLTD);

Front_Cornering_Outside(3) = FrontFg + FrontDF + FLLT; % Z-Direction Load applied from PneuT
Front_Cornering_Outside(2) = -1*(Front_Cornering_Outside(3)/NetFgLoad)*CW*CO_G; % Y-Direction Load applied from PneuT
Front_Cornering_Outside(1) = 0; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Front_Cornering_Outside(4) = 0; % Caliper Load in Z-Direction
Front_Cornering_Outside(5) = 0; % Rotor Braking Reaction Load through Bearing in Z-Direction

Front_Cornering_Inside(3) = FrontFg + FrontDF - FLLT; % Z-Direction Load applied from PneuT
Front_Cornering_Inside(2) = (Front_Cornering_Inside(3)/NetFgLoad)*CW*CO_G; % Y-Direction Load applied from PneuT
Front_Cornering_Inside(1) = 0; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Front_Cornering_Inside(4) = 0; % Caliper Load in Z-Direction
Front_Cornering_Inside(5) = 0; % Rotor Braking Reaction Load through Bearing in Z-Direction

%% Braking 
%Longitudinal Load Transfer Equation
LongLTB = 0.5*(CW*Br_G/WB*CGz);

Rear_Braking(3) = RearFg+RearDF-LongLTB; % Z-Direction Load applied from PneuT
Rear_Braking(2) = 0; % Y-Direction Load applied from PneuT
Rear_Braking(1) = 0.5*CW*Br_G*(1-BB); % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Rear_Braking(4) = -1*Rear_Braking(1)*WCz/RBrCR; % Caliper Load in Z-Direction
Rear_Braking(5) = -1*Rear_Braking(4); % Rotor Braking Reaction Load through Bearing in Z-Direction

Front_Braking(3) = FrontFg + FrontDF + LongLTB; % Z-Direction Load applied from PneuT
Front_Braking(2) = 0; % Y-Direction Load applied from PneuT
Front_Braking(1) = 0.5*CW*Br_G*BB; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Front_Braking(4) = -1*Front_Braking(1)*WCz/FBrCR; % Caliper Load in Z-Direction
Front_Braking(5) = -1*Front_Braking(4); % Rotor Braking Reaction Load through Bearing in Z-Direction

%% Reverse Braking 
LongLTRB = 0.5*(CW*RB_G/WB*CGz);

Rear_Reverse_Braking(3) = RearFg + LongLTRB; % Z-Direction Load applied from PneuT
Rear_Reverse_Braking(2) = 0; % Y-Direction Load applied from PneuT
Rear_Reverse_Braking(1) = -0.5*CW*RB_G*(1-BB); % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Rear_Reverse_Braking(4) = Rear_Reverse_Braking(1)*WCz/RBrCR; % Caliper Load in Z-Direction
Rear_Reverse_Braking(5) = -1*Rear_Reverse_Braking(4); % Rotor Braking Reaction Load through Bearing in Z-Direction

Front_Reverse_Braking(3) = FrontFg - LongLTRB; % Z-Direction Load applied from PneuT
Front_Reverse_Braking(2) = 0; % Y-Direction Load applied from PneuT
Front_Reverse_Braking(1) = -0.5*CW*RB_G*BB; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Front_Reverse_Braking(4) = Front_Reverse_Braking(1)*WCz/FBrCR; % Caliper Load in Z-Direction
Front_Reverse_Braking(5) = -1*Front_Reverse_Braking(4); % Rotor Braking Reaction Load through Bearing in Z-Direction

%% Bump

Rear_Bump(3) = RearFg * Bump_G + RearDF; % Z-Direction Load applied from PneuT
Rear_Bump(2) = 0; % Y-Direction Load applied from PneuT
Rear_Bump(1) = 0; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Rear_Bump(4) = 0; % Caliper Load in Z-Direction
Rear_Bump(5) = 0; % Rotor Braking Reaction Load through Bearing in Z-Direction

Front_Bump(3) = FrontFg * Bump_G + FrontDF; % Z-Direction Load applied from PneuT
Front_Bump(2) = 0; % Y-Direction Load applied from PneuT
Front_Bump(1) = 0; % X-Direction Load applied from Bearing center height, aligned w/ PneuT
Front_Bump(4) = 0; % Caliper Load in Z-Direction
Front_Bump(5) = 0; % Rotor Braking Reaction Load through Bearing in Z-Direction

%% Compounds

Front_Compound_1 = [CP_G1(1)/CO_G CP_G1(2)/Br_G CP_G1(3)/Bump_G]*[Front_Cornering_Outside; Front_Braking; Front_Bump];
Front_Compound_2 = [CP_G2(1)/CO_G CP_G2(2)/Br_G CP_G2(3)/Bump_G]*[Front_Cornering_Outside; Front_Braking; Front_Bump];

Rear_Compound_1 = [CP_G1(1)/CO_G CP_G1(2)/Br_G CP_G1(3)/Bump_G]*[Rear_Cornering_Outside; Rear_Braking; Rear_Bump];
Rear_Compound_2 = [CP_G2(1)/CO_G CP_G2(2)/Br_G CP_G2(3)/Bump_G]*[Rear_Cornering_Outside; Rear_Braking; Rear_Bump];

%% Organizing Data

%Creating Matrix of all Values
Load_Cases = [
Front_Cornering_Outside
Front_Cornering_Inside
Front_Braking
Front_Reverse_Braking
Front_Bump
Front_Compound_1
Front_Compound_2
[0 0 0 0 0]
Rear_Cornering_Outside
Rear_Cornering_Inside
Rear_Braking
Rear_Reverse_Braking
Rear_Bump
Rear_Compound_1
Rear_Compound_2
];

%Splitting Values based on x,y,z
PneuT_X = Load_Cases(:,1);
PneuT_Y = Load_Cases(:,2);
PneuT_Z = Load_Cases(:,3);
Caliper_Z = Load_Cases(:,4);
Rotor_Z = Load_Cases(:,5);

%Creating Table
t = table(PneuT_X,PneuT_Y,PneuT_Z, Caliper_Z, Rotor_Z);
Names = {'Front_Cornering_Outside', 'Front_Cornering_Inside', 'Front_Braking', 'Front_Reverse_Braking',...
    'Front_Bump', 'Front_Compound_1', 'Front_Compound_2', '-----------------------', 'Rear_Cornering_Outside', 'Rear_Cornering_Inside',...
    'Rear_Braking', 'Rear_Reverse_Braking', 'Rear_Bump', 'Rear_Compound_1', 'Rear_Compound_2'};

t.Properties.RowNames = Names;
display(t);
