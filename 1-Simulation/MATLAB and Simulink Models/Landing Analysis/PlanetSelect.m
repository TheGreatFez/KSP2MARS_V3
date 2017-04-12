Planet = 'Mars'; % Choice between Earth and Mars
%load 'DragData_3manCap.mat'
load 'DragData_TwoStageTestCraft.mat'
load 'EarthAtmoDataRSS.mat'
load 'Mars Atmo Data.mat'
load 'KerbinAtmoData.mat'
%load 'ThrustMassData.mat'
%load 'TwoStageTestCraft.mat'
load 'MEM_Lander.mat'
Max = 80000;
LastStage = size(ThrustMassData,1);
g0 = 9.80665; % m/s^2 standard gravity
q0 = 1.1577; % kg/m^3
Min_Throttle = 1.1*g0*ThrustMassData(1,1)/(1000*ThrustMassData(1,3));
Max_Throttle = min(1,3*g0*ThrustMassData(1,1)/(1000*ThrustMassData(1,3)));
K1 = 1.5;
K2 = .825;

% REQUIRED INPUTS AND THE DEFAULT VALUES

ThrustEnable = 0;       % 0 = Thrust Disabled, 1 = Thrust Enabled

Aero_On = 1;            % 0 = All Aerodynamics Disabled, 1 = All Aerodynamics Enabled
Drag_On = 1;        
Constant_Drag_On = 1;
Lift_On = 0;
Constant_Lift_On = 1;
RefSurfArea = 3.154;     % m^2
Cd = 50;                 % Value for Constant Coefficient of Drag
Cl = 2;                 % Value for Constant Coefficient of Lift

GravityTurn = 1;
PitchOverAlt = 1000;
PitchOverAngle = 7.5;
Throttle = 1.5*g0*ThrustMassData(1,1)/(1000*ThrustMassData(1,3));
Coasting_Parameter = 306.412; % The time to apoapsis that the ship must be under to be able to stage
PitchSwitch = .5;

PitchProgram = PitchProgramSet(GravityTurn,PitchOverAlt,PitchOverAngle);

[R_planet,GndAlt_planet,Omega_planet,AtmoAlt_planet,GM_planet,Planet] = PlanetParameters(Planet);

if strcmp('Earth',Planet) == 1
    AtmoData = EarthAtmoDataRSS;
elseif strcmp('Mars',Planet) == 1
    AtmoData = MarsAtmoDataRSS;
elseif strcmp('Kerbin',Planet) == 1
    AtmoData = KerbinAtmoData;
else
    AtmoData = 0;   
end

TargetOrbit = 100000-GndAlt_planet; % m

entry_angle = 6.5; % degrees
entry_speed = 11000; % m/s

Px = 0; % m
Py = R_planet+AtmoAlt_planet; % m

X = [Px;Py;0];
Y = [0;0;Omega_planet];
Z = cross(X,Y);

Vx = Z(1); % m/s
Vy = Z(2); % m/s

CraftInitialV = [Vx;Vy];
CraftInitialP = [Px;Py];
%Mass = 5600; % kg
%WetMass = 7100; % kg
%DryMass = 3100; % kg 
