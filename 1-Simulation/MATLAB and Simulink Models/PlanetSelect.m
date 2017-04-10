Planet = 'Kerbin'; % Choice between Earth and Mars
%load 'DragData_3manCap.mat'
load 'DragData_TwoStageTestCraft.mat'
load 'EarthAtmoDataRSS.mat'
load 'Mars Atmo Data.mat'
load 'KerbinAtmoData.mat'
%load 'ThrustMassData.mat'
load 'TwoStageTestCraft.mat'
Max = 80000;
LastStage = size(ThrustMassData,1);
g0 = 9.80665; % m/s^2 standard gravity
q0 = 1.1577; % kg/m^3
Min_Throttle = 1.1*g0*ThrustMassData(1,1)/(1000*ThrustMassData(1,3));
Max_Throttle = min(1,3*g0*ThrustMassData(1,1)/(1000*ThrustMassData(1,3)));
K1 = 1.5;
K2 = .825;


GravityTurn = 1;
PitchOverAlt = 1000;
PitchOverAngle = 7.5;
Throttle = 1.7*g0*ThrustMassData(1,1)/(1000*ThrustMassData(1,3));
SSIGN = 306.412;
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
Py = R_planet+.1; % m

X = [Px;Py;0];
Y = [0;0;Omega_planet];
Z = cross(X,Y);

Vx = Z(1); % m/s
Vy = Z(2); % m/s

CraftInitialV = [Vx;Vy];
CraftInitialP = [Px;Py];

RefSurfArea = 10.8; % m^2
Cd = 2;
%Mass = 5600; % kg
%WetMass = 7100; % kg
%DryMass = 3100; % kg 
