function [ R_planet,GndAlt_planet,Omega_planet,AtmoAlt_planet,GM_planet,Planet ] = PlanetParameters( Planet )
%PLANETPARAMETERS Summary of this function goes here
%   Detailed explanation goes here

if strcmp('Earth',Planet) == 1
    GndAlt_planet = 0;
    R_planet = 6371000; %m
    Omega_planet = (2*pi)/(23.9345*60*60); %radians/sec
    AtmoAlt_planet = 130000; %m
    GM_planet = 3.986004418E+14; %m^3/s^2
elseif strcmp('Mars',Planet) == 1
    GndAlt_planet = 13667.65375; % m
    R_planet = 3389500+GndAlt_planet; % m
    Omega_planet = (2*pi)/(24.6229*60*60); %radians/sec
    AtmoAlt_planet = 130000; %m
    GM_planet = 4.2828E13; %m^3/s^2
elseif strcmp('Kerbin',Planet) == 1
    GndAlt_planet = 0; % m
    R_planet = 600000+GndAlt_planet; % m
    Omega_planet = (2*pi)/(5.983*60*60); %radians/sec
    AtmoAlt_planet = 70000; %m
    GM_planet = 3.5316E12; %m^3/s^2
else
    GndAlt_planet = 0;
    R_planet = 0;
    Omega_planet = 0;
    AtmoAlt_planet = 0;
    GM_planet =  0;
    Planet = 'Error';    
end

end

