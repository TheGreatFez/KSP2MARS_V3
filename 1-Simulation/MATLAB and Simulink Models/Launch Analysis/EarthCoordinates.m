[a b] = size(PlanetRelative);

max_angle = 2*pi;

plotTheta = linspace(0,max_angle,1000)';

for i=1:1000
    plotXY(i,1) = R_planet*sin(plotTheta(i));
    plotXY(i,2) = R_planet*cos(plotTheta(i));
    plotATMOXY(i,1) = (R_planet+AtmoAlt_planet)*sin(plotTheta(i));
    plotATMOXY(i,2) = (R_planet+AtmoAlt_planet)*cos(plotTheta(i));
end