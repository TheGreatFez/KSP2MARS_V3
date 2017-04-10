function [P,rho, a0]=AtmosphericMetric(h)
%This file with caluclate the temperature and altitude at a specific
%altitude above the Earth's surface

%http://www.grc.nasa.gov/WWW/k-12/airplane/atmos.html

if h < 7000 
    T=-31 - .000998*h + 273.1; 
    P=.699*exp(-.00009*h); 
    rho=(P)/(.1921*(T)); 
    
else 
    T=-23.4 - .00222*h + 273.1; 
    P=.699*exp(-.00009*h); 
    rho=(P)/(.1921*(T));
    
end

gamma=1.4;
R=287;

a0=sqrt(gamma*R*T);

end