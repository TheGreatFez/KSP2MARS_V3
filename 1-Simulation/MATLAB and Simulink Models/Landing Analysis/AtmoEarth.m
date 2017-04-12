function [P,rho, a0]=AtmoEarth(h)
%This file with caluclate the temperature and altitude at a specific
%altitude above the Earth's surface

%http://www.grc.nasa.gov/WWW/k-12/airplane/atmos.html

if h < 11000 
    T=15.04 - .00649*h + 273.1; 
    P=101.29*((T/(288.08))^5.256); 
    rho=(P)/(.2869*(T)); 
    
elseif 11000 < h < 25000 
    T=-56.46 + 273.1; 
    P=22.65*exp(1.73-.000157*h);  
    rho=(P)/(.2869*(T)); 
    
elseif h > 25000 
    T = -131.21 + .00299*h + 273.1;
    P = 2.488*(T/216.6)^(-11.388);
    rho=(P)/(.2869*(T));
end

gamma=1.4;
R=287;

a0=sqrt(gamma*R*T);

end