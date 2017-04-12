Vap = sqrt((GM_planet/a_check)*((1-ecc_check)/(1+ecc_check)));
Vcir = sqrt(GM_planet/(Ra_check+R_planet));

DeltaVCir = Vcir - Vap;
DeltaVLeft = g0*Isp_check*log(Mass_check/DryMass_check) - DeltaVCir;