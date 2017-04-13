function [ fitness ] = Fitness( DeltaVLeft, ecc_check, Ra_check, TargetOrbit )
%User defined fitness calculation to determine the performance of each set
%of genes

fitness = 0;
OrbitAchieved = 0;

if (DeltaVLeft >= 0) && (Ra_check >= TargetOrbit) 
    OrbitAchieved = 1;
end
Scale = 1/1000;
Weight_ecc = 500;
Weight_DeltaV = 1;
Fitness_E = Scale*(Weight_ecc*(1-ecc_check))^2;
Fitness_D = Scale*(Weight_DeltaV*DeltaVLeft)^2;
    
fitness = OrbitAchieved*(Fitness_E + Fitness_D);

%Percent_E = Fitness_E/fitness;
%Percent_D = Fitness_D/fitness;