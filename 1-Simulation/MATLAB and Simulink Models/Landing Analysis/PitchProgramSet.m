function [PitchProgram] = PitchProgramSet( GravityTurn, PitchOverAlt, PitchOverAngle)
%Defined pitch program
x = linspace(0,200000,1000)';
p1 = 0.9216;
p2 = 0.3541;
p3 = 0;

if GravityTurn == 1
    for i=1:1000
        PitchProgram(i,1) = x(i);
        if x(i) <= PitchOverAlt
            PitchProgram(i,2) = PitchOverAngle*(x(i)/PitchOverAlt);
        else
            PitchProgram(i,2) = PitchOverAngle;
        end
    end
else
    for i=1:1000
        PitchProgram(i,1) = x(i);
        PitchProgram(i,2) = K1*(p1*(x(i)*K2)^p2)+p3;
    end
end
