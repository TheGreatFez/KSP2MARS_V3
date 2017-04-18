Throttle = children(gen,1);
PitchOverAlt = children(gen,2);
PitchOverAngle = children(gen,3);
GravityTurn = 1;
PitchProgram = PitchProgramSet(GravityTurn, PitchOverAlt, PitchOverAngle);
SSIGN = children(gen,4);
PitchSwitch = children(gen,5);
evalc('sim(''NewOrbitSimulation'')');