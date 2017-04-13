Throttle = Best(1);
PitchOverAlt = Best(2);
PitchOverAngle = Best(3);
GravityTurn = 1;
PitchProgram = PitchProgramSet(GravityTurn, PitchOverAlt, PitchOverAngle);
SSIGN = Best(4);
PitchSwitch = Best(5);
evalc('sim(''NewOrbitSimulation'')');
PlotPlanetRel;