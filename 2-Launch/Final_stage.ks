declare parameter ap_setpoint. //Apoapsis setpoint
declare parameter headng. //Horizon Bearing

lock vs to ship:verticalspeed. //vertical speed

lock throttle to 1.

set setpoint to ap_setpoint*1000. //setpoint for PID

//Initial gains
set Kp to 0.005379.
set Ki to 0.0001355.
set Kd to 0.03819/8.214.

//PIDLoop definitions
set PitchPID to PIDLOOP(Kp, Ki, Kd).
set PitchPID:minoutput to -30.
set PitchPID:maxoutput to 30.

set start_time to time:seconds.

set steertag to heading(headng, 0).//steering variable to feed to lock steering

lock steering to steertag.

until vs<0{
      set error to orbit:apoapsis-setpoint.//Error to feed to PID

      set cur_time to time:seconds - start_time.

      set pitch to PitchPID:update(cur_time, error).//PID output

      set steertag to heading(headng, pitch).//update steering variable

      clearscreen.

      print "Apoapsis: " + orbit:apoapsis + "m" at (0,1).

      print "Pitch: " + pitch + "deg" at (0,3).
      
      wait 0.
}


until (setpoint-orbit:periapsis)<10000{

      //Calculating horizontal orbital speed
      set vorbh to (ship:velocity:orbit-ship:velocity:surface):mag+ship:groundspeed.
      //Calculating centrifugal acceleration
      set centrifugal_acc to vorbh*vorbh/(orbit:body:radius+ship:altitude).

      //gravitational acceleration
      set grav_acc to orbit:body:mu/((orbit:body:radius+ship:altitude)^2).

      //effective acceleration
      set effec_acc to grav_acc-centrifugal_acc.

      //Setting pitch
      set pitch to MAX(-30, MIN(30,arcsin(effec_acc/(ship:availablethrust/ship:mass)))).

      //compensating for lag between 2 until loops
      if vs<-1{
      	 set pitch to MAX(-30, MIN(60, pitch-vs*0.2)).
      }

      set steertag to heading(headng, pitch).//Update steering variable

      clearscreen.

      print "Altitude: " + ship:altitude + "m" at (0,1).

      print "Pitch: " + pitch + "deg" at (0,3).

      wait 0.
}

lock throttle to 0.
unlock steering.