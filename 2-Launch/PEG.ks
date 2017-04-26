//For more info check: http://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance
//and for some corrections check: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19660006073.pdf


/////////////////////////////////////
/////////Navigation Parameters///////
/////////////////////////////////////

declare parameter azi is 45.
declare parameter orbit_alt is 220.

set gravity_0 to 9.80665.

lock throttle to 1.

list engines in eng_list.
from {local i is 0.} until i = 1 step {set i to i.} do{
    for j in eng_list{
        if j:ignition=True{set i to i+1. wait 2. set PEG_isp to j:visp. break.}
    }
    wait 0.
}

set final_radius to orbit_alt*1000+orbit:body:radius.
set final_vert_vel to 0.
set final_horiz_vel to sqrt(orbit:body:mu/final_radius).

/////////////////////////////////////
////////////Guidance/////////////////
/////////////////////////////////////
run once matrix_solver.

declare function guidance{
    declare local parameter guess is tau*0.5.

    local ship_r is ship:position - orbit:body:position.

    local ship_v is orbit:velocity:orbit.

    local exhaust_vel is PEG_isp*gravity_0.

    local acceleration is (ship:availablethrust*throttle/ship:mass).

    local tau is exhaust_vel/acceleration.

    local vert_vel is vdot(ship_r:normalized,ship_v).

    local delta_v_0 is -exhaust_vel*ln(1-guess/tau).

    local delta_v_1 is delta_v_0*tau - exhaust_vel*guess.

    local ideal_distance_0 is delta_v_0*guess - delta_v_1.

    local ideal_distance_1 is ideal_distance_0*tau - 0.5*exhaust_vel*(guess^2).

    local matrix_b is list(final_vert_vel-vert_vel,final_radius-ship_r:mag-vert_vel*guess).

    local matrix_a is list(list(delta_v_0, delta_v_1),list(ideal_distance_0,ideal_distance_1)).

    local result is matrix_solver(matrix_a,matrix_b).

    local param_A to result[0].
    local param_B to result[1].

    return list(param_A, param_B).
}

/////////////////////////////////////
////////////Estimation///////////////
/////////////////////////////////////

declare function estimate{
    declare local parameter params.
    declare local parameter guess.
    declare local parameter dt is 0.

    local param_A is params[0].

    local param_B is params[1]. 

    local param_A_new is param_A + (dt*param_B).
    local param_B_new is param_B.
    local new_guess is guess-dt.

////////////Navigation basis vectors//////////////

    local ship_r is ship:position-orbit:body:position.
    local ship_v is orbit:velocity:orbit.

    local vert_vel is vdot(ship_r:normalized,ship_v).

    local angular_momentum is vcrs(ship_r,ship_v).

    local down_track_norm is vcrs(angular_momentum:normalized, ship_r:normalized).

    local horiz_vel is vdot(down_track_norm,ship_v).

    local final_angular_momentum is final_horiz_vel*final_radius.

    local delta_angular_momentum is final_angular_momentum-angular_momentum:mag.

    local angular_vel is horiz_vel/ship_r:mag.

    local final_angular_vel is final_horiz_vel/final_radius.

    local mean_ship_radius is 0.5*(final_radius+ship_r:mag).

/////////////Vehicle Performance/////////////////

    local exhaust_vel is PEG_isp*gravity_0.

    local acceleration is (ship:availablethrust*throttle/ship:mass).

    local tau is exhaust_vel/acceleration.

    local param_C is ((orbit:body:mu/(ship_r:mag^2))-(ship_r:mag*angular_vel^2))/acceleration.

    local sinPitch is param_A_new + param_C.

/////////////Estimation////////////////////////

    local final_acceleration is acceleration/(1-(new_guess/tau)).

    local final_param_C is ((orbit:body:mu/(final_radius^2)) - (final_radius*final_angular_vel^2))/final_acceleration.

    local final_sinPitch is param_A_new + param_B_new*new_guess + final_param_C.
    local sinPitch_rate is (final_sinPitch-sinPitch)/new_guess.

    local cosPitch is 1 - 0.5*(sinPitch^2).
    local cosPitch_rate is -(sinPitch*sinPitch_rate).
    local cosPitch_accel is -0.5*(sinPitch_rate^2).

    local delta_v is (delta_angular_momentum/mean_ship_radius + exhaust_vel*guess*(cosPitch_rate + cosPitch_accel*tau) + 0.5*cosPitch_accel*exhaust_vel*(guess^2))/(cosPitch + cosPitch_rate*tau + cosPitch_accel*(tau^2)).

    local guess_update is tau*(1-constant:e^(-delta_v/exhaust_vel)).

    return list(guess_update, sinPitch).
}

//////////////////////////////////////////////
/////////////////Steering/////////////////////
//////////////////////////////////////////////

set steertag to heading(azi,0).
lock steering to steertag.

set exhaust_vel to PEG_isp*gravity_0.
set acceleration to (ship:availablethrust*throttle)/ship:mass.
set tau to exhaust_vel/acceleration.

set init_guess to tau*0.5 - 1.
print init_guess.
set final_guess to init_guess + 1.

until abs(final_guess-init_guess)<0.1{

      set init_guess to final_guess.

      set params to guidance(init_guess).

      set final_guess to estimate(params,init_guess,0)[0].

      wait 0.
}

set params to guidance(final_guess).

print final_guess.

set new_guess to final_guess.

set dt to 0.01.

until new_guess<10{

      set start_time to time:seconds.
      
      set results to estimate(params,new_guess,dt).

      set new_guess to results[0].
      set desired_pitch to arcsin(results[1]).

      set params to guidance(new_guess).

      set steertag to heading(azi, desired_pitch).

      wait 0.

      set dt to time:seconds-start_time.

      clearscreen.

      print "Desired Pitch: "+desired_pitch at (0,2).

      print "Time to cut-off: "+new_guess+" secs" at (0,4).

}

wait until final_horiz_vel-orbit:velocity:orbit:mag<=1.

lock throttle to 0.