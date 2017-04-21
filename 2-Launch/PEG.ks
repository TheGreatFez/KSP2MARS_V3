local parameter azi is 45.
local parameter ApSetpoint is 180.

set planet_radius to orbit:body:radius.

set gravity to orbit:body:mu/(planet_radius^2).

wait 5.

list engines in pitch_eng.

for i in pitch_eng{
    set pitch_isp to i:visp.
}

wait until ship:availablethrust>1.

declare function Time_to_go_determinant{

	declare parameter tau, time_to_go.

	local vertical_speed is ship:verticalspeed.
	local horizontal_speed is ship:groundspeed.
	local exhaust_velocity is pitch_isp*gravity.
	local terminal_height is ApSetpoint*1000+planet_radius.
	local terminal_horizontal_speed is sqrt(orbit:body:mu/terminal_height).
	local terminal_vertical_speed is 0.

	local delta_v is sqrt((terminal_horizontal_speed-horizontal_speed)^2+(terminal_vertical_speed-vertical_speed + gravity*time_to_go)^2).

	return tau*(1- constant:e^(-delta_v/exhaust_velocity))-time_to_go.

}

declare function pitch_calc{

    local vertical_speed is ship:verticalspeed.
    local horizontal_speed is ship:groundspeed.
    local exhaust_velocity is pitch_isp*gravity.
    local terminal_height is ApSetpoint*1000+planet_radius.
    local terminal_horizontal_speed is sqrt(orbit:body:mu/terminal_height).
    local terminal_vertical_speed is 0.

    set tau to ship:mass*exhaust_velocity/ship:availablethrust.

    set guess1 to 0.
    set guess2 to 1.

    set start_time to time:seconds.

    until abs(guess2-guess1)<1e-6{
        set guess3 to guess2 - Time_to_go_determinant(tau, guess2)*(guess2-guess1)/(Time_to_go_determinant(tau, guess2)-Time_to_go_determinant(tau, guess1)).

        set guess1 to guess2.
        set guess2 to guess3.

        clearscreen.

        print "Guess: "+guess2 at (0,2).

    }

    local final_pitch to arctan((0-ship:verticalspeed+gravity*guess2)/(terminal_horizontal_speed-horizontal_speed)).

    local coeff_A1 to cos(final_pitch)*ln(tau/(tau-guess2)).

    local coeff_B1 to cos(final_pitch)*(tau*ln(tau/(tau-guess2)) - guess2).

    local coeff_A2 to exhaust_velocity*cos(final_pitch)*(guess2 - (tau-guess2)*ln(tau/(tau-guess2))).

    local coeff_B2 to exhaust_velocity*cos(final_pitch)*(-0.5*(guess2^2) + tau*(guess2 - (tau-guess2)*ln(tau/(tau-guess2)))).

    local coeff_C2 to (ship:altitude+planet_radius)-(terminal_height) - 0.5*gravity*guess2*guess2 + vertical_speed*guess2 + exhaust_velocity*sin(final_pitch).

    local coeff_K1 to coeff_B1*coeff_C2/(coeff_A2*coeff_B1-coeff_A1*coeff_B2).

    local coeff_K2 to coeff_A1*coeff_K1/coeff_B1.

    local Pitch_aim to final_pitch - coeff_K1 + coeff_K2*guess2.

    return Pitch_aim.

}

set steertag to heading(azi, 0).

lock steering to steertag.

until abs(orbit:periapsis-ApSetpoint)<10000{

    set pitch_aim to pitch_calc().

    set steertag to heading(azi, pitch_aim).

    clearscreen.

    print "Desired Pitch: "+pitch_aim at (0,2).

    wait 0.

}