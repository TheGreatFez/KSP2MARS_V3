//###################################################################
//
//   Very Very WIP, nowhere near finished
//
//###################################################################

@lazyglobal off.

run steerlib.

global landing_site is latlng(0, 153.1).

declare function create_deorbit_node {
	local target_periapsis is 20000.
	local fudge_factor is 60.
	local ship_lng is ship:geoposition:lng.
	local burn_point_lng is landing_site:lng + 180 + fudge_factor.
	print ship_lng.
	print burn_point_lng.
	local angular_freq is 360 / ship:orbit:period.
	local time_to_node is (burn_point_lng - ship_lng) / angular_freq.
	set time_to_node to abs(time_to_node).
	local r2 is altitude + ship:body:radius.
	local r1 is target_periapsis + ship:body:radius.
	local delta_v is -1 * sqrt(ship:body:mu/r2)*(1-sqrt(2*r1/(r1+r2))).
	local n is node(time:seconds + time_to_node,0,0,delta_v).
	add n.	
}

declare function ballast_control {
	parameter a_fraction.
	local a_tank is ship:partstagged("ballast_a")[0].
	local b_tank is ship:partstagged("ballast_b")[0].
	local a_1 is a_tank:resources[0]:amount. wait 0.01. local a_2 is a_tank:resources[0]:amount.
	if a_1 = a_2 {
		local total_ballast_amount is a_tank:resources[0]:amount + b_tank:resources[0]:amount.
		local required_a_amount is a_fraction * total_ballast_amount.
		local err is required_a_amount - a_tank:resources[0]:amount.
		if err > 0.01 {
			local transferer is transfer("LeadBallast", b_tank, a_tank, abs(err)).
			set transferer:active to true.
		} else if err < 0.01 {
			local transferer is transfer("LeadBallast", a_tank, b_tank, abs(err)).
			set transferer:active to true.
		}	
	}
}

declare function entry {
	local p is 20.
	local roll_mod is 0.
	local roll_pid_k is list(0.05, 0.001, 0.1).
	local yaw_pid_k is list(0.05, 0.00001, 0.1).
	local pitch_pid_k is list(0.3, 0.0001 , 0.1).
	global roll_pid is pidloop(roll_pid_k[0], roll_pid_k[1], roll_pid_k[2], -1, 1).
	global yaw_pid is pidloop(yaw_pid_k[0], yaw_pid_k[1], yaw_pid_k[2], -1, 1).
	set roll_pid:setpoint to 0.
	set yaw_pid:setpoint to 270.

	
	set ship:control:pitch to 0.
	set ship:control:yaw to 0.
	set ship:control:roll to 0.
	
	global pitch_pid is pidloop(pitch_pid_k[0], pitch_pid_k[1], pitch_pid_k[2], 0, 60).
	
	ballast_control(1).
	local runmode is "lifting".
	until ship:groundspeed < 20 {
		if runmode = "lifting" {
			set ship:control:roll to roll_pid:update(time:seconds, roll_for()).
			set ship:control:yaw to yaw_pid:update(time:seconds, compass_for()).
			if ship:groundspeed < 700 {
				lock throttle to 1.
				set ship:control:pitch to 0.
				set ship:control:yaw to 0.
				set ship:control:roll to 0.
				lock steering to heading(270, p).
				set pitch_pid:setpoint to 0.
				set runmode to "burning".
			}
		} else if runmode = "burning" {
			set p to pitch_pid:update(time:seconds, ship:verticalspeed).
			local m is 0.5 / 60.
			local c is 1 - (m * 60).
			ballast_control(c + (m * p)).
			print c + (m * p).
			
		}
		wait 0.05.
		clearscreen.
	}
	ballast_control(0.5).
}


declare function final_descent {
	
}

//create_deorbit_node().

entry().