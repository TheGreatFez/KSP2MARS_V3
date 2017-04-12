clearscreen.
//looks throught all engines and returns the engines in the specified stage.
//[Int] => list-of-engines
function act_eng {
Parameter s is stage:number.

	local res is list().
	list engines in all_eng.
	local length is all_eng:length-1.
	if length = 0 { return all_eng.}
	else {
		from {local x is length.} until x = 0 step {set x to x-1.} do {
			if all_eng[x]:stage = s{
				res:add(all_eng[x]).
			}
		}
		return res.
	}
}
//evaluates the Propellant status of an engine
// => Boolean
function node_ullage {

	for eng in act_eng {
		return eng[0]:Getmodule("ModuleEngineRF"):Getfield:("Propellant") = "very Stable".
	}
}
//calculates the apprx. burntime for a maneuvernodes delta-v
//deltav:[double] => seconds:[double]
function mnvr_t {
Parameter dV.

	local leng_mnvr is act_eng.
	local length is leng_mnvr:length -1. //lets assume we have the same kind of engines in our current stage.
	local thrust is leng_mnvr[length]:Maxthrust *1e3.
	local mass is Ship:mass * 1e3.
	local e is constant():e.
	local isp is leng_mnvr[length]:Getmodule("ModuleEnginesRF"):Getfield("specific Impulse").
	local g is 9.80665.
	local result is (g * mass * isp * (1- e ^(-dV / (g * isp))) / thrust).

	return result / (length+1).
}
//steadily decreasing the throttle.
//deltav:[double] => throttle:[double]
function node_throttle_down {
parameter _deltav.
return ln(_deltav^_deltav)/100. //just a brainfart. If anyone wants to provide a better function feel free to do so.
}
//
//node[array] => Boolean
function node_completed {
Parameter _node.

	wait until _node:deltav:mag < 29.5.
	lock throttle to min(0.15,node_throttle_down(_node:deltav:mag) ).
	wait until _node:deltav:mag < 5.
	set throttle to 0.
	RCS on.
	set steering to ship:facing:vector.
	set ship:control:fore to 1.
	wait until _node:deltav:mag < 0.4 and vang(_node:deltav, ship:facing:vector) > 90. //TODO:still not precise enough.
	remove nextnode.
	set ship:control:fore to 0.
	RCS off.
	set proceed to true.
}
function node_east {

return vcrs(ship:up:vector,ship:north:vector).
}
//This function gets the compass heading of a given vector.
//vector[list] => heading[double]
function get_compass_hdg {
Parameter vector.

	local trig_x is vdot(ship:north:vector,vector).
	local trig_y is vdot(node_east(),vector).
	local result is arctan2(trig_y,trig_x).
	if result < 0 return 360 + result.
	else return result.
}
function node_boost {
Parameter dir, val.

	set dir to val.
	wait 0.1.
	set dir to 0.
}

//Evaluates the ships current position to the navall horizon
function roll_me {

	if vang(facing:vector,up:vector) < 0.1 { 
    return 0. }
	set raw to vang(vxcl(facing:vector,up:vector), facing:starvector).
	if vang(up:vector, facing:topvector) > 90 {
		if raw > 90 {
			return 270 - raw.
		} 
		else {
			return -90 - raw.
		}
	} 
	else {
		return raw - 90.
	}
}
//getting the roll into the right plane first. This is important because the math for yaw and pitch correction would be a disease otherwise .
//
function node_roll {

	
	lock n_roll to roll_me.
	set done to false.
	set n_roll_dir to 0.

	
	if n_roll > 0 {
		set n_roll_dir to -1.
		set ship:control:roll to n_roll_dir.
		wait 0.3.
		set ship:control:roll to 0.
	}
	else {
		set n_roll_dir to 1.
		set ship:control:roll to n_roll_dir.
		wait 0.3.
		set ship:control:roll to 0.
	}
	When n_roll >= -0.1 and n_roll <= 0.1 then {
		set ship:control:roll to -n_roll_dir.
		wait 0.3.
		set ship:control:roll to 0.
		set done to true.
		set node_roll_complete to true.
		if n_roll >= -0.1 and n_roll <= 0.1 {
			preserve.
		}
	}
	
	Until done {
		print"current roll "+n_roll at(0,5).
	}
	
//	wait until done.
	
}
//The function is adjusting the alignment - cavemenstyle. Nothing to be proud of at the moment.
//Its using the navball to get the ship pointing into the nodes direction.
//No initial rotation check yet, it would be possible to implement something with ship:angularmomentum or ship:angularvel.
//The angularvelocity has to be monitored during the maneuver to counteract the torque ... somehow.
//node[array] => 
function node_align {
Parameter _node.

//	set node_roll_complete to false.
//	node_roll().
//	wait until node_roll_complete.
set ship_yaw to 0.
	lock ship_pitch to 90-vang(up:vector,ship:facing:forevector).
	lock node_pitch to 90-vang(up:vector,_node:deltav).
	lock ship_yaw to -1*ship:bearing.
	if ship_yaw < 0 set ship_yaw to ship_yaw+360. 
	lock node_yaw to get_compass_hdg(_node:deltav).
	
	lock pitch_dif to abs(ship_pitch - node_pitch).
	lock yaw_dif to abs(ship_yaw - node_yaw).
	
	local yaw_Dir is 0. //Using these two values to save the initial input.
	local pit_dir is 0.
	RCS on.
	
	if ship_yaw > node_yaw {
		set yaw_dir to -1.
		set ship:control:yaw to yaw_dir.
		wait 0.3.
		set ship:control:yaw to 0.
	} else {
		set yaw_dir to 1.
		set ship:control:yaw to yaw_dir.
		wait 0.3.
		set ship:control:yaw to 0.
	}
	
	if ship_pitch > node_pitch {
		set pit_dir to -1.
		set ship:control:pitch to pit_dir.
		wait 0.3.
		set ship:control:pitch to 0.
	} else {
		set pit_dir to 1.
		set ship:control:pitch to pit_dir.
		wait 0.3.
		set ship:control:pitch to 0.
	}	
	
	
	Until yaw_dif < 1 and pitch_dif < 1 {
		print"Ship pitch : " + ship_pitch at(0,5).
		print"Node pitch :" +node_pitch at(0,6).
		print"Ship yaw : " + ship_yaw at(0,7).
		print"Node yaw :" +node_yaw at(0,8).
		print"dif pitch :" +pitch_dif at(0,9).
		print"dif yaw :" +yaw_dif at(0,10).
		print"activating Thrusters"at(0,11).
		if yaw_dif <= 1 {
			node_boost(ship:control:yaw, -yaw_dir).
		}
		if pitch_dif <= 1 {
			node_boost(ship:control:pitch, -pit_dir).
		}
		wait 0.01.
	}
	
	When vang(ship:facing:forevector,_node:deltav) < 0.5 then {
	set sasmode to "maneuver".
	SAS on.
	wait 5.
	SAS off.
	RCS off.
	}
}
//main function
//node[array] => boolean
function node_exec {
Parameter _node.

	node_align(_node).
	wait until mnvr_t(_node:deltav)+3 > _node:eta - mnvr_t(_node:deltav)+3.
	RCS on.
	set pilot:control:fore to 1.
	local proceed is false.
	when node_ullage then {
		set proceed to true.
	}
	wait until proceed.
	set pilot:control:fore to 0.
	set proceed to false.
	RCS off.
	set throttle to 1.
	node_completed(_node).
	wait until proceed.
	return true.
}

node_align(nextnode).
wait until 1 < 0.