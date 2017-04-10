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
//steadily decreasing the throttle. _deltav is an elelement of R,Q \ {0}
//deltav:[double] => throttle:[double]
function node_throttle_down {
parameter _deltav.
return ln(_deltav^_deltav)/100. //just a brainfart. If anyone wants to provide a better function feel free to send a pull.
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
	wait until _node:deltav:mag < 0.4 and vang(_node:deltav, ship:facing:vector) > 90. //TODO:still not precise. It would be a good idea to burn for x dv into the direction.
	remove nextnode.
	set ship:control:fore to 0.
	RCS off.
	set proceed to true.
}
//node[array] => yaw[Int]
function get_compass_hdg {
Parameter _node.

	local east is vcrs(ship:up:vector,ship:north:vector).
	local trig_x is vdot(ship:north:vector,_node:deltav:vector).
	local trig_y is vdot(east,_node:Deltav:vector).
	local result is arctan2(trig_y,trig_x).
	if result < 0 return 360 + result.
	else return result.
}
function node_boost {
Parameter dir, val.

	set dir to val.
	wait 0.5.
	set dir to 0.
}
//
//node[array] => Boolean
function node_align {
Parameter _node.

	local lock ship_pitch to 90-vang(up:vector,ship:facing:forevector).
	local lock node_pitch to 90-vang(up:vector,_node:deltav:vector).
	local lock ship_yaw to mod(-100*bearing,100*360)/100. //TODO: Check the statement, if not working, replace it with -bearing and unquote next. Currently its two decimalplaces long.
//	if ship_yaw < 0 set ship_yaw to ship_yaw+360. 
	local lock node_yaw to get_compass_hdg(_node).
	
	local lock pitch_dif to abs(ship_pitch - node_pitch).
	local lock yaw_dif to abs(ship_yaw - node_yaw).
	
	local yaw_Dir is 0.
	local pit_dir is 0.
	
	if ship_yaw > node_yaw {
		set yaw_dir to 1.
		node_boost(yaw, yaw_dir).
	} else {
		set yaw_dir to -1.
		node_boost(yaw, yaw_dir).
	}
	
	if ship_pitch > node_pitch {
		set pit_dir to 1.
		node_boost(pitch, pit_dir).
	} else {
		set pit_dir to -1.
		node_boost(pitch, pit_dir).
	}	
	
	Until yaw_dif < 1 and pitch_dif < 1 {
		if yaw_diff <= 1 {
			node_boost(yaw, -yaw_dir).
		}
		if pitch_dif <= 1 {
			node_boost(pitch, -pit_dir).
		}
	}
	
	wait until vang(ship:facing:forevector,_node:deltav:vector) < 1.
	set sasmode to "maneuver".
	SAS on.
	wait 5.
	SAS off.	
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