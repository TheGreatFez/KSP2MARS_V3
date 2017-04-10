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
// => Boolean
function node_ullage {

	for eng in act_eng {
		return eng[0]:Getmodule("ModuleEngineRF"):Getfield:("Propellant") = "very Stable".
	}
}
//deltav:[Int] => seconds:[Int]
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

//deltav:[Int] => throttle:[Int]
function node_throttle_down {
parameter _deltav.
return ln(_deltav^_deltav)/100. //just a brainfart. If anyone wants to provide a better function feel free to send a push.
}
//node:[list] => Boolean
function node_completed {
Parameter _node.

	wait until _node:deltav:mag < 29.5.
	lock throttle to min(0.15,node_throttle_down(_node:deltav:mag) ).
	wait until _node:deltav:mag < 5.
	set throttle to 0.
	RCS on.
	set steering to ship:facing:vector.
	set ship:control:fore to 1.
	wait until _node:deltav:mag < 0.4 and vang(_node:deltav, ship:facing:vector) > 90. //still not precise. It would be a good idea to simply burn for x dv into the direction (next iteration).
	remove nextnode.
	set ship:control:fore to 0.
	RCS off.
	return true.
}
//This will take into account the angular speed at which the ship is rotating in its next iteration. This will prevent kOS from oversteering and using too much fuel.
//right now its just a placeholder for when I have some more time to work on it.
//node:[list] =>
function node_align {
Parameter _node.

	lock steering to _node.
}
//node:list => boolean
function node_exec {
Parameter _node.

	node_align(_node).
	wait until mnvr_t(_node:deltav)+3 > _node:eta - mnvr_t(_node:deltav)+3.
	RCS on.
	set pilot:control:fore to 1.
	wait until node_ullage.
	set pilot:control:fore to 0.
	RCS off.
	set throttle to 1.
	wait until node_completed(_node).
	return true.
}
