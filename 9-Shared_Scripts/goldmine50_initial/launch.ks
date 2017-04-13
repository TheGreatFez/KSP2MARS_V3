declare function launch_time{
	declare parameter launch_lat, launch_lng, target, ut.

	set lat_1 to launch_lat.
	set bearing to 90-target:orbit:inclination.

	set lng_0 to target:orbit:lan-target:body:rotationangle.

	set delta_lng to arcsin(sin(lat_1)*tan(bearing)/cos(lat_1)).

	set lng_1 to lng_0 + delta_lng.

	set target_lng to launch_lng.

	set rot_period to ship:body:rotationperiod.

	set dt to (lng_1-target_lng)*(rot_period/360).

	if dt<0{set dt to dt+rot_period.}

	clearscreen.

	return ut+dt.
}

list targets in my_targs.
for i in my_targs{
    if i:name = "Chaser"{ set target to i. }//change this as necessary or comment out and select target maually before execution of script
}

set t1 to target.

set l_time to launch_time(ship:geoposition:lat, ship:geoposition:lng, t1, time:seconds).

set lag to 6*60.//lag time to orbit change depending on launch vehicle

print "Launch at UT: "+ (l_time-lag) +" s".

kuniverse:timewarp:warpto(l_time-lag).

declare function Launch_azimuth{
	declare parameter incline.
	declare parameter ApSetpoint.

	set launch_lat to ship:geoposition:lat.

	if incline<launch_lat{
	   return "Please pick an appropriate inclination...".
	}

	set beta to arcsin(cos(incline)/cos(launch_lat)).

	set orbit_speed to sqrt(orbit:body:mu/(ApSetpoint*1000+orbit:body:radius)).

	set equatorial_speed to 2.0*constant:pi*orbit:body:radius/orbit:body:rotationperiod.

	set vrot to V(0,0,0).
	set vrot:X to orbit_speed*sin(beta)-equatorial_speed*cos(launch_lat).
	set vrot:Y to orbit_speed*cos(beta).

	set beta_rot to arctan(vrot:X/vrot:Y).
	set vrot to vrot:mag.
	set dv_saving to orbit_speed-vrot.

	return beta_rot.
}

set ApSetpoint to 240.
set rot to Launch_azimuth(t1:orbit:inclination, ApSetpoint).
print "at Launch Azimuth: "+rot+"deg".

wait 1.
lock throttle to 1.

set steertag to heading(rot, 90).
lock steering to steertag.

wait until ship:airspeed>=100.

set a to 0.928738.
set b to 0.001612.
set c to -99.6840.

set Ku to 2.5.
set Tu to 8.0.
set Kp to 0.45*Ku.
set Ki to 1.2*Kp/Tu.
set Kd to 0.

set offset to PIDLOOP(Kp, Ki, Kd).
set offset:maxoutput to 5.0.
set offset:minoutput to -5.0.

until ship:availablethrust<1{
set v to ship:velocity:surface.
set pitprof to 90.0-a*arctan(b*(v:mag+c)).

set vpitch to 90-vang(heading(0,90):vector, v).

set diff to vpitch - pitprof.

set act to offset:update(TIME:SECONDS, diff).

set pitprof to pitprof + act.

set steertag to heading(rot, pitprof).

set Ap to ship:apoapsis*0.001.

if Ap>ApSetpoint-20{
break.
}

clearscreen.
print "Pitch: " + pitprof at (0,0).
print "Apoapsis: " + Ap at (0,2).
print "Deviation: " + act at (0,4).
wait 0.01.
}
clearscreen.
run Final_stage(ApSetpoint,rot,target:orbit:inclination).