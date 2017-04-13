declare function icpg_eval {
	// Evaluates ICPG equations for current state variables and returns final state. 
	// See ICPG paper for more information.

	// Function arguments.
	parameter guidance_params.												// guidance_params = [cutoff_eta,pitch_angle,eng_thrust]
	parameter current_state.												// current_state = [init_xvel,init_yvel,init_ypos,eng_Ve,tau]
	parameter body_params is list(ship:body:mu,ship:body:radius).			// body_params = [body_mu,body_rad]
	
	// Unpack input arguments.
	local cutoff_eta is guidance_params[0].									// Time to engine cut-off.
	local pitch_angle is guidance_params[1].								// Pitch angle over horizon in degrees.
	local eng_thrust is guidance_params[2].									// Normalized thrust (from 0 to 1).
	
	local init_xvel is current_state[0].									// Current horizontal velocity in m/s.
	local init_yvel is current_state[1].									// Current vertical velocity in m/s.
	local init_ypos is current_state[2].									// Current height over ground in meters.
    local eng_Ve is current_state[3].										// Average stage effective exhaust velocity in m/s.
	local tau is current_state[4] / eng_thrust.								// Time to burn total vehicle mass under current mass flow rate.
	
	local body_mu is body_params[0].										// Parent body standard gravitational parameter in m^3/s^2.
	local body_rad is body_params[1].										// Parent body radius in meters.
	
	// Calculate external forces (centrifugal and gravitational) in a rotating frame of reference.
	local cent_acc is (init_xvel^2) / (body_rad+init_ypos).
	local grav_acc is body_mu / ((body_rad+init_ypos)^2).
	local ext_acc is cent_acc - grav_acc.
	
	// Evaluate guidance equations.
	local final_xvel is eng_Ve*ln(tau/(tau-cutoff_eta))*cos(pitch_angle) + init_xvel.
	local final_yvel is eng_Ve*ln(tau/(tau-cutoff_eta))*sin(pitch_angle) + ext_acc*cutoff_eta + init_yvel.
	local final_ypos is -eng_Ve*((tau-cutoff_eta)*ln(tau/(tau-cutoff_eta))-cutoff_eta)*sin(pitch_angle) + 0.5*ext_acc*cutoff_eta^2 + init_yvel*cutoff_eta + init_ypos.
	
	// Pack output arguments.
	local final_state is list(final_xvel,final_yvel,final_ypos).
	return final_state.
}