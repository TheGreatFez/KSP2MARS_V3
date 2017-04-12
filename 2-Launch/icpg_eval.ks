declare function icpg_eval {
	local avIsp is 0.
	list engines in engine_list.
	for eng in engine_list {
		set avIsp to avIsp + eng:maxthrust / maxthrust * eng:isp.
	}
	
	parameter guidance_params.
	parameter current_state.
	parameter body_params is list(ship:body:mu,ship:body:radius).
	
	local cutoff_eta is guidance_params[0].
	local pitch_angle is guidance_params[1].
	local eng_thrust is guidance_params[2].
	
	local init_xvel is current_state[0].
	local init_yvel is current_state[1].
	local init_ypos is current_state[2].
    local eng_Ve is current_state[3].
	local tau is current_state[4] / eng_thrust.
	
	local body_mu is body_params[0].
	local body_rad is body_params[1].
	
	local cent_acc is (init_xvel^2) / (body_rad+init_ypos).
	local grav_acc is body_mu / ((body_rad+init_ypos)^2).
	local ext_acc is cent_acc - grav_acc.
	
	local final_xvel is eng_Ve*ln(tau/(tau-cutoff_eta))*cos(pitch_angle) + init_xvel.
	local final_yvel is eng_Ve*ln(tau/(tau-cutoff_eta))*sin(pitch_angle) + ext_acc*cutoff_eta + init_yvel.
	local final_ypos is -eng_Ve*((tau-cutoff_eta)*ln(tau/(tau-cutoff_eta))-cutoff_eta)*sin(pitch_angle) + 0.5*ext_acc*cutoff_eta^2 + init_yvel*cutoff_eta + init_ypos.
	
	local final_state is list(final_xvel,final_yvel,final_ypos).
	return final_state.
}