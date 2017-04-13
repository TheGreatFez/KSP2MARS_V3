declare function icpg_cost {
	// Evaluates ICPG equations for current state variables and returns guidance performance (cost). 
	// See ICPG paper for more information.

	parameter target_state.														// target_state = [final_xvel,final_yvel,final_ypos]
	parameter weights.															// weights = [xvelerror_w,yvelerror_w,yposerror_w]
	parameter current_state.													// current_state = [init_xvel,init_yvel,init_ypos,eng_Ve,tau]
	parameter body_params.														// body_params = [body_mu,body_rad]
	parameter constraints.														// constraints = [max_cutofftime,max_pitchangle,max_thrust]
	parameter guidance_params.													// guidance_params = [cutoff_eta,pitch_angle,eng_thrust]
	
	// Evaluate current final_state and unpack variables.
	local final_state is icpg_eval(guidance_params, current_state, body_params).
	local final_xvel is final_state[0].
	local final_yvel is final_state[1].
	local final_ypos is final_state[2].
	
	local target_xvel is target_state[0].
	local target_yvel is target_state[1].
	local target_ypos is target_state[2].
	
	// Calculate absolute errors in final state.
	local xvel_abserror is final_xvel - target_xvel.
	local yvel_abserror is final_yvel - target_yvel.
	local ypos_abserror is final_ypos - target_ypos.
	
	// Weight errors according to input weights.
	local vel_werror is (weights[0] * xvel_abserror)^2 + (weights[1]*yvel_abserror)^2.
	local pos_werror is (0.05*weights[2]*ypos_abserror)^2.
	
	// Calculate base cost.
	local cost is vel_werror + pos_werror.
	
	// Penalty method for cost function (WIP).
	if guidance_params[0] < 0 {
		set cost to cost + 1e5*abs(guidance_params[0]).
	} else if guidance_params[0] > constraints[0] {
		set cost to cost + 1e5*abs(guidance_params[0]-constraints[0]).
	}
	if guidance_params[1] < 0 {
		set cost to cost + 1e5*abs(guidance_params[1]).
	} else if guidance_params[1] > constraints[1] {
		set cost to cost + 1e5*abs(guidance_params[1]-constraints[1]).
	}
	if guidance_params[2] < 0.5 {
		set cost to cost + 1e5*abs(guidance_params[2]-0.5).
	} else if guidance_params[2] > constraints[2] {
		set cost to cost + 1e8*abs(guidance_params[2]-constraints[2]).
	}
	
	// Return adjusted cost.
	return cost.
}