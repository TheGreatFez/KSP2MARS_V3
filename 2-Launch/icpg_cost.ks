@lazyglobal off.

declare function icpg_cost {
	parameter target_state.
	parameter weights.
	parameter current_state.
	parameter body_params.
	parameter constraints.
	parameter guidance_params.
	
	local final_state is icpg_eval(guidance_params, current_state, body_params).
	local final_xvel is final_state[0].
	local final_yvel is final_state[1].
	local final_ypos is final_state[2].
	
	local target_xvel is target_state[0].
	local target_yvel is target_state[1].
	local target_ypos is target_state[2].
	
	local xvel_abserror is final_xvel - target_xvel.
	local yvel_abserror is final_yvel - target_yvel.
	local ypos_abserror is final_ypos - target_ypos.
	
	local vel_werror is (weights[0] * xvel_abserror)^2 + (weights[1]*yvel_abserror)^2.
	local pos_werror is (0.05*weights[2]*ypos_abserror)^2.
	
	local cost is vel_werror + pos_werror.

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
	
	return cost.
}