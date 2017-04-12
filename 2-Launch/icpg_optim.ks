declare function icpg_optim {
	parameter target_state.
	parameter weights.
	parameter current_state.
	parameter body_params.
	parameter constraints.
	parameter optim_settings.
	
	local init_guess is list(350,23,0.9).
	local cutoff_eta is init_guess[0].
	local pitch_angle is init_guess[1].
	local eng_thrust is init_guess[2].
	
	local icpg_optimfun is icpg_cost@:bind(target_state,weights,current_state,body_params,constraints).
	
	local optim_guid is acc_grad_descent(icpg_optimfun@,init_guess,optim_settings).
	return optim_guid.
}