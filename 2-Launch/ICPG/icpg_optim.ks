declare function icpg_optim {
	// Optimizes ICPG parameters for desired final state.
	// See ICPG paper for more information.

	// Function arguments.
	parameter target_state.													// target_state = [final_xvel,final_yvel,final_ypos]
	parameter weights.														// weights = [xvelerror_w,yvelerror_w,yposerror_w]
	parameter current_state.												// current_state = [init_xvel,init_yvel,init_ypos,eng_Ve,tau]
	parameter body_params.													// body_params = [body_mu,body_rad]
	parameter constraints.													// constraints = [max_cutofftime,max_pitchangle,max_thrust]
	parameter optim_settings.												// optim_settings = [tolerance,max_iter,learning_rate,momentum_factor]
	parameter init_guess.													// init_guess = [cutofftime_guess,pitchangle_guess,thrust_guess]
	
	// Prepare variables for optimization.
	local cutoff_eta is init_guess[0].
	local pitch_angle is init_guess[1].
	local eng_thrust is init_guess[2].
	
	// Create function to optimize.
	local icpg_optimfun is icpg_cost@:bind(target_state,weights,current_state,body_params,constraints).
	
	// Use gradient descent from optim_lib.ks to optimize ICPG parameters.
	local optim_guid is acc_grad_descent(icpg_optimfun@,init_guess,optim_settings).
	return optim_guid.
}