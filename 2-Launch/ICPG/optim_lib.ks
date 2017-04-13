declare function test_fun {									// Test function for optimization.
	parameter inputs.
	local xval is inputs[0].
	local yval is inputs[1].	
	local zval is inputs[2].	
	
	declare out to xval^2 + yval^2 + zval^2.
	
	return out.
}


declare function gradient {									// Gradient of a 3D function
	parameter input_fun.
	parameter fun_args is list(0,0,0).
	
	local grad is list(0,0,0).
	
	set fwd_eval to list(fun_args[0]+0.001,fun_args[1]+0.001,fun_args[2]+0.001).
	set bwd_eval to list(fun_args[0]-0.001,fun_args[1]-0.001,fun_args[2]-0.001).
	
	set grad[0] to (input_fun(list(fwd_eval[0],fun_args[1],fun_args[2]))-input_fun(list(bwd_eval[0],fun_args[1],fun_args[2])))/0.002.
	set grad[1] to (input_fun(list(fun_args[0],fwd_eval[1],fun_args[2]))-input_fun(list(fun_args[0],bwd_eval[1],fun_args[2])))/0.002.	
	set grad[2] to (input_fun(list(fun_args[0],fun_args[1],fwd_eval[2]))-input_fun(list(fun_args[0],fun_args[1],bwd_eval[2])))/0.002.	

	return grad.
}


declare function grad_descent {
	// Momentum accelerated gradient descent algorithm.

	parameter input_fun.									// Function to optimize (has to have 3 arguments)
	parameter init_guess is list(0,0,0).					// (x,y,z)
	parameter settings is list(1e-4,50,0.1,0.9).			// (Tol,MaxIter,LearningRate,MomentumFactor)
	parameter optim_verbose is 0.							// If optim_verbose = 1, print debugging data.

	// Initialize helping variables.
	local step_size is 1.
	local iter is 0.
	local grad is list(0,0,0).
	local step_grad is list(0,0,0).
	declare param to init_guess.
	local old_param is list(0,0,0).
	local old_grad is list(0,0,0).
	
	local stop_condition is false.
	
	until stop_condition = true {
		// Check for fulfilment of stopping conditions.
		if settings[0] = 0 {
			if iter > (settings[1]-1) {
				print("Stop criteria reached.").
				print("   Iterations: " + iter).
				print("   Tolerance: " + step_size).
				set stop_condition to true.
			}
		}
		else if settings[0] <> 0 {
			if iter > (settings[1]-1) or step_size <= settings[0] {
				print("Stop criteria reached.").
				print("   Iterations: " + iter).
				print("   Tolerance: " + step_size).
				set stop_condition to true.				
			}
		} 
		
		// Update helping variables.
		set iter to iter+1.
		set old_param to param.
		set old_grad to grad.
		
		// Update gradient.
		set grad to gradient(input_fun@,param).
		
		// Calculate the descent step to take.
		set step_grad[0] to -settings[2]*(settings[3]*old_grad[0]).
		set step_grad[1] to -settings[2]*(settings[3]*old_grad[1]).
		set step_grad[2] to -settings[2]*(settings[3]*old_grad[2]).
		
		// Update step size and current parameters.
		set step_size to sqrt(step_grad[0]^2+step_grad[1]^2+step_grad[2]^2).
		set param[0] to param[0] + step_grad[0].
		set param[1] to param[1] + step_grad[1].
		set param[2] to param[2] + step_grad[2].
		
		if optim_verbose = 1 {print(param).}
	}
	
	if optim_verbose = 1 {print(""). print("Optimized value: "). print(param).}
	return param.
}


declare function acc_grad_descent {
	// Function-aware momentum accelerated gradient descent algorithm. Not safe for negative values.

	parameter input_fun.									// Function to optimize (has to have 3 arguments)
	parameter init_guess is list(0,0,0).					// (x,y,z)
	parameter settings is list(1e-4,50,0.1,0.9).			// (Tol,MaxIter,LearningRate,MomentumFactor)
	parameter optim_verbose is 0.							// If optim_verbose = 1, print debugging data.
	
	// Initialize helping variables.
	local step_size is 1.
	local iter is 0.
	local grad is list(0,0,0).
	local step_grad is list(0,0,0).
	declare param to init_guess.
	local old_param is list(0,0,0).
	local old_grad is list(0,0,0).
	
	local stop_condition is false.
	
	until stop_condition = true {
		// Check for fulfilment of stopping conditions.
		if settings[0] = 0 {
			if iter > (settings[1]-1) {
				print("Stop criteria reached.").
				print("   Iterations: " + iter).
				print("   Tolerance: " + step_size).
				set stop_condition to true.
			}
		}
		else if settings[0] <> 0 {
			if iter > (settings[1]-1) or step_size <= settings[0] {
				print("Stop criteria reached.").
				print("   Iterations: " + iter).
				print("   Tolerance: " + step_size).
				set stop_condition to true.				
			}
		} 
		
		// Update helping variables.
		set iter to iter+1.
		set old_param to param.
		set old_grad to grad.
		
		// Update gradient.
		set grad to gradient(input_fun@,param).
		
		// Calculate the descent step to take.
		set step_grad[0] to -settings[2]*(settings[3]*old_grad[0]+grad[0]*param[0]).
		set step_grad[1] to -settings[2]*(settings[3]*old_grad[1]+grad[1]*param[1]).
		set step_grad[2] to -settings[2]*(settings[3]*old_grad[2]+grad[2]*param[2]).
		
		// Update step size and current parameters.
		set step_size to sqrt(step_grad[0]^2+step_grad[1]^2+step_grad[2]^2).
		set param[0] to param[0] + step_grad[0].
		set param[1] to param[1] + step_grad[1].
		set param[2] to param[2] + step_grad[2].
		
		if optim_verbose = 1 {print(param).}
	}
	if optim_verbose = 1 {print(""). print("Optimized value: "). print(param).}
	return param.
}