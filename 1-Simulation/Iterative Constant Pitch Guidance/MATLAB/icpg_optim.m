function [optim_param] = icpg_optim(target_state,weights,current_state,body_params,constraints,learning_rate)
    tol = 5e-4;
    max_iter = 10000;
    momentum_fac = 0.9998;
    
    guidance_params = {380 20 1};

    step_size = 1;
    iter = 0;
    grad = [0,0,0];

    stop_condition = 0;
    
    while stop_condition == 0
        if tol == 0
            if iter > max_iter-1
                fprintf('Stop criteria reached.\n   Number of iterations: %d \n   Current step size: %d \n',iter,step_size)
                stop_condition = 1;
            end
        elseif tol ~= 0
            if step_size <= tol || iter > max_iter-1
                fprintf('Stop criteria reached.\n   Number of iterations: %d \n   Current step size: %d \n',iter,step_size)
                stop_condition = 1;
            end
        end
        iter = iter+1;
        old_guidance_params = guidance_params;
        old_grad = grad;
        
        time_guid_first = {guidance_params{1}+0.001,guidance_params{2},guidance_params{3}};
        time_guid_second = {guidance_params{1}-0.001,guidance_params{2},guidance_params{3}};
        time_args_first = {target_state,weights,time_guid_first,current_state,body_params,constraints};
        time_args_second = {target_state,weights,time_guid_second,current_state,body_params,constraints};
        grad(1) = (icpg_cost2(time_args_first)-icpg_cost2(time_args_second))./0.002;
        
        pitch_guid_first = {guidance_params{1},guidance_params{2}+0.001,guidance_params{3}};
        pitch_guid_second = {guidance_params{1},guidance_params{2}-0.001,guidance_params{3}};
        pitch_args_first = {target_state,weights,pitch_guid_first,current_state,body_params,constraints};
        pitch_args_second = {target_state,weights,pitch_guid_second,current_state,body_params,constraints};
        grad(2) = (icpg_cost2(pitch_args_first)-icpg_cost2(pitch_args_second))./0.002;

        thrust_guid_first = {guidance_params{1},guidance_params{2},guidance_params{3}+0.001};
        thrust_guid_second = {guidance_params{1},guidance_params{2},guidance_params{3}-0.001};
        thrust_args_first = {target_state,weights,thrust_guid_first,current_state,body_params,constraints};
        thrust_args_second = {target_state,weights,thrust_guid_second,current_state,body_params,constraints};
        grad(3) = (icpg_cost2(thrust_args_first)-icpg_cost2(thrust_args_second))./0.002;
        
        grad = -learning_rate.*(momentum_fac.*old_grad + grad);

        guidance_params = num2cell(cell2mat(guidance_params) + grad);
        step_size = norm(cell2mat(guidance_params)-cell2mat(old_guidance_params));
                
        optim_param = guidance_params;        
    end
end