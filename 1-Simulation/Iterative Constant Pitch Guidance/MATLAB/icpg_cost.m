function [cost] = icpg_cost(target_state,weights,guidance_params,current_state,body_params)
% ___________________
% 
% ICPG_COST: Evaluate ICPG performance with supplied parameters (lower is better).
%     
%     INPUTS
%         target_state = {target_xvel,target,yvel,target_ypos}
%             target_xvel: Desired final horizontal velocity. Measured in m*s^-1.
%             target_yvel: Desired final vertical velocity. Measured in m*s^-1.
%             target_ypos: Desired final height over ground. Measured in meters.
%         weights = {xvel_weight,yvel_weight,ypos_weight}
%             xvel_weight: Horizontal velocity weight used in cost calculations.
%             yvel_weight: Vertical velocity weight used in cost calculations.
%             ypos_weight: Final height weight used in cost calculations.
%         guidance_params = {time,pitch_angle,thrust}
%             For help, run 'help icpg_eval' on command window.
%         current_state = {xvel_0,yvel_0,ypos_0,Ve,tau}
%             For help, run 'help icpg_eval' on command window.
%         body_params = [body_mu,body_radius]
%             For help, run 'help icpg_eval' on command window.
%     
%     OUTPUTS
%         cost: Performance of guidance (lower is better).
%         
%     EXAMPLE:
%         target_state = {7700,0,165000};
%         weights = {1,1,1};
%         guidance_params = {350,22,0.95};
%         current_state = {2100,1100,65000,3450,550};
%         body_params = {3.986004418e14,6371000};
%         
%         >> icpg_cost(target_state,weights,guidance_params,current_state,body_params)
% 
%            ans =
% 
%                0.4650
% 
%     NOTES:
%         - Parameter sweeps are possible for one input parameter at a time, e.g. time.
%         - Depending on the inputs, some answers will be complex. Ignore those.
% ___________________

% Evaluate ICPG with supplied parameters
    final_state = icpg_eval(guidance_params,current_state,body_params);
    final_xvel = final_state{1};
    final_yvel = final_state{2};
    final_ypos = final_state{4};
    
% Unpack target state parameters
    target_xvel = target_state{1};
    target_yvel = target_state{2};
    target_ypos = target_state{3};

% Calculate absolute and weighted errors at final state
    xvel_abserror = final_xvel - target_xvel;
    yvel_abserror = final_yvel - target_yvel;
    ypos_abserror = final_ypos - target_ypos;
    
    vel_werror = (weights{1}.*xvel_abserror).^2 + (weights{2}.*yvel_abserror).^2;
    pos_werror = (0.05.*weights{3}.*ypos_abserror).^2;
% Calculate cost (performance) of supplied parameters
    cost = vel_werror + pos_werror;
end