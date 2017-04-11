function [final_state] = icpg_eval(guidance_params,current_state,body_params)
% ___________________
% 
% ICPG_EVAL: Evaluate Iterative Constant Pitch Guidance scheme.
%     
%     INPUTS
%         guidance_params = {time,pitch_angle,thrust}
%             time: Time from now to evaluate the guidance. Measured in seconds.
%             pitch_angle : Pitch angle over horizon to hold. Measured in (sexagesimal) degrees.
%             thrust: Thrust setting from 0 to 1.
%         current_state = {xvel_0,yvel_0,ypos_0,Ve,tau}
%             xvel_0: Current horizontal velocity. Measured in m*s^-1.
%             yvel_0: Current vertical velocity. Measured in m*s^-1.
%             ypos_0: Current height over ground. Measured in meters.
%             Ve: Effective exhaust velocity of engines. Measured in m*s^-1.
%             tau: Time to burn whole mass at current mdot (total_mass/mdot). Measured in seconds.
%         body_params = [body_mu,body_radius]
%             body_mu: Parent body standard gravitational parameter. Measured in m^3*s^-2.
%             body_radius: Parent body radius. Measured in meters.
%     
%     OUTPUTS
%         final_state = {xvel,yvel,xpos,ypos}
%             xvel: Final horizontal velocity. Measured in m*s^-1.
%             yvel: Final vertical velocity. Measured in m*s^-1.
%             xpos: Final downrange distance from current position. Measured in m*s^-1.
%             ypos: Final height over ground. Measured in meters.
%         
%     EXAMPLE:
%         guidance_params = {350,22,0.95};
%         current_state = {2100,1100,65000,3450,550};
%         body_params = {3.986004418e14,6371000};
%         
%         >> icpg_eval(guidance_params,current_state,body_params)
% 
%         ans = 
% 
%             [5.0676e+03]    [-829.2122]    [1.1752e+06]    [8.0401e+04]
% 
%     NOTES:
%         - Parameter sweeps are possible for one input parameter at a time, e.g. time.
%         - Depending on the inputs, some answers will be complex. Ignore those.
% ___________________
    
% Unpack parameters
    time = guidance_params{1};
    pitch_angle = deg2rad(guidance_params{2});
    thrust = guidance_params{3};
    
    xvel_0 = current_state{1};
    yvel_0 = current_state{2};
    ypos_0 = current_state{3};
    Ve = current_state{4};
    tau = current_state{5}./thrust;
    
    body_mu = body_params{1};
    body_radius = body_params{2};

% Calculate external forces
    centrifg_acc = (xvel_0.*xvel_0)./(body_radius+ypos_0);
    gravity_acc = body_mu./((body_radius+ypos_0).^2);
    external_acc = centrifg_acc - gravity_acc;
    
% Evaluate ICPG on current state
    xvel = Ve.*log(tau./(tau-time)).*cos(pitch_angle) + xvel_0;
    yvel = Ve.*log(tau./(tau-time)).*sin(pitch_angle) + external_acc.*time + yvel_0;
    xpos = -Ve.*((tau-time).*log(tau./(tau-time))-time).*cos(pitch_angle) + xvel_0.*time;
    ypos = -Ve.*((tau-time).*log(tau./(tau-time))-time).*sin(pitch_angle) + 0.5.*external_acc.*time.*time + yvel_0.*time + ypos_0;
    
% Return state evaluated at initial parameters
    final_state = {xvel,yvel,xpos,ypos};
end
