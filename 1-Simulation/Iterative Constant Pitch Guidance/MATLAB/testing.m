% guidance_params = {4.084430815108222e+02,23.559796328620640,0.9608315};
current_state = {2200,1100,70000,3380,468};
body_params = {3.986004418e14,6371000};

target_state = {7700,0,165000};
weights = {1.4,2,0.1};

% final_state = icpg_eval(guidance_params,current_state,body_params);
% icpg_cost(target_state,weights,guidance_params,current_state,body_params);


best_guidance = icpg_optim(target_state,weights,current_state,body_params,{468,45,1},5e-10);
guidance_params = best_guidance;


constraints = {468,45,1};
time_guid_first = {guidance_params{1}+0.001,guidance_params{2},guidance_params{3}};
time_guid_second = {guidance_params{1}-0.001,guidance_params{2},guidance_params{3}};
time_args_first = {target_state,weights,time_guid_first,current_state,body_params,constraints};
time_args_second = {target_state,weights,time_guid_second,current_state,body_params,constraints};
grad(1) = -(icpg_cost2(time_args_first)-icpg_cost2(time_args_second))./0.002;

pitch_guid_first = {guidance_params{1},guidance_params{2}+0.001,guidance_params{3}};
pitch_guid_second = {guidance_params{1},guidance_params{2}-0.001,guidance_params{3}};
pitch_args_first = {target_state,weights,pitch_guid_first,current_state,body_params,constraints};
pitch_args_second = {target_state,weights,pitch_guid_second,current_state,body_params,constraints};
grad(2) = -(icpg_cost2(pitch_args_first)-icpg_cost2(pitch_args_second))./0.002;

thrust_guid_first = {guidance_params{1},guidance_params{2},guidance_params{3}+0.001};
thrust_guid_second = {guidance_params{1},guidance_params{2},guidance_params{3}-0.001};
thrust_args_first = {target_state,weights,thrust_guid_first,current_state,body_params,constraints};
thrust_args_second = {target_state,weights,thrust_guid_second,current_state,body_params,constraints};
grad(3) = -(icpg_cost2(thrust_args_first)-icpg_cost2(thrust_args_second))./0.002;







% Integrate flight trajectory
best_guidance = cell2mat(best_guidance);
guidance_params = {(0:ceil(best_guidance(1))),best_guidance(2),best_guidance(3)};
time = guidance_params{1};
final_state = icpg_eval(guidance_params,current_state,body_params);
best_final_state = [final_state{1}(end),final_state{2}(end),final_state{4}(end)];
plot(final_state{3}./1000,final_state{4}./1000);