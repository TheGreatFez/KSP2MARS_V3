clearscreen.
switch to 0. 

run once optim_lib.
run once icpg_eval.
run once icpg_cost.
run once icpg_optim.

set guidance_params to list(390,23,1).
set current_state to list(2150,1150,65000,3380,468).
set body_params to list(3.986004418e14,6371000).

set weights to list(1,1,0.1).
set constraints to list(468,45,1).
set optim_settings to list(0,10000000,2e-10,0.999).

set target_state to list(7700,0,165000).

print(icpg_optim(target_state,weights,current_state,body_params,constraints,optim_settings)).