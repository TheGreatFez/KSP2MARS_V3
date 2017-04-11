function [cost] = icpg_cost2(args)
    target_state = args{1};
    weights = args{2};
    guidance_params = args{3};
    current_state = args{4};
    body_params = args{5};
    constraints = args{6};
    cost = icpg_cost(target_state,weights,guidance_params,current_state,body_params);
    if guidance_params{1} < 0
        cost = cost + 1e+5.*guidance_params{1}.^2;
    elseif guidance_params{1} > constraints{1}
        cost = cost + 1e+5.*(guidance_params{1}-constraints{1}).^3;
    end
    if guidance_params{2} < 0
        cost = cost + 1e+5.*guidance_params{2}.^2;
    elseif guidance_params{2} > constraints{2}
        cost = cost + 1e+5.*(guidance_params{2}-constraints{2}).^2;
    end
    if guidance_params{3} < 0.5
        cost = cost + 1e+5.*(guidance_params{3}-0.5).^4;
    elseif guidance_params{3} > constraints{3}
        cost = cost + 1e+8.*(guidance_params{3}-constraints{3}).^4 + 1e+9.*(guidance_params{3}-constraints{3}).^2;
    end
end