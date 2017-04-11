function [out,grad,step_size,iter] = grad_descent(fun,args,learning_rate,momentum_fac,tol,max_iter)
    out = args;
    step_size = 1;
    iter = 0;
    grad = [0,0];
    old_grad = [0,0];
   
    stop_condition = 0;        
    while stop_condition == 0
        if tol == 0
            if iter >= max_iter-1
                stop_condition = 1;
            end
        elseif tol ~= 0
            if step_size <= tol || iter >= max_iter-1
                stop_condition = 1;
            end
        end    
        iter = iter+1;
        old_out = out;
        old_grad = grad;
        
        grad(1) = (fun([out(1)+0.001,out(2)])-fun([out(1)-0.001,out(2)]))./0.002;
        grad(2) = (fun([out(1),out(2)+0.001])-fun([out(1),out(2)-0.001]))./0.002;
        grad = momentum_fac.*old_grad + grad;
        out = out - learning_rate.*grad;
        step_size = norm(out-old_out);
    end    
end