function x = RK4(model, h, x, u)

    k1 = h * feval(model, x, u);            % evaluate derivative k1
    k2 = h * feval(model, x+k1/2, u);	    % evaluate derivative k2
    k3 = h * feval(model, x+k2/2, u);	    % evaluate derivative k3
    k4 = h * feval(model, x+k3, u);		    % evaluate derivative k4
    
    x = x +(k1 + 2*k2 + 2*k3 + k4)/6;		% averaged output
end