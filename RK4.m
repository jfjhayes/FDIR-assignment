function x = RK4(model, h, x, u)
    
    % RK4 integration
    k1 = h * model(x, u);                   % evaluate derivative k1
    k2 = h * model(x+k1/2, u);	            % evaluate derivative k2
    k3 = h * model(x+k2/2, u);	            % evaluate derivative k3
    k4 = h * model(x+k3, u);		        % evaluate derivative k4
    
    x = x +( k1 + 2*k2 + 2*k3 + k4)/6;		% averaged output
end