function [t,y] = solveODE(th0,offset)
%#codegen
delta_t = [0 10];
t = 0:1;
y = bsxfun(@times,ones(size(t')),th0);

opts = odeset('Events',@stopHumanODE,'AbsTol',1e-3);
[t,y] = ode45(@(t,y) humanODE(t, y),delta_t,th0+offset,opts);

end

