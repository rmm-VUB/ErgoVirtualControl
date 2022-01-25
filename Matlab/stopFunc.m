function [position,isterminal,direction] = stopFunc(t,y)

global l1 l2 l3 x_des y_des X_on Y_on precision


s1 = sin(y(1));
s12 = sin(y(1)+y(2));
s123 = sin(y(1)+y(2)+y(3));
c1 = cos(y(1));
c12 = cos(y(1)+y(2));
c123 = cos(y(1)+y(2)+y(3));

x_ee = l1*c1 + l2*c12 + l3*c123;
y_ee = l1*s1 + l2*s12 + l3*s123;

errorX = x_ee - x_des;
errorY = y_ee - y_des;

error = sqrt(X_on*errorX^2 + Y_on*errorY^2);

position = error-precision;

isterminal = 1;  % Halt integration 
direction = 0;   % The zero can be approached from either direction

end

