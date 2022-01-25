function [position,isterminal,direction] = stopHumanODE(t,y)

global x_des y_des z_des
global X_on Y_on Z_on
global precision

T_ee = getTransf(y(1),y(2),y(3),y(4),y(5),y(6),y(7));

errorX = (T_ee(1,4)-x_des);
errorY = (T_ee(2,4)-y_des);
errorZ = (T_ee(3,4)-z_des);

error = sqrt(X_on*errorX^2 + Y_on*errorY^2 + Z_on*errorZ^2);

position = error-precision;

isterminal = 1;  % Halt integration 
direction = 0;   % The zero can be approached from either direction

end

