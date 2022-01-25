function dy = humanODE(t,y)
%#codegen

global k1 k2 k3 k4 k5 k6 k7
global c1_d c2_d c3_d c4_d c5_d c6_d c7_d
global th1_star th2_star th3_star th4_star th5_star th6_star th7_star
global x_des y_des z_des
global kx kIx cx ky kIy cy kz kIz cz
global X_on Y_on Z_on
global cumErrorX cumErrorY cumErrorZ
global x_lim_1 x_lim_2 y_lim_1 y_lim_2 z_lim_1 z_lim_2
global dy_prev

dy = zeros(7,1);

T_ee = getTransf(y(1),y(2),y(3),y(4),y(5),y(6),y(7));

J = getJacobian(y(1),y(2),y(3),y(4),y(5),y(6),y(7));

X_des = x_des;
Y_des = y_des;
Z_des = z_des; 

errorX = (T_ee(1,4)-X_des);
errorY = (T_ee(2,4)-Y_des);
errorZ = (T_ee(3,4)-Z_des);

cumErrorX = cumErrorX + errorX;
cumErrorY = cumErrorY + errorY;
cumErrorZ = cumErrorZ + errorZ;

f = [X_on; Y_on; Z_on].*[-kx*errorX-kIx*cumErrorX;-ky*errorY-kIy*cumErrorY;-kz*errorZ-kIz*cumErrorZ];

tau = J'*f;

dy(1) = (tau(1) - k1*(y(1) - th1_star)^3)/c1_d;
dy(2) = (tau(2) - k2*(y(2) - th2_star)^3)/c2_d;
dy(3) = (tau(3) - k3*(y(3) - th3_star)^3)/c3_d;
dy(4) = (tau(4) - k4*(y(4) - th4_star)^3)/c4_d;
dy(5) = (tau(5) - k5*(y(5) - th5_star)^3)/c5_d;
dy(6) = (tau(6) - k6*(y(6) - th6_star)^3)/c6_d;
dy(7) = (tau(7) - k7*(y(7) - th7_star)^3)/c7_d;

if (y(1) >= pi/2 && dy(1) > 0) || (y(1) <= -pi/30 && dy(1) < 0)
    dy(1) = 0;
end

if (y(2) >= pi/4 && dy(2) > 0) || (y(2) <= -pi/4 && dy(2) < 0)
    dy(2) = 0;
end

if (y(3) >= pi/2 && dy(3) > 0) || (y(3) <= -pi/2 && dy(3) < 0)
    dy(3) = 0;
end

if (y(4) >= pi && dy(4) > 0) || (y(4) <= -pi/10 && dy(4) < 0)
    dy(4) = 0;
end

if (y(5) >= pi && dy(5) > 0) || (y(5) <= 0 && dy(5) < 0)
    dy(5) = 0;
end

if (y(6) >= pi/4 && dy(6) > 0) || (y(6) <= -pi/2 && dy(6) < 0)
    dy(6) = 0;
end

if (y(7) >= 4*pi/5 && dy(7) > 0) || (y(7) <= 0 && dy(7) < 0)
    dy(7) = 0;
end

% if(T_ee(1,4) < -0.3 && v_ee(1) < 0)
%    dy = pinv(J)*[0; v_ee(2); v_ee(3)];
% end

v_ee = J*dy;

limit_reached = 0;
v = v_ee;

if ~X_on && ((T_ee(1,4) <= x_lim_1 && v_ee(1) < 0) || (T_ee(1,4) >= x_lim_2 && v_ee(1) > 0))
    %    if (T_ee(1,4) <= x_lim_1 && v_ee(1) < 0)
    %     v(1) = 1;
    %    end
    %    if (T_ee(1,4) >= x_lim_2 && v_ee(1) > 0)
    %     v(1) = -1;
    %    end
    v(1) = 0;
    limit_reached = 1;
end

if ~Y_on && ((T_ee(2,4) <= y_lim_1 && v_ee(2) < 0) || (T_ee(2,4) >= y_lim_2 && v_ee(2) > 0))
    %     if (T_ee(2,4) <= y_lim_1 && v_ee(2) < 0)
    %         v(2) = 1;
    %     end
    %     if (T_ee(2,4) >= y_lim_2 && v_ee(2) > 0)
    %         v(2) = -1;
    %     end
    v(2) = 0;
    limit_reached = 1;
end

if ~Z_on && ((T_ee(3,4) <= z_lim_1 && v_ee(3) < 0) || (T_ee(3,4) >= z_lim_2 && v_ee(3) > 0))
    %     if (T_ee(3,4) <= z_lim_1 && v_ee(3) < 0)
    %         v(3) = 1;
    %     end
    %     if (T_ee(3,4) >= z_lim_2 && v_ee(3) > 0)
    %         v(3) = -1;
    %     end
    v(3) = 0;
    limit_reached = 1;
end

if limit_reached
    dy = pinv(J)*v;
end

dy_prev = dy;