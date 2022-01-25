clc; clear all; close all;

global l1 l2 l3 l4
global k1 k2 k3 k4 k5 k6 k7
global c1_d c2_d c3_d c4_d c5_d c6_d c7_d
global th1_star th2_star th3_star th4_star th5_star th6_star th7_star
global x_des y_des z_des
global kx kIx cx ky kIy cy kz kIz cz
global X_on Y_on Z_on
global cumErrorX cumErrorY cumErrorZ
global x_lim_1 x_lim_2 y_lim_1 y_lim_2 z_lim_1 z_lim_2
global precision
global dy_prev

udpr = dsp.UDPReceiver('LocalIPPort', 8051);
udps = dsp.UDPSender('RemoteIPPort', 8052);
udpsRobot = dsp.UDPSender('RemoteIPAddress', '134.184.20.129', 'RemoteIPPort', 8053);
udpsRobot2 = dsp.UDPSender('RemoteIPAddress', '134.184.20.129', 'RemoteIPPort', 8056);
udprRobot = dsp.UDPReceiver('LocalIPPort', 8054);
udprRobot2 = dsp.UDPReceiver('LocalIPPort', 8055);
setup(udpr);

x_lim_1 = -0.7;
x_lim_2 = -0.2;
y_lim_1 = -0.8;
y_lim_2 = 0;
z_lim_1 = 0.3;
z_lim_2 = 0.7;

precision = 0.01;
x_des = 0.1;
y_des = -0.4;
z_des = 0;

if x_des < x_lim_1
    x_des = x_lim_1;
end

if x_des > x_lim_2
    x_des = x_lim_2;
end

if y_des < y_lim_1
    y_des = y_lim_1;
end

if y_des > y_lim_2
    y_des = y_lim_2;
end

if z_des < z_lim_1
    z_des = z_lim_1;
end

if z_des > z_lim_2
    z_des = z_lim_2;
end

X_on = 0;
Y_on = 0;
Z_on = 0;

th1_0 = 0;
th2_0 = 0;
th3_0 = 0;
th4_0 = 0;
th5_0 = 0;
th6_0 = 0;
th7_0 = 0;

th1_star = 0;
th2_star = 0;
th3_star = 0;
th4_star = 0;
th5_star = 0;
th6_star = 0;
th7_star = 80*pi/180;

l1 = 0.58;
l2 = 0.2;
l3 = 0.31;
l4 = 0.28;

k1 = 2000;
k2 = 2000;
k3 = 2000;
k4 = 100;
k5 = 100;
k6 = 100;
k7 = 100;

c1_d = 1;
c2_d = 1;
c3_d = 1;
c4_d = 1;
c5_d = 1;
c6_d = 1;
c7_d = 1;

kx = 200;
ky = 200;
kz = 200;
kIx = 5;
kIy = 5;
kIz = 5;
cx = 0;
cy = 0;
cz = 0;

cumErrorX = 0;
cumErrorY = 0;
cumErrorZ = 0;

dy_prev = zeros(7,1);

offset = 0.000001*ones(7,1);

th0_prev = zeros(7,1);

executed = 0;

nextStep = 1;

step = 0;

time = 0;
step_rec = zeros(1, 30);
th0_rec = zeros(7,4000);
Tee_init_rec = zeros(4,4,2000);
Tee_rec = zeros(4,4,2000);
Tee_robot_rec = zeros(4,4,2000);
errorX_rec = zeros(1,4000);
errorY_rec = zeros(1,4000);
errorZ_rec = zeros(1,4000);
score1_rec = zeros(10,4000);
score2_rec = zeros(10,4000);
score3_rec = zeros(10,4000);
opt_post_unity_rec = zeros(7,2000);
opt_post_robot_rec = zeros(7,2000);

t_step_rec = zeros(1, 1000);
t_th0_rec = zeros(1, 1000);
t_Tee_init_rec = zeros(1, 1000);
t_Tee_rec = zeros(1, 1000);
t_Tee_robot_rec = zeros(1, 1000);
t_errorX_rec = zeros(1, 1000);
t_errorY_rec = zeros(1, 1000);
t_errorZ_rec = zeros(1, 1000);
t_score1_rec = zeros(1,1000);
t_score2_rec = zeros(1,1000);
t_score3_rec = zeros(1,1000);
t_opt_post_unity_rec = zeros(1,1000);
t_opt_post_robot_rec = zeros(1,1000);

i_step = 1;
i_th0 = 1;
i_Tee_init = 1;
i_Tee = 1;
i_Tee_robot = 1;
i_errorX = 1;
i_errorY = 1;
i_errorZ = 1;
i_score1 = 1;
i_score2 = 1;
i_score3 = 1;
i_opt_post_unity = 1;
i_opt_post_robot = 1;

tic
i=1;
while 1
    if(step > 12)
        break;
    end
%     if(step == 0)
%         while 1
%             dataReceivedRobot2 = udprRobot2();
%             if ~isempty(dataReceivedRobot2)
%                 break;
%             end
%             pause(0.1);
%         end
%         udpsRobot(typecast(int32(nextStep),'uint8'));
%         step = step + 1;
%         step_rec(i_step) = step;
%         t_step_rec(i_step) = toc;
%         i_step = i_step + 1;
%         step = 7;
%     end
    
    data = [];
    dataSent = [];
    dataSentRobot = [];
    dataReceivedRobot = [];
    dataReceived = udpr();
    dataReceivedRobot = udprRobot();
    data = typecast( dataReceived , 'single');
    th0 = zeros(7,1);
    y = zeros(1,7);
    yRobot = zeros(1,7);
    T_ee_init = zeros(4,4);
    T_ee = zeros(4,4);
    T_ee_robot = zeros(4,4);
    score1 = zeros(10,1);
    score2 = zeros(10,1);
    score3 = zeros(10,1);
    errorX = -300;
    errorY = -300;
    errorZ = -300;
    
    dataReceivedRobot2 = udprRobot2();
    if ~isempty(dataReceivedRobot2)
        if(dataReceivedRobot2(1) ~= 'R')
            %if(step ~=2 && step~=9)
            step = step + 1;
            
            step_rec(i_step) = step;
            t_step_rec(i_step) = toc;
            i_step = i_step + 1;
            %end
            udpsRobot(typecast(int32(nextStep),'uint8'));
            pause(1.0);
        else
            step = 0;
        end
    end
    
    if ~isempty(data)
        th0(1) = double(data(1))*pi/180;
        th0(2) = double(data(2))*pi/180;
        th0(3) = -double(data(3))*pi/180;
        th0(4) = double(data(4))*pi/180;
        th0(5) = double(data(5))*pi/180;
        th0(6) = -double(data(6))*pi/180;
        th0(7) = double(data(7))*pi/180;
        
        th0_rec(:, i_th0) = th0;
        t_th0_rec(i_th0) = toc;
        i_th0 = i_th0 + 1;
        
        score1 = REBA(th0')';
        
        score1_rec(:, i_score1) = score1;
        t_score1_rec(i_score1) = toc;
        i_score1 = i_score1 + 1;
        
        T_ee_init = getTransf(th0(1),th0(2),th0(3),th0(4),th0(5),th0(6),th0(7));
        x_des = T_ee_init(1,4);
        y_des = T_ee_init(2,4);
        z_des = T_ee_init(3,4);
        
        Tee_init_rec(:, :, i_Tee_init) = T_ee_init;
        t_Tee_init_rec(i_Tee_init) = toc;
        i_Tee_init = i_Tee_init + 1;
        
        x_init = T_ee_init(1,4);
        y_init = T_ee_init(2,4);
        z_init = T_ee_init(3,4);
        
        T_ee_t = getTransf(th0_prev(1),th0_prev(2),th0_prev(3),th0_prev(4),th0_prev(5),th0_prev(6),th0_prev(7));
        
        errorX = T_ee_t(1,4) - x_des;
        errorY = T_ee_t(2,4) - y_des;
        errorZ = T_ee_t(3,4) - z_des;
        error = sqrt(X_on*errorX^2 + Y_on*errorY^2 + Z_on*errorZ^2);
        
        errorX_rec(i_errorX) = errorX;
        errorY_rec(i_errorY) = errorY;
        errorZ_rec(i_errorZ) = errorZ;
        t_errorX_rec(i_errorX) = toc;
        t_errorY_rec(i_errorY) = toc;
        t_errorZ_rec(i_errorZ) = toc;
        i_errorX = i_errorX + 1;
        i_errorY = i_errorY + 1;
        i_errorZ = i_errorZ + 1;
        
        X_on = 1;
        Y_on = 1;
        Z_on = 1;
        
        if error >= precision %|| (X_on == 0 && Y_on == 0 && Z_on == 0)
            [t,y] = solveODE_mex([0,0,0,0,0,0,0],offset');
            score2 = REBA([y(end,1),y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7)])';
            
            score2_rec(:, i_score2) = score2;
            t_score2_rec(i_score2) = toc;
            i_score2 = i_score2 + 1;
            
            dataSent = [typecast(int32(y(end,1)*180/pi),'uint8'), typecast(int32(y(end,2)*180/pi),'uint8'), ...
                typecast(int32(-y(end,3)*180/pi),'uint8'), typecast(int32(y(end,4)*180/pi),'uint8'), typecast(int32(y(end,5)*180/pi),'uint8'), ...
                typecast(int32(-y(end,6)*180/pi),'uint8'), typecast(int32(y(end,7)*180/pi),'uint8'), typecast(int32(0),'uint8'), ...
                typecast(int32(0),'uint8'), typecast(int32(0),'uint8'), typecast(int32(0),'uint8')];
            
            udps(dataSent);
            
            opt_post_unity_rec(:, i_opt_post_unity) = [y(end,1),y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7)]';
            t_opt_post_unity_rec(i_opt_post_unity) = toc;
            i_opt_post_unity = i_opt_post_unity + 1;
            
            th0_prev = [y(end,1),y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7)];
            
            T_ee = getTransf(y(end,1),y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7));
            
            Tee_rec(:, :, i_Tee) = T_ee;
            t_Tee_rec(i_Tee) = toc;
            i_Tee = i_Tee + 1;
            
            if(step==2 || step == 8)
                X_on = 0;
                Y_on = 0;
                Z_on = 0;
                
                %human_model(th0(1),th0(2),th0(3),th0(4),th0(5),th0(6),th0(7));
                
                [t,yRobot] = solveODE_mex([0,0,0,0,0,0,0],offset');
                score3 = REBA([yRobot(end,1),yRobot(end,2),yRobot(end,3),yRobot(end,4),yRobot(end,5),y(end,6),yRobot(end,7)])';
                
                score3_rec(:, i_score3) = score3;
                t_score3_rec(i_score3) = toc;
                i_score3 = i_score3 + 1;
                
                T_ee_robot = getTransf(yRobot(end,1),yRobot(end,2),yRobot(end,3),yRobot(end,4),yRobot(end,5),yRobot(end,6),yRobot(end,7));
                
                dataSentRobot = [typecast(T_ee_robot(1,4)-x_init,'uint8'), typecast(T_ee_robot(2,4)-y_init,'uint8'), typecast(T_ee_robot(3,4)-z_init,'uint8')];
                udpsRobot2(dataSentRobot);
                
                Tee_robot_rec(:, :, i_Tee_robot) = T_ee_robot;
                t_Tee_robot_rec(i_Tee_robot) = toc;
                i_Tee_robot = i_Tee_robot + 1;
                
                opt_post_robot_rec(:, i_opt_post_robot) = [yRobot(end,1),yRobot(end,2),yRobot(end,3),yRobot(end,4),yRobot(end,5),yRobot(end,6),yRobot(end,7)]';
                t_opt_post_robot_rec(i_opt_post_robot) = toc;
                i_opt_post_robot = i_opt_post_robot + 1;
                
                [T_ee_robot(1,4)-x_init, T_ee_robot(2,4)-y_init, T_ee_robot(3,4)-z_init]
                
                %human_model(yRobot(end,1),yRobot(end,2),yRobot(end,3),yRobot(end,4),yRobot(end,5),yRobot(end,6),yRobot(end,7));
                
                %                 step = step + 1;
                %
                %                 step_rec(i_step) = step;
                %                 t_step_rec(i_step) = toc;
                %                 i_step = i_step + 1;
                
                while 1
                    dataReceivedRobot2 = udprRobot2();
                    if ~isempty(dataReceivedRobot2)
                        if(dataReceivedRobot2(1) ~= 'R')

                            step = step + 1;
                            
                            step_rec(i_step) = step;
                            t_step_rec(i_step) = toc;
                            i_step = i_step + 1;
                
                            udpsRobot(typecast(int32(nextStep),'uint8'));
                            pause(1.0);
                        else
                            step = 0;
                        end
                        
                        break;
                    end
                end
            end
            
        end
        
        %if ~isempty(dataReceivedRobot)
        %    human_model(th0(1),th0(2),th0(3),th0(4),th0(5),th0(6),th0(7));
        %end
        
    end
    
    pause(0.1);
end

release(udpr);
release(udprRobot);
release(udprRobot2);
release(udps);
release(udpsRobot);
release(udpsRobot2);
