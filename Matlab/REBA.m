function reba_score = REBA(q)

q = q*180/pi;

reba_A_table = zeros(3,5,4);

reba_A_table(1,:,:) = [[1, 2, 3, 4]; [2, 3, 4, 5]; ...
    [2, 4, 5, 6]; [3, 5, 6, 7]; ...
    [4, 6, 7, 8]];
reba_A_table(2,:,:) = [[1, 2, 3, 4]; [3, 4, 5, 6];
    [4, 5, 6, 7]; [5, 6, 7, 8];
    [6, 7, 8, 9]];
reba_A_table(3,:,:) = [[3, 3, 5, 6]; [4, 5, 6, 7];
    [5, 6, 7, 8]; [6, 7, 8, 9];
    [7, 7, 8, 9]];

reba_B_table = zeros(2, 6, 3);

reba_B_table(1,:,:) = [[1, 2, 2]; [1, 2, 3];
    [3, 4, 5]; [4, 5, 5];
    [6, 7, 8]; [7, 8, 8]];
reba_B_table(2,:,:) = [[1, 2, 3]; [2, 3, 4];
    [4, 5, 5]; [5, 6, 7];
    [7, 8, 8]; [8, 9, 9]];

reba_C_table = zeros(12, 12);

reba_C_table(1,:) = [1, 1, 1, 2, 3, 3, 4, 5, 6, 7, 7, 7];
reba_C_table(2,:) = [1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 7, 8];
reba_C_table(3,:) = [2, 3, 3, 3, 4, 5, 6, 7, 7, 8, 8, 8];
reba_C_table(4,:) = [3, 4, 4, 4, 5, 6, 7, 8, 8, 9, 9, 9];
reba_C_table(5,:) = [4, 4, 4, 5, 6, 7, 8, 8, 9, 9, 9, 9];
reba_C_table(6,:) = [6, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 10];
reba_C_table(7,:) = [7, 7, 7, 8, 9, 9, 9, 10, 10, 11, 11, 11];
reba_C_table(8,:) = [8, 8, 8, 9, 10, 10, 10, 10, 10, 11, 11, 11];
reba_C_table(9,:) = [9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12];
reba_C_table(10,:) = [10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12];
reba_C_table(11,:) = [11, 11, 11, 11, 12,12, 12, 12, 12, 12, 12, 12];
reba_C_table(12,:) = [12, 12, 12, 12, 12,12, 12, 12, 12, 12, 12, 12];

% Neck score
n = 1;

% Trunk score
t = 1;
if q(1) < -10 || q(1) >  3
    t = 2;
end 

if q(1) >= 20
    t = 3;
end

if q(3) > 30 || q(3) < -30
    t = t + 1;
end

if q(2) < -30/2 || q(3) > 30/2
    t = t + 1;
end

% Leg score
l = 1;

score_A = reba_A_table(n,t,l);

% Upper arm score
ua = 1;
if q(4) < -20 || (q(4) > 20 && q(4) < 45)
    ua = 2;
end

if (q(4) >= 45 && q(4) < 90)
    ua = 3;
end

if q(4) >= 90
    ua = 4;
end

% if q(5) > 20
%     ua = ua + 1;
% end

if q(5) > 30
    ua = ua + 1;
end

% Lower arm score
la = 1;
if q(7) <= 60 || q(7) >= 100
    la = 2;
end

% Wrist score
w = 1;

% if q(11) > 15 || q(11) < -15
%     w = 2;
% end

score_B = reba_B_table(la,ua,w);


score_C = reba_C_table(score_A,score_B);

reba_score = score_C;% + 0*norm([q(1) q(2) q(3) q(5) q(6) q(7) q(9) q(11)]);

reba_score = [reba_score;score_A;score_B;score_C;n;t;l;ua;la;w];
end

