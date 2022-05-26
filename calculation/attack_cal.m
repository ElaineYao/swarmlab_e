% syms delta_x delta_y delta_vx delta_vy;
syms delta_x delta_y delta_vx delta_vy;
syms x1 y1 x2 y2 vx_1 vy_1 vx_2 vy_2 vx_o vy_o vx_g vy_g;

% vars
M = readmatrix('/media/EDrive/swarmlab_e/vel_1_cal.csv');
row_idx = 2;
x1 = M(row_idx, 2);
y1 = M(row_idx, 3);
x2 = M(row_idx, 4);
y2 = M(row_idx, 5);
vx_1 = M(row_idx, 6);
vy_1 = M(row_idx, 7);
vx_2 = M(row_idx, 8);
vy_2 = M(row_idx, 9);

loc1 = [x1, y1]; % loc of target drone
vel1 = [vx_1, vy_1]; % vel of target drone
loc2 = [x2, y2]; % attacked drone
vel2 = [vx_2, vy_2]; % vel of attacked drone
delta_loc = [delta_x, delta_y]; % gps attack
delta_vel = [delta_vx, delta_vy]; % IMU attack
loc2_att = loc2+delta_loc; % attacked location 
vel2_att = vel2+delta_vel; % attacked velocity
% vel_obstacle
vx_o = M(row_idx, 10);
vy_o = M(row_idx, 11);
% vel_target
vx_g = M(row_idx, 12);
vy_g = M(row_idx, 13);
% vel_rep
dist_l = sqrt(sum((loc2_att-loc1).^2));
diff_l = loc1 - loc2_att;
vx_rel = 0.03*(1-25/(dist_l))*diff_l(1,1);
vy_rel = 0.03*(1-25/(dist_l))*diff_l(1,2);
% vel_fric
dist_v = sqrt(sum((vel2_att-vel1).^2));
diff_v = vel1 - vel2_att;
% set v_fric = 0
dist_v = 0.1;
if (dist_v < 0.63)
    vx_fric = 0;
    vy_fric = 0;
else
    vx_fric = 0.05*(1-0.63/dist_v)*diff_v(1,1);
    vy_fric = 0.05*(1-0.63/dist_v)*diff_v(1,2);
end

% vx = vx_o+vx_g+vx_rel+vx_fric;
% vx = subs(vx, {(delta_y -y1+y2), (vx_g+vx_o), (x2-x1)}, {sym('b'), sym('a'), sym('c')})
% pretty(vx)

% vy = vy_o+vy_g+vy_rel+vy_fric
% vy = subs(vy, {(delta_y -y1+y2), (vy_g+vy_o), (x2-x1)}, {sym('b'), sym('m'), sym('c')});

% ------------uncomment when u have the expression for vx ---------------
% Now I have the value for a b c
a = vx_g+vx_o
b = y2-y1
epsilon = 0;
c = x2-x1+epsilon
delta_x = linspace(-15,15, 20000);
% delta_x = 0;
% delta_x = 4.5385; % vx = -0.5734
vx = a - (c + delta_x).*(3./(4.*((c + delta_x).^2 + b^2 ).^(1/2)) - 3/100);

plot(delta_x,vx);


% max_f = -100;
% max_b = -100;
% max_x = 0;
% for b = -15:0.01:15
%     
%     vx = a - (c + delta_x).*(3./(4.*((c + delta_x).^2 + b^2).^(1/2)) - 3/100);
%     f = @(delta_x) vx;
% %     plot(delta_x,vx);
%     [max_v,idx] = max(f(delta_x));
%    
%     if max_v>max_f
%         max_f = max_v;
%         max_b = b;
%         max_x = delta_x(idx);
%     end
% end
% 
% disp(max_f)
% disp(max_b)
% disp(max_x)
% 
% % % ------------uncomment when u have the expression for vy ---------------
% m = vy_g+vy_o;
% b = 0;
% % c = x2-x1;
% delta_x = 1; % vy = 0.0391
% vy = m - b*(3/(4*((c + delta_x)^2 + b^2)^(1/2)) - 3/100) 
% 
% 
