% syms delta_x delta_y delta_vx delta_vy;
syms  delta_vx delta_vy;

delta_x = linspace(0,1, 10000);
% vars
b = 0.05;
delta_y = 0.05;
x1 = 88.0194;
y1 = 150.0408;
x2 = 83.4799;
y2 = 149.9551;
vx_1 = -0.6132;
vy_1 = 0.0507;
vx_2 = -0.2485;
vy_2 = -0.0527;
loc1 = [x1, y1]; % loc of target drone
vel1 = [vx_1, vy_1]; % vel of target drone
loc2 = [x2, y2]; % attacked drone
vel2 = [vx_2, vy_2]; % vel of attacked drone
delta_loc = [delta_x, delta_y]; % gps attack
delta_vel = [delta_vx, delta_vy]; % IMU attack
loc2_att = loc2+delta_loc; % attacked location 
vel2_att = vel2+delta_vel; % attacked velocity
% vel_obstacle
vx_o = -7.3231;
vy_o = -0.0158;
% vel_target
vx_g = 5.9997;
vy_g = 0.0549;
% vel_rep
dist_l = sqrt(sum((loc2_att-loc1).^2));
diff_l = loc1 - loc2_att;
vx_rel = 0.03*(25./(dist_l)-1).*diff_l(1,1);
vy_rel = 0.03.*(25/(dist_l)-1).*diff_l(1,2);
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

vx = vx_o+vx_g+vx_rel+vx_fric
vy = vy_o+vy_g+vy_rel+vy_fric


plot(delta_x,vx);

