% syms delta_x delta_y delta_vx delta_vy;
syms delta_x delta_y delta_vx delta_vy;
syms x1 y1 x2 y2 x3 y3 vx_1 vy_1 vx_2 vy_2 vx_3 vy_3 vx_o vy_o vx_g vy_g;

% target drone
loc1 = [x1, y1]; % loc of target drone
vel1 = [vx_1, vy_1]; % vel of target drone
loc2 = [x2, y2]; % attacked drone
vel2 = [vx_2, vy_2]; % vel of attacked drone
delta_loc = [delta_x, delta_y]; % gps attack
delta_vel = [delta_vx, delta_vy]; % IMU attack
% attacked drone
loc2_att = loc2+delta_loc; % attacked location 
vel2_att = vel2+delta_vel; % attacked velocity
% drone 3 - brown
loc3 = [x3, y3]; 
vel3 = [vx_3, vy_3]; 

% vel_rep
dist_l = sqrt(sum((loc2_att-loc1).^2));
diff_l = loc1 - loc2_att
dist_l3 = sqrt(sum((loc3-loc1).^2));
diff_l3 = loc1 - loc3;

% when dist_1>25
vx_rel2 = 0.03*(1-25/(dist_l))*-diff_l(1,1);
vx_rel3 = 0.03*(1-25/(dist_l3))*-diff_l3(1,1);

vx_rel = 0.03*(1-25/(dist_l))*-diff_l(1,1)+0.03*(1-25/(dist_l3))*-diff_l3(1,1);
vy_rel = 0.03*(1-25/(dist_l))*-diff_l(1,2)+0.03*(1-25/(dist_l3))*-diff_l3(1,2);
% % when dist_1<25
% vx_rel = 0.03*(25/(dist_l)-1)*diff_l(1,1);
% vy_rel = 0.03*(25/(dist_l)-1)*diff_l(1,2);
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

vx_rel
vy_rel;

vx = vx_o+vx_g+vx_rel+vx_fric;
% vx = subs(vx, {(delta_y -y1+y2), (vx_g+vx_o), (x2-x1)}, {sym('b'), sym('a'), sym('c')})
% pretty(vx);

vy = vy_o+vy_g+vy_rel+vy_fric;
% pretty(vy);
