% syms delta_x delta_y delta_vx delta_vy;
% syms delta_x delta_y delta_vx delta_vy;
% syms x1 y1 x2 y2 vx_1 vy_1 vx_2 vy_2 vx_o vy_o vx_g vy_g;
syms delta_y

% vars
num_agents = 3;

M = readmatrix('~/swarmlab_e/vel_1_cal.csv');
m_size = size(M);
row = m_size(1,1);
% 1st drone - target drone; 2nd drone - attacked drone
% pos_mat: 2*num_agents, store position(x,y) of each agent
pos_mat = zeros(2,num_agents);
% vel_mat: 2*num_agents, store velocity(vx,vy) of each agent
vel_mat = zeros(2,num_agents);
% vo:[vo_x;vo_y] 2*1, store the velocity generated by the obstacle, vel_obstacle
vo = zeros(2,1);
% vo:[vg_x;vg_y] 2*1, store the velocity generated by the goal, vel_goal
vg = zeros(2,1);

% attack parameters
delta_x = 0;
% delta_y = 0;
delta_vx = 0;
delta_vy = 0;

% for row_idx = 1:row
for row_idx = 11:11
    for agent = 1: num_agents
        pos_mat(:,agent) = [M(row_idx, 2*agent); M(row_idx, 2*agent+1)];
        vel_mat(:,agent) = [M(row_idx, 2*num_agents+2*agent); M(row_idx, 2*num_agents+2*agent+1)];
%         % compensate for GPS attack
%         pos_mat(:,2) = pos_mat(:,2)+[0;9.8]
    end
    vo = [M(row_idx, num_agents*4+2);M(row_idx, num_agents*4+3)];
    vg = [M(row_idx, num_agents*4+4);M(row_idx, num_agents*4+5)];
%     x1 = M(row_idx, 2);
%     y1 = M(row_idx, 3);
%     x2 = M(row_idx, 4);
%     y2 = M(row_idx, 5);
%     vx_1 = M(row_idx, 6);
%     vy_1 = M(row_idx, 7);
%     vx_2 = M(row_idx, 8);
%     vy_2 = M(row_idx, 9);

%     loc1 = pos_mat(:,1); % loc of target drone
%     vel1 = vel_mat(:,1); % vel of target drone
%     loc2 = pos_mat(:,2); % attacked drone
%     vel2 = vel_mat(:,2); % vel of attacked drone
%     delta_loc = [delta_x, delta_y]; % gps attack
%     delta_vel = [delta_vx, delta_vy]; % IMU attack
%     loc2_att = loc2+delta_loc; % attacked location 
%     vel2_att = vel2+delta_vel; % attacked velocity
% %     % vel_obstacle
% %     vx_o = M(row_idx, 10);
% %     vy_o = M(row_idx, 11);
% %     % vel_target
% %     vx_g = M(row_idx, 12);
% %     vy_g = M(row_idx, 13);
%     % vel_rep
%     dist_l = sqrt(sum((loc2_att-loc1).^2));
%     diff_l = loc1 - loc2_att;
%     
%     % when dist_1>25
%     vx_rel = 0.03*(1-25/(dist_l))*diff_l(1,1);
%     vy_rel = 0.03*(1-25/(dist_l))*diff_l(1,2);
%     for agent2 = 3:num_agents
%         dist_agent2 = sqrt(sum((pos_mat(:,agent2)-loc1).^2));
%         diff_agent2 = loc1 - pos_mat(:,agent2);
%         vx_rel = 0.03*(1-25/(dist_agent2))*diff_agent2(1,1);
%         vy_rel = 0.03*(1-25/(dist_agent2))*diff_agent2(1,2);
%     end
%     
%     % vel_fric
%     dist_v = sqrt(sum((vel2_att-vel1).^2));
%     diff_v = vel1 - vel2_att;
%     % set v_fric = 0
%     dist_v = 0.1;
%     if (dist_v < 0.63)
%     vx_fric = 0;
%     vy_fric = 0;
%     else
%     vx_fric = 0.05*(1-0.63/dist_v)*diff_v(1,1);
%     vy_fric = 0.05*(1-0.63/dist_v)*diff_v(1,2);
%     end
   
    

    % ------------uncomment when u have the expression for vx ---------------
%     delta_x = linspace(-15,15, 20000);
    % delta_x = 4.5385; % vx = -0.5734
    vx_g = vg(1,1);
    vx_o = vo(1,1);
    vy_g = vg(2,1);
    vy_o = vo(2,1);
    x1 = pos_mat(1,1);
    y1 = pos_mat(2,1);
%     % GPS attack the 2nd drone version
%     x3 = pos_mat(1,3);
%     y3 = pos_mat(2,3);
%     x2 = pos_mat(1,2);
%     y2 = pos_mat(2,2);
    % GPS attack the 3rd drone version
    x3 = pos_mat(1,2);
    y3 = pos_mat(2,2);
    x2 = pos_mat(1,3);
    y2 = pos_mat(2,3);

    vx = vx_g + vx_o + (3/(4*((x1 - x3)^2 + (y1 - y3)^2)^(1/2)) - 3/100)*(x1 - x3) - (3/(4*((delta_x - x1 + x2)^2 + (delta_y - y1 + y2)^2)^(1/2)) - 3/100)*(delta_x - x1 + x2);
    vy = vy_g + vy_o + (3/(4*((x1 - x3)^2 + (y1 - y3)^2)^(1/2)) - 3/100)*(y1 - y3) - (3/(4*((delta_x - x1 + x2)^2 + (delta_y - y1 + y2)^2)^(1/2)) - 3/100)*(delta_y - y1 + y2);
% 
%     v_mat = [vx, vy];
% ------------
%     vx_rel = (3/(4*((delta_x - x1 + x2)^2 + (delta_y - y1 + y2)^2)^(1/2)) - 3/100)*(delta_x - x1 + x2) - (3/(4*((x1 - x4)^2 + (y1 - y4)^2)^(1/2)) - 3/100)*(x1 - x4) - (3/(4*((x1 - x3)^2 + (y1 - y3)^2)^(1/2)) - 3/100)*(x1 - x3);
%     v_mat = [vx_rel];
% -----------
%     vx_rel2 = (3/(4*((delta_x - x1 + x2)^2 + (delta_y - y1 + y2)^2)^(1/2)) - 3/100)*(delta_x - x1 + x2);
%     vx_rel3 = -(3/(4*((x1 - x3)^2 + (y1 - y3)^2)^(1/2)) - 3/100)*(x1 - x3);
%     vx_rel4 = -(3/(4*((x1 - x4)^2 + (y1 - y4)^2)^(1/2)) - 3/100)*(x1 - x4);
    
%     vx_rel = vx_rel2 + vx_rel3 + vx_rel4;
    vx_rel = (3/(4*((x1 - x3)^2 + (y1 - y3)^2)^(1/2)) - 3/100)*(x1 - x3) - (3/(4*((delta_x - x1 + x2)^2 + (delta_y - y1 + y2)^2)^(1/2)) - 3/100)*(delta_x - x1 + x2);
    % solve vy_rel = 0
    delta_y = linspace(-15,0,2000);
%     delta_y = 0;
    vy_rel = (3/(4*((x1 - x3)^2 + (y1 - y3)^2)^(1/2)) - 3/100)*(y1 - y3) - (3./(4*((delta_x - x1 + x2)^2 + (delta_y - y1 + y2).^2).^(1/2)) - 3/100).*(delta_y - y1 + y2)+0.25;
    plot( delta_y, vy_rel);
    
    
%     v_mat = [vx vy vx_rel vy_rel vo' vg'];

%     writematrix(v_mat,'vel_my.csv','Delimiter',',', 'WriteMode','append');
end 


