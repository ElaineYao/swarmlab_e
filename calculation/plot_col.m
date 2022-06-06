%% Plot velocity

% M = readmatrix('/media/EDrive/swarmlab_e/vel_y_mat_1.csv');
% v_cmd = 2;
% v_rel = 3;
% v_rel1 = 4;
% v_rel2 = 5;
% v_fric = 6;
% v_o = 7;
% v_g = 8;
% 
% start = 1;
% % end_r = 1550;
% [end_r,] = size(M);
% x = M(start:end_r,1);
% 
% figure
% % plot(x, M(start:end_r,v_rel), x, M(start:end_r,v_fric), x, M(start:end_r,v_o), x, M(start:end_r,v_g));
% % 
% % hold off
% % legend('v_{rel}', 'v_{fric}', 'v_{o}', 'v_{g}')
% 
% % plot(x, M(start:end_r,v_cmd), x, M(start:end_r,v_rel));
% % 
% % hold off
% % legend('v_{cmd}', 'v_{rel}')
% 
% % plot(x, M(start:end_r,v_rel), x, M(start:end_r,v_rel1), x, M(start:end_r,v_rel2));
% % 
% % hold off
% % legend('v_{rel}', 'v_{rel1}', 'v_{rel2}')
% 
% % plot(x, M(start:end_r,v_cmd), x, M(start:end_r,v_rel), x, M(start:end_r,v_rel1), x, M(start:end_r,v_rel2), x, M(start:end_r,v_fric), x, M(start:end_r,v_o), x, M(start:end_r,v_g));
% % 
% % hold off
% % legend('v_{cmd}','v_{rel}', 'v_{rel1}', 'v_{rel2}', 'v_{fric}', 'v_{o}', 'v_{g}')
% 
% plot(x, M(start:end_r,v_cmd), x, M(start:end_r,v_rel), x, M(start:end_r,v_fric), x, M(start:end_r,v_o), x, M(start:end_r,v_g));
% 
% hold off
% legend('v_{cmd}','v_{rel}', 'v_{fric}', 'v_{o}', 'v_{g}')

%% Plot location

% M = readmatrix('/media/EDrive/swarmlab_e/vel_1_cal.csv');
% start = 1;
% % end_r = 500;
% [end_r,] = size(M);
% pos_x = M(start:end_r,3);
% pos_y = M(start:end_r,2);
% plot(pos_x, pos_y);
% 
% hold on
% 
% obs_x = [150.0000  163.2583  168.7500  163.2583  150.0000  136.7417  131.2500  136.7417  150.0000 150.0000  163.2583  168.7500  163.2583  150.0000  136.7417  131.2500  136.7417  150.0000];
% obs_y = [18.7500   13.2583    0.0000  -13.2583  -18.7500  -13.2583   -0.0000   13.2583   18.7500 18.7500   13.2583    0.0000  -13.2583  -18.7500  -13.2583   -0.0000   13.2583   18.7500];
% 
% plot(obs_x, obs_y);
% 
% % gx = [160 135.972];
% % gy = [180 -15.5851];
% % 
% % hold on
% % plot(gx, gy);
% 
% legend('drone 1');

%% Plot two vy_rel
% M = readmatrix('/media/EDrive/swarmlab_e/vel_y_mat_1.csv');
% N = readmatrix('/media/EDrive/swarmlab_e/vel_y_mat_1_brown.csv');
% 
% x1 = M(:, 1);
% y1 = M(:, 2);
% x2 = N(:, 1);
% y2 = N(:, 2);
% 
% plot(x1, y1, x2, y2);
% hold off
% legend("blue", "brown")

%% Plot velocity vectors
% M = readmatrix("/media/EDrive/swarmlab_e/vel_1_xy.csv");
% [rows,] = size(M);
% x = zeros(1);
% y = zeros(1);
% % rows = 2;
% unit_v = 1;
% for i =1:rows
%     max_v = max(abs(M(i,2:end)));
%     % for vel_cmd
% %     x(2*i-1) = unit_v*(i-1); % vector start
% %     x(2*i) = (M(i,2)/max_v)*unit_v;
% %     y(2*i-1) = unit_v*(i-1);
% %     y(2*i) = (M(i,3)/max_v)*unit_v;
% %     vector_x =[x(2*i-1), x(2*i)];
% %     vector_y =[y(2*i-1), y(2*i)];
%     
%     x_start = 0; % vector start
%     x(2*i) = (M(i,2)/max_v)*unit_v;
%     y_start = 0;
%     y(2*i) = (M(i,3)/max_v)*unit_v;
%     
%     plot([x_start, M(i,2)], [y_start, M(i,3)], [x_start, M(i,4)], [y_start, M(i,5)], [x_start, M(i,6)], [y_start, M(i,7)], [x_start, M(i,8)], [y_start, M(i,9)], [x_start, M(i,10)], [y_start, M(i,11)]);
%     hold off
%     legend("v_{cmd}","v_{rep}", "v_{fric}", "v_{obs}", "v_{g}")
%     
%     haha = 0;
%     
% end


%% Plot velocity and position along with the obstacle
M = readmatrix('/media/EDrive/swarmlab_e/vel_1_cal.csv');

obs_x = [150.0000  163.2583  168.7500  163.2583  150.0000  136.7417  131.2500  136.7417  150.0000 150.0000  163.2583  168.7500  163.2583  150.0000  136.7417  131.2500  136.7417  150.0000];
obs_y = [18.7500   13.2583    0.0000  -13.2583  -18.7500  -13.2583   -0.0000   13.2583   18.7500 18.7500   13.2583    0.0000  -13.2583  -18.7500  -13.2583   -0.0000   13.2583   18.7500];


% start = 1;
% % end_r = 500;
% [end_r,] = size(M);
% pos_x = M(start:end_r,3);
% pos_y = M(start:end_r,2);
% plot(pos_x, pos_y);

N = readmatrix("/media/EDrive/swarmlab_e/vel_1_xy.csv");
[rows,] = size(M);
x = zeros(1);
y = zeros(1);
% rows = 2;
unit_v = 1;
for i =1:rows
    max_v = max(abs(N(i,2:end)));
    % for vel_cmd
%     x(2*i-1) = unit_v*(i-1); % vector start
%     x(2*i) = (M(i,2)/max_v)*unit_v;
%     y(2*i-1) = unit_v*(i-1);
%     y(2*i) = (M(i,3)/max_v)*unit_v;
%     vector_x =[x(2*i-1), x(2*i)];
%     vector_y =[y(2*i-1), y(2*i)];
    
    x_start = M(i+2000,3); % vector start
    x(2*i) = (N(i,2)/max_v)*unit_v;
    y_start = M(i+2000,2);
    y(2*i) = (N(i,3)/max_v)*unit_v;
    rate = 1;
    
    plot(obs_x, obs_y);
    hold on
    plot([x_start, x_start+N(i,2)/rate], [y_start, y_start+N(i,3)/rate], [x_start, x_start+N(i,4)/rate], [y_start, y_start+N(i,5)/rate], [x_start, x_start+N(i,6)/rate], [y_start, y_start+N(i,7)/rate], [x_start, x_start+N(i,8)/rate], [y_start, y_start+N(i,9)/rate], [x_start, x_start+N(i,10)/rate], [y_start, y_start+N(i,11)/rate]);
    hold off
    legend("obstacle","v_{cmd}","v_{rep}", "v_{fric}", "v_{obs}", "v_{g}")
    
    haha = 0;
    
end



