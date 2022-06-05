
M = readmatrix('/media/EDrive/swarmlab_e/vel_y_mat_1.csv');
v_cmd = 2;
v_rel = 3;
v_rel1 = 4;
v_rel2 = 5;
v_fric = 6;
v_o = 7;
v_g = 8;

start = 1;
% end_r = 1550;
[end_r,] = size(M);
x = M(start:end_r,1);

figure
% plot(x, M(start:end_r,v_rel), x, M(start:end_r,v_fric), x, M(start:end_r,v_o), x, M(start:end_r,v_g));
% 
% hold off
% legend('v_{rel}', 'v_{fric}', 'v_{o}', 'v_{g}')

% plot(x, M(start:end_r,v_cmd), x, M(start:end_r,v_rel));
% 
% hold off
% legend('v_{cmd}', 'v_{rel}')

% plot(x, M(start:end_r,v_rel), x, M(start:end_r,v_rel1), x, M(start:end_r,v_rel2));
% 
% hold off
% legend('v_{rel}', 'v_{rel1}', 'v_{rel2}')

% plot(x, M(start:end_r,v_cmd), x, M(start:end_r,v_rel), x, M(start:end_r,v_rel1), x, M(start:end_r,v_rel2), x, M(start:end_r,v_fric), x, M(start:end_r,v_o), x, M(start:end_r,v_g));
% 
% hold off
% legend('v_{cmd}','v_{rel}', 'v_{rel1}', 'v_{rel2}', 'v_{fric}', 'v_{o}', 'v_{g}')

plot(x, M(start:end_r,v_cmd), x, M(start:end_r,v_rel), x, M(start:end_r,v_fric), x, M(start:end_r,v_o), x, M(start:end_r,v_g));

hold off
legend('v_{cmd}','v_{rel}', 'v_{fric}', 'v_{o}', 'v_{g}')