function [vel_command, collisions] = compute_vel_vasarhelyi(self, p_swarm, r_agent, dt,time)
    % ------- dt = 0.01, r_agent = 0.5, self -> Swarm, p_swarm -> struct
    
    % VASARHELYI SWARM ALGORITHM
    % This is an implementation of the Vasarhelyi algorithm. It allows the
    % navigation of a swarm of agents in presence of obstacles and walls.
    %
    % Ref:      Vasarhelyi, Science Robotics, 2018
    % 
    % Modif:    a cohesion term has been added to make the agents get
    %           closer when they are farther than r0_rep.
    %
    % Inputs:
    %   p_swarm: swarm parameters
    %   r_agent: safety radius of agents
    %   dt: time step
    %
    % Outputs:
    %   vel_command: commanded velocities for every agent
    %   collisions: [nb_agent_collisions nb_obs_collisions min_dist_obs]
    %
    
    
    %% Initialize variables
    
    pos = self.get_pos_ned(); % 3*5 double
    vel = self.get_vel_ned(); % 3*5 double

    % Initialize variables
    nb_agents = self.nb_agents; % 5
    M = zeros(nb_agents, nb_agents);    % Neighborhood matrix 5*5 double
    dist_mat = zeros(nb_agents, nb_agents);    % Distance matrix 5*5 double
    vel_rep = zeros(3, nb_agents);      % Repulsion velocity 3*5 double
    vel_fric = zeros(3, nb_agents);     % Velocity matching velocity  3*5 double
    vel_wall = zeros(3, nb_agents);     % Arena repulsion velocity  3*5 double
    vel_obs = zeros(3, nb_agents);      % Obstacle repulsion velocity  3*5 double
    vel_command = zeros(3, nb_agents);  % Total commanded velocity  3*5 double
    
    % Elaine initialization
    temp_vel_rep = zeros(2,1);
    vel_y_rep = zeros(1, 3, 3);
    
    nb_agent_collisions = 0; % Nb of collisions among agents  
    nb_obs_collisions = 0; % Nb of collisions against obstacles
    min_dist_obs = 20; % Minimal distance between any agent and the obstacle. Set it to a relatively high value in the beginning

    
    %% Compute velocity commands
    
    %         ----- Expe 2: GPS attack start -------
%         pos(1:2,3) = pos(1:2,3)+[0;-2];

%         ----- Expe 2: GPS attack end -------

% %         ----- Expe 3: GPS attack start -------
%         if (time > 0.08) && (time <7)
%             pos(1:2,2) = pos(1:2,2)+[0;-9.8];
%         elseif time>7
%             pos(1:2,2) = pos(1:2,2)+[0;2.3];
%         end
%         if (time > 1.5) 
%             pos(1:2,2) = pos(1:2,2)+[0;-7];
% %        
%         end

%         ----- Expe 3: GPS attack end -------

%         ----- Expe 4: GPS attack start -------
%         if (time>10)
%         if (mod(100*time,20)==0)
%             pos(1:2,2) = pos(1:2,2) + [0.5;0];
%         end
%         if (mod(100*time,2)==1)
%             pos(1:2,2) = pos(1:2,2) + [-1.5;0];
%         end
%         end
%         if (time>30)
%             pos(1:2,2) = pos(1:2,2) + [-1.5;0];
%         end
        
%         ----- Expe 4: GPS attack end -------

%         ----- Expe 5: GPS attack start -------
        if (time > 0) && (time <5)
%         if (time > 3.45)
            pos(1:2,2) = pos(1:2,2) + [0;3];
        end
        if (time >=5) && (time <150)
            pos(1:2,2) = pos(1:2,2) + [0;10];
        end
%         if (time >=15) && (time <500)
%             pos(1:2,2) = pos(1:2,2) + [0;13];
%         end
%         ----- Expe 5: GPS attack end -------
    
    for agent = 1:nb_agents
        
        %% Find neighbors
%         % ----- Expe 1: GPS attack start -------
%         % Compute agent-agent distance matrix
%         p_rel = pos - pos(:, agent); % substract the agent'th column -> get the relative coord
%         dist = sqrt(sum((p_rel.^2), 1)); % sum(A, 1)-> sum in column 1*5 -> every column is the relative absolute dist to agent
%         dist_mat(agent, :) = dist; % dist matrix ->every row is the relative dist for each agent
% 
%         % Define neighbours list
%         neig_list = (1:nb_agents)'; % 5*1 => 1; 2; 3; 4; 5
%         neig_list = neig_list(dist ~= 0); % for index = 1, dist = 0, so neig_list = 2; 3; 4; 5 (get the neighbor list)
%         if dist(neig_list) > (p_swarm.r-0.01)
% %             delta = dist(neig_list)-p_swarm.r+0.01; % to control the distance to be maximum 150 
%             tar_dist = 137.5; % max_dist between agents
%             delta = dist(neig_list)-tar_dist+0.01;
%             pos(1,2) = pos(1,2) - delta - 0.01;
%         end
%        
%         % ----- Expe 1: GPS attack end ---------


        
        % Compute agent-agent distance matrix
        p_rel = pos - pos(:, agent); % substract the agent'th column -> get the relative coord
        dist = sqrt(sum((p_rel.^2), 1)); % sum(A, 1)-> sum in column 1*5 -> every column is the relative absolute dist to agent
        dist_mat(agent, :) = dist; % dist matrix ->every row is the relative dist for each agent

        % Define neighbours list
        neig_list = (1:nb_agents)'; % 5*1 => 1; 2; 3; 4; 5
        neig_list = neig_list(dist ~= 0); % for index = 1, dist = 0, so neig_list = 2; 3; 4; 5 (get the neighbor list)
        
        % Count collisions
        % r_agent is the radius of two agents. if the dist is smaller than
        % 2*0.5 then collision happens. -1 is bc minus itself's dist
        
        nb_agent_collisions = nb_agent_collisions + sum(dist < 2 * r_agent) - 1; 

        % Initialize number of neighbours
        nb_neig = nb_agents - 1;

        % Constraint on neighborhood given by the euclidean distance
        if isfield(p_swarm, 'r') % determine if a field is in the struct 
            % No place to create p_swarm???
            neig_list = neig_list(dist(neig_list) < p_swarm.r); % p_swarm_r = 150, 2;3;4;5, perhaps if the dist > r, we don't think it's part of the swarm
            nb_neig = length(neig_list); % 4
        end

        % Constraint on neighborhood given by the topological distance
        if isfield(p_swarm, 'max_neig') % max_neig = 10
            if nb_neig > p_swarm.max_neig % since nb_neig = 4, I don't think it'll go into this branch
                [~, idx] = sort(dist(neig_list));
                neig_list = neig_list(idx(1:p_swarm.max_neig));
                nb_neig = p_swarm.max_neig;
            end
        end

        % Adjacency matrix (asymmetric in case of limited fov)
        M(agent, neig_list) = 1; % 5*5, but row 1, column 2-5 = 1

        
        %% Compute different contributions
        % You see, the decision of each drone is affected by all of it's
        % neighbors
        if nb_neig ~= 0
            v_rel = vel - vel(:, agent); % relative velocity
            v_rel_norm = sqrt(sum((v_rel.^2), 1)); % relative distance in velocity

            % Compute vel and pos unit vector between two agents
            p_rel_u = -p_rel ./ dist; % pos dist relative vector, tho I don't know why we should do this
            v_rel_u = -v_rel ./ v_rel_norm;

            for agent2 = neig_list' % for each of its neighbor
                
                % Repulsion and attraction
                if dist(agent2) < p_swarm.r0_rep  % repulsion compare the distance between a1&a2, if<25, repulse
                    % p_swarm.p_rep = 0.03 gain
                    % The equation is in the paper. But I know that the
                    % overall vel_rep is the sum of resulted velocity with
                    % all other agents(dist), I think the x,y shows the
                    % direction of v. It depends on the dist in x&y axis
                    
                    % Elaine - extra vel_y_cal
                    temp_vel_rep = p_swarm.p_rep * (p_swarm.r0_rep - dist(agent2)) * p_rel_u(:, agent2);
                    vel_y_rep(:, agent2, agent) = temp_vel_rep(2);
                    
                    vel_rep(:, agent) = vel_rep(:, agent) + ...
                        p_swarm.p_rep * (p_swarm.r0_rep - dist(agent2)) * p_rel_u(:, agent2);
                else  % attraction
                    
                    % Elaine - extra vel_y_cal
                    temp_vel_rep = p_swarm.p_rep * (p_swarm.r0_rep - dist(agent2)) * p_rel_u(:, agent2);
                    vel_y_rep(:, agent2, agent) = temp_vel_rep(2);
                    
                    % if > 25 then attraction
                    vel_rep(:, agent) = vel_rep(:, agent) + ...
                        p_swarm.p_rep * (dist(agent2) - p_swarm.r0_rep) *- p_rel_u(:, agent2);
                end

                % Velocity alignement
                % What are v_fric, r0_fric, a_fric, p_fric, v_fric_max -> check in
                % paper
                % get_v_max(0.63, 15.8927-85.3, 4.16, 3.2) a_fric is the
                % relative acceleration when braking
                v_fric_max = get_v_max(p_swarm.v_fric, dist(agent2) - p_swarm.r0_fric, p_swarm.a_fric, p_swarm.p_fric);
                
                % v_rel_norm is the relative distance in velocity. I don't
                % know why we'll have this
                if v_rel_norm(agent2) > v_fric_max
                    vel_fric(:, agent) = vel_fric(:, agent) + ...
                        p_swarm.C_fric * (v_rel_norm(agent2) - v_fric_max) * v_rel_u(:, agent2);
                end
%                 dlmwrite('log.csv',v_fric_max,'delimiter',',','-append');
         
            end
        end
        
        
        %% Wall and obstacle avoidance
        
        % However, in this simulation, is_actice_arena and spheres = False
        % Add arena repulsion effect - False
        if (p_swarm.is_active_arena == true)
            unit = eye(3); % identity matrix, 3 axises
            %On each axis we have the two repulsions
            for axis = 1:3
                %On each axis there is two forces (each side of the arena)
                for dir = 1:2
                    % x_arena 3*2 double [-100 100; -100 100; -100 100]
                    dist_ab = abs(pos(axis, agent) - p_swarm.x_arena(axis, dir)); % absolute dist between the current agent pos and each side of the arena
                    % v_shill = 13.6; still don't understand. model the
                    % wall has velocity?
                    %Compute velocity of wall shill agent toward center of the arena
                    v_wall_virtual = unit(:, axis) .* p_swarm.v_shill;
                    
                    if dir == 2
                        v_wall_virtual = -v_wall_virtual; % replusion from the opposite direction
                    end

                    %Compute relative velocity (Wall - Agent)
                    vel_ab = sqrt(sum((vel(:, agent) - v_wall_virtual).^2)); % v_wall_virtual 3*3 but vel(:, agent) 3*1

                    v_wall_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

                    if vel_ab > v_wall_max
                        vel_wall(:, agent) = vel_wall(:, agent) + ...
                            (vel_ab - v_wall_max) * (v_wall_virtual - vel(:, agent)) ./ vel_ab;
                    end
                end
            end
        end

        % Compute spheric effect - False
        if (p_swarm.is_active_spheres == true)

            for obs = 1:p_swarm.n_spheres
                % Get obstacle center and radius
                c_obs = p_swarm.spheres(1:3, obs); % if the obstacle is sphere, then we also need z axis to calculate
                r_obs = p_swarm.spheres(4, obs);

                % Compute distance agent(a)-obstacle(b)
                dist_ab = sqrt(sum((pos(:, agent) - c_obs).^2)) - r_obs;
                nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

                % Set the virtual speed of the obstacle direction out of
                % the obstacle
                v_obs_virtual = (pos(:, agent) - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill;

                % Compute relative velocity agent-obstacle
                vel_ab = sqrt(sum((vel(:, agent) - v_obs_virtual).^2));
                
                if dist_ab < min_dist_obs
                    min_dist_obs = dist_ab;
                end
      
                v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

                if vel_ab > v_obs_max
                    vel_obs(:, agent) = vel_obs(:, agent) + (vel_ab - v_obs_max) * (v_obs_virtual - vel(:, agent)) ./ vel_ab;
                end
            end
        end

        % Compute cylindric effect - True
        if (p_swarm.is_active_cyl == true)

            for obs = 1:p_swarm.n_cyl % 14 n_cyl
                % Get obstacle center and radius
                c_obs = p_swarm.cylinders(1:2, obs); % e.g., c_obs = [0;150]
                r_obs = p_swarm.cylinders(3, obs); % e.g., r_obs = 18.75

                % Compute distance agent(a)-obstacle(b)
                dist_ab = sqrt(sum((pos(1:2, agent) - c_obs).^2)) - r_obs; % relative dist with obs - r_obs -> real dist, dist_ab = 118.1149
                nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent); % r_agent = 0.5. Because the pos is the coord of agent center
                

                % Set the virtual speed of the obstacle direction out of
                % the obstacle
                
                % I don't understand why we have to simulate obs speedist_abd...
                % (pos(1:2, agent) - c_obs) is the x-y vector for dist diff
                % dist vector/ab dist -> unit dist diff, then *v_shill
                v_obs_virtual = (pos(1:2, agent) - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill; % v_shill = 13.6

                % Compute relative velocity agent-obstacle
                vel_ab = sqrt(sum((vel(1:2, agent) - v_obs_virtual).^2));
                
                % dist_ab records the minimal dist between agent&obstacle
                if dist_ab < min_dist_obs
                    min_dist_obs = dist_ab;
                end
                % Again, get_v_max?
                v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

                if vel_ab > v_obs_max
                    vel_obs(1:2, agent) = vel_obs(1:2, agent) + (vel_ab - v_obs_max) * (v_obs_virtual - vel(1:2, agent)) ./ vel_ab;
                end

            end

        end
        
        %% Sum agent-agent and obstacle contributions
        % velocity contribution of all parts, add them together
        vel_command(:, agent) = vel_rep(:, agent) + vel_fric(:, agent) + vel_obs(:, agent) + vel_wall(:, agent);

        % Add self propulsion OR migration term
        v_norm = sqrt(sum((vel(:, agent).^2), 1));

        if p_swarm.is_active_migration == true% migration % True
            % v_ref = 6; u_ref = [3;0;0] not sure why we add this, I guess
            % this velocity is for moving towards the target
            vel_command(:, agent) = vel_command(:, agent) + p_swarm.v_ref * p_swarm.u_ref; 
        elseif p_swarm.is_active_goal == true
            x_goal_rel = p_swarm.x_goal(:, agent) - pos(:, agent);
            u_goal = x_goal_rel / norm(x_goal_rel);
            vel_command(:, agent) = vel_command(:, agent) + p_swarm.v_ref * u_goal;
        else
            % self-propulsion
            if v_norm > 0
                vel_command(:, agent) = vel_command(:, agent) + p_swarm.v_ref * vel(:, agent) / v_norm;
            end
        end
    end
    vel_cmd_temp = vel_command(:,1)';
    writematrix(vel_cmd_temp(1:2),'vel_1_ori.csv','Delimiter',',', 'WriteMode','append');

    
    %% Compute collisions and bound velocities and accelerations

    % Total number of collisions per time step
    nb_agent_collisions = nb_agent_collisions / 2; % reciprocal if two agents collide with each other, we only count them as 1
    collisions = [nb_agent_collisions nb_obs_collisions min_dist_obs];

    % Add random effect on velocities - False
    if isfield(p_swarm, 'c_r')
        vel_command = vel_command + p_swarm.c_r * randn(3, nb_agents);
    end

    % Bound velocities and acceleration
    if ~isempty(p_swarm.max_v) % now max_v = 7
        vel_cmd_norm = sqrt(sum((vel_command.^2), 1)); % vel command, square sum and sqrt
        v_norm = sqrt(sum((vel.^2), 1));
        
        idx_to_bound = (vel_cmd_norm > p_swarm.max_v); 
        if sum(idx_to_bound) > 0 % if the vel_cmd value is larger than max_v, for those larger ones
            % scale the velocity, vel_command < vel_cmd_norm
            vel_command(:, idx_to_bound) = p_swarm.max_v * ...
                vel_command(:, idx_to_bound) ./ repmat(vel_cmd_norm(idx_to_bound), 3, 1);
            % repmat -> repeat copies of array
        end
    end
    if ~isempty(p_swarm.max_a) % max_a = 10, if larger, scale the accelerator
        accel_cmd = (vel_command-vel)./dt;
        accel_cmd_norm = sqrt(sum(accel_cmd.^2, 1));
        idx_to_bound = ( accel_cmd_norm > p_swarm.max_a | accel_cmd_norm < - p_swarm.max_a);
        if sum(idx_to_bound) > 0
            vel_command(:, idx_to_bound) = vel(:, idx_to_bound) + ...
                dt*p_swarm.max_a * accel_cmd(:, idx_to_bound) ./ ...
                repmat(accel_cmd_norm(idx_to_bound), 3, 1);
        end
    end
    
    vel_rep_xy_1 = vel_rep(:, 1)';
    vel_fric_xy_1 = vel_fric(:, 1)';
    vel_obs_xy_1 = vel_obs(:, 1)';
    vel_xy_1 = vel(:, 1)';
    x_goal_rel_1 = p_swarm.x_goal(:, 1) - pos(:, 1);
    u_goal_1 = x_goal_rel_1 / norm(x_goal_rel_1);
    vg_xy_1 = (p_swarm.v_ref * u_goal_1)';
    vel_command_xy_1 = vel_command(:, 1)';
    vel_theta_1 = rad2deg(atan(vel_command_xy_1(1)/vel_command_xy_1(2)));
    vel_val_1 = sqrt(vel_command_xy_1(1)^2 + vel_command_xy_1(2)^2);
    dist_ab_1 = sqrt(sum((pos(1:2, 1) - c_obs).^2)) - r_obs;
    
    px_12 = pos(1,1) - pos(1,2);
    py_12 = pos(2,1) - pos(2,2);
    
    vx_12 = vel(1,1) - vel(1,2);
    vy_12 = vel(2,1) - vel(2,2);
%     xt1_2 = pos(:,1)
    
    v_rel_1 = vel - vel(:, 1); % relative velocity
    v_rel_norm_1 = sqrt(sum((v_rel_1.^2), 1)); % relative distance in velocity

    
    % time, vf_angle, vf_value, dist_obstacle,sqrt(delta_v^2) ,(delta_vx^2 +delta_vy^2), v_previous, v_f, v_rep,v_fric, v_o, v_goal
%     accel_cmd = (vel_command-vel)./dt;
%         accel_cmd_norm = sqrt(sum(accel_cmd.^2, 1));
    % (vx_cmd, vy_cmd) Col 10, Col 11
    vel_matrix_1 = [time, vel_theta_1, vel_val_1, dist_ab_1, v_rel_norm_1(2), sqrt(sum(((vel_command_xy_1(1:2) - vel_xy_1(1:2))./dt).^2, 1)) ,vel_xy_1(1:2), vel_command_xy_1(1:2),vel_rep_xy_1(1:2), vel_fric_xy_1(1:2), vel_obs_xy_1(1:2), vg_xy_1(1:2)];
    writematrix(vel_matrix_1,'vel_1-1.csv','Delimiter',',', 'WriteMode','append');
    
    vel_cal_mat_1 = [time, pos(1:2,1)', pos(1:2,2)', pos(1:2,3)', vel(1:2,1)', vel(1:2,2)', vel(1:2,3)', vel_obs_xy_1(1:2), vg_xy_1(1:2)];
    writematrix(vel_cal_mat_1,'vel_1_cal.csv','Delimiter',',', 'WriteMode','append');
    
    %     time, v_cmd, v_rep, 
    vel_y_mat_1 = [time, vel_command(2, 1), vel_rep(2, 1), vel_y_rep(1,2,1), vel_y_rep(1,3,1), vel_fric(2, 1), vel_obs(2, 1), vg_xy_1(2)];
    writematrix(vel_y_mat_1,'vel_y_mat_1.csv','Delimiter',',', 'WriteMode','append');
    % Recalculate the distance
%     pos_real = pos;
%     pos_real(1:2,2) = pos_real(1:2,2) - [2;0];
%     for agent_real = 1:nb_agents
%         p_rel_real = pos_real - pos_real(:, agent_real); % substract the agent'th column -> get the relative coord
%         dist_real = sqrt(sum((p_rel_real.^2), 1)); % sum(A, 1)-> sum in column 1*5 -> every column is the relative absolute dist to agent
%         dist_mat_real(agent_real, :) = dist_real; %
%     end
    
    % time; neighbor_dist; vx_cmd; vx_rep; vx_fric; vx_obs; vx_g
    
    x_goal_rel_1 = p_swarm.x_goal(:, 1) - pos(:, 1);
    u_goal_1 = x_goal_rel_1 / norm(x_goal_rel_1);
    vg_xy_1 = (p_swarm.v_ref * u_goal_1)';
    vel_matrix_1 = [time, dist_mat(1, :), vel_command(1, 1), vel_rep(1, 1), vel_fric(1, 1), vel_obs(1, 1), vg_xy_1(1)];
    writematrix(vel_matrix_1,'vel_1.csv','Delimiter',',', 'WriteMode','append');
    
    % time; neighbor_dist; vx_cmd; vx_rep; vx_fric; vx_obs; vx_g
    x_goal_rel_2 = p_swarm.x_goal(:, 2) - pos(:, 2);
    u_goal_2 = x_goal_rel_2 / norm(x_goal_rel_2);
    vg_xy_2 = (p_swarm.v_ref * u_goal_2)';
    vel_matrix_2 = [time, dist_mat(2, :), vel_command(1, 2), vel_rep(1, 2), vel_fric(1, 2), vel_obs(1, 2), vg_xy_2(1)];
    writematrix(vel_matrix_2,'vel_2.csv','Delimiter',',', 'WriteMode','append');
    
%     % time; neighbor_dist; vx_cmd; vx_rep; vx_fric; vx_obs; vx_g
%     x_goal_rel_3 = p_swarm.x_goal(:, 3) - pos(:, 3);
%     u_goal_3 = x_goal_rel_3 / norm(x_goal_rel_3);
%     vg_xy_3 = (p_swarm.v_ref * u_goal_3)';
%     vel_matrix_3 = [time, dist_mat(3, :), vel_command(1, 3), vel_rep(1, 3), vel_fric(1, 3), vel_obs(1, 3), vg_xy_3(1)];
%     writematrix(vel_matrix_3,'vel_3.csv','Delimiter',',', 'WriteMode','append');
%     
%     % time; neighbor_dist; vx_cmd; vx_rep; vx_fric; vx_obs; vx_g
%     x_goal_rel_4 = p_swarm.x_goal(:, 4) - pos(:, 4);
%     u_goal_4 = x_goal_rel_4 / norm(x_goal_rel_4);
%     vg_xy_4 = (p_swarm.v_ref * u_goal_4)';
%     vel_matrix_4 = [time, dist_mat(4, :), vel_command(1, 4), vel_rep(1, 4), vel_fric(1, 4), vel_obs(1, 4), vg_xy_4(1)];
%     writematrix(vel_matrix_4,'vel_4.csv','Delimiter',',', 'WriteMode','append');
%     
    
%     vel_rep_xy = vel_rep(:, 2)';
%     vel_fric_xy = vel_fric(:, 2)';
%     vel_obs_xy = vel_obs(:, 2)';
%     vel_xy = vel(:, 2)';
%     x_goal_rel = p_swarm.x_goal(:, 2) - pos(:, 2);
%     u_goal = x_goal_rel / norm(x_goal_rel);
%     vg_xy = (p_swarm.v_ref * u_goal)';
%     vel_command_xy = vel_command(:, 2)';
%     vel_theta = rad2deg(atan(vel_command_xy(1)/vel_command_xy(2)));
%     vel_val = sqrt(vel_command_xy(1)^2 + vel_command_xy(2)^2);
%     dist_ab_2 = sqrt(sum((pos(1:2, 2) - c_obs).^2)) - r_obs;
%     
%     v_rel_2 = vel - vel(:, 2); % relative velocity
%     v_rel_norm_2 = sqrt(sum((v_rel_2.^2), 1)); % relative distance in velocity
%     
%     % time, vf_angle, vf_value, dist_obstacle, delta_vx, delta_vy, v_previous, v_f, v_rep,v_fric, v_o, v_goal
%     vel_matrix = [time, vel_theta, vel_val, dist_ab_2, v_rel_norm_2(1), sum((vel_command_xy(1:2) - vel_xy(1:2)).^2), vel_xy(1:2), vel_command_xy(1:2),vel_rep_xy(1:2), vel_fric_xy(1:2), vel_obs_xy(1:2), vg_xy(1:2)];
%     writematrix(vel_matrix,'vel_2.csv','Delimiter',',', 'WriteMode','append');

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate V fric max
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Allowed max velocity when braking without collision
function [ v_fricmax ] = get_v_max(v_fric, r, a, p)

    if r < 0
        v_fricmax = 0;
    elseif r * p > 0 && r * p < a / p
        v_fricmax = r * p;
    else
        v_fricmax = sqrt(2 * a * r - a^2 / p^2);
    end
    % bc v_fric = 0.63, r<0, v_fricmax = 0, thus v_fricmax<v_fric, thus
    % v_fricmax = 0.63
    if v_fricmax < v_fric
        v_fricmax = v_fric;
    end
end
