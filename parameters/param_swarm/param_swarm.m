% In your main, run this script after the swarm initialization

% Variables to be set
p_swarm.is_active_migration = false;
p_swarm.is_active_goal = true;
p_swarm.is_active_arena = false;
p_swarm.is_active_spheres = false;
p_swarm.is_active_cyl = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isfield(p_swarm, 'nb_agents')
    p_swarm.nb_agents = 3;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max radius of influence - Metric distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r = 150;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max number of neighbors - Topological distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isfield(p_swarm, 'max_neig')
    p_swarm.max_neig = 10;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Radius of collision -
% it is the radius of the sphere that approximates
% the drone. A collision is counted when two 
% spheres intersect.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r_coll = 0.5;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arena parameters - Cubic arena
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x,y,z positions of the walls
p_swarm.x_arena = [-100 100; % x wall
            -100 100; % y_wall
            -100 100]; % z_wall
p_swarm.center_arena = sum(p_swarm.x_arena, 2) / 2;

% Parameter that defines the influence radius of the arena repulsion force
p_swarm.d_arena = 1.5;

% Constant of proportionality of the arena repulsion force
p_swarm.c_arena = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Spheric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.spheres = [
    -70 70 0; % x_obstacle
    50 50 200; % y_obstacle
    5 5 5; % z_obstacle
    50 50 50]; % r_obstacle

p_swarm.n_spheres = length(p_swarm.spheres(1, :));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cylindric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (exist('map','var') && ACTIVE_ENVIRONMENT)

    nb_obstacles = length(map.buildings_east);
    cylinder_radius = map.building_width / 2;
    
    p_swarm.cylinders = [
        map.buildings_north'; % x_obstacle
        map.buildings_east'; % y_obstacle
        repmat(cylinder_radius, 1, nb_obstacles)]; % r_obstacle

    p_swarm.n_cyl = length(p_swarm.cylinders(1, :));
    
else
    p_swarm.cylinders = 0;
    p_swarm.n_cyl = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference values
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inter-agent distance
if ~isfield(p_swarm, 'd_ref')
    p_swarm.d_ref = 25;
end

% Velocity direction
p_swarm.u_ref = [1 0 0]';

% Speed
if ~isfield(p_swarm, 'v_ref')
    p_swarm.v_ref = 6;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Velocity and acceleration bounds for the agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.max_a = 10;
% p_swarm.max_a = []; % leave empty if you use a real drone model
p_swarm.max_v = 7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial position and velocity for the swarm
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial positions are contained in a cubic area
p_swarm.P0 = [-10,150,-50]'; % [m] position of a vertex of the cube
p_swarm.P = 20; % [m] cube edge size

% Velocities are inizialized in a cubic subspace
p_swarm.V0 = [0,0,0]'; % [m/s]
p_swarm.V = 0; % [m/s]

% Seed to avoid random effects
p_swarm.seed = 5;
rng(p_swarm.seed);

% Set the goal
% Front setting ----
goal_y = 164.5;
goal_x = 180;
p_swarm.x_goal = [goal_x goal_x goal_x;goal_y goal_y goal_y;-38 -38 -38];
% p_swarm.x_goal = [80 80; 150 150 ; -38 -38];


% ---------Elaine --------
init_pos = rand(3,p_swarm.nb_agents);
% init_x = rand(1,p_swarm.nb_agents)*1.75 - 0.5;
% init_y = rand(1,p_swarm.nb_agents)*-3.5 -2.5;
% init_pos = [init_x; init_y;0.6*ones(1,p_swarm.nb_agents)];
init_pos = [init_pos(1:2,:);0.6*ones(1,p_swarm.nb_agents)];
% p_swarm.Pos0 = p_swarm.P0 + p_swarm.P * rand(3,p_swarm.nb_agents);
p_swarm.Pos0 = p_swarm.P0 + p_swarm.P * init_pos;
% p_swarm.Pos0 = [-5.5601 8.3722 5.3182 -6.2456 -1.1738;
%   167.4146 159.7682 160.3684 151.6148 153.1662;
%   -38.0000 -38.0000 -38.0000 -38.0000 -38.0000];

% % ------ Two drones setting start-------
% px = -18.75-5; % 5
% % Front setting ----
% dx = 105; % 105 when I increase it to 120, the dist = 3.138
% % the drone behind
% % dx = -5;
% p_swarm.Pos0 = [px (px+dx); 150 150; -38 -38];
% 
% % disp('init_pos');                
% % disp(init_pos);
% % disp(p_swarm.Pos0);
% 
% % ------ Two drones setting end-------

% ------ Four drones setting start-------
bx = 5;
px = -18.75-bx; % 5
% dx = 10+18.5;
dx = 80; % should > 38.75+5
dy = 25;

% p_swarm.Pos0 = [px (px+dx) (px+dx_2) (px+dx_2) (px+dx_3); 150 (150+dy_2) (150-dy_2) (150+dy_2) (150+dy_2); -38 -38 -38 -38 -38];
p_swarm.Pos0 = [px (px+dx) (px+dx); 150 (150-25-dy) (150+25); -38 -38 -38];
% ------ Four drones setting end-------

p_swarm.Vel0 = p_swarm.V0 + p_swarm.V * rand(3,p_swarm.nb_agents);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Call algorithm-specific swarm parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if exist('SWARM_ALGORITHM','var')
    str = "param_";
    run(strcat(str, SWARM_ALGORITHM));
end
