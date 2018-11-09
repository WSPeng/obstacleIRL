function params = obstacle_defaultparameter(params)

% Create default parameters.

% Obstacles
obs{1}.a = [1.2 1.2;0.4 1];
obs{1}.p = [2 1;1 1];
obs{1}.partition = [-pi 0;0 pi];

if isfield(params, 'bounds')
    obs{1}.x0 = [0.5*params.bounds(1);0.2*params.bounds(2)];  % position of the obstacle
else
    obs{1}.x0 = [0.5*10;0.2*10];
end
if isfield(params, 'rho')
    obs{1}.rho = params.rho;
else
    obs{1}.rho = 1;
end
if isfield(params, 'sf')
    obs{1}.sf = [1;1]*params.sf; % the safety factor
else
    obs{1}.sf = [1;1]*1;
end
obs{1}.tailEffect = true;
obs{1}.th_r = 0*pi/180;

% simulation options
opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
% disable the drawing function of Simulation
opt_sim.plot = false; %enabling the animation
opt_sim.obstacle = obs; %no obstacle is defined


default_params = struct(...
    'seed',0,...
    'obstacle_pos',obs{1}.x0,...
    'obstacle_pos_repel',[0.5;0.9],...
    'start_point',[0;2.2],...
    'target_point',[10;2.2],...
    'bounds',[10;10],...
    'opt_sim',opt_sim);

% Set parameters.
params = filldefaultparams(params,default_params);
