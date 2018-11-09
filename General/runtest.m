% Run IRL test with specified algorithm and example.
function [test_result, mdp_params, reward] = runtest(algorithm,algorithm_params,...
    mdp,mdp_params,test_params)

% test_result - structure that contains results of the test

% algorithm - string specifying the IRL algorithm to use
% algorithm_params - parameters of the specified algorithm
% mdp - string specifying example to test on
% mdp_params - specifying parameters for example
% test_params - general parameters for the test:
%   training_samples (32) - number of example trajectories to query
%   training_sample_lengths (100) - length of each sample trajectory

% Seed random number generators.
rng(1);

% Set default test parameters.
% test_params = setdefaulttestparams(test_params);

mdp_params.bounds = [10;10];

% Construct MDP and features.
[mdp_data, mdp_params, reward] = feval(strcat(mdp,'build'), mdp_params);

% 
if mdp_params.naive_table.bool
    bounds = [mdp_params.naive_table.rho_bound,...
              mdp_params.naive_table.sf_bound];
    cell = mdp_params.naive_table.cell;
    % state_grid first col is rho, the second col is sf.
    state_grid = buildgrid(bounds, cell);
    
    mdp_data.grid_reward = [state_grid, zeros(length(state_grid),2)];
    
    for i = 1: length(state_grid)
        rho = state_grid(i, 1);
        sf = state_grid(i, 2);
        
        mdp_params.opt_sim.obstacle{1}.rho = rho;
        mdp_params.opt_sim.obstacle{1}.sf = sf;
    
        x0 = mdp_params.start_point; % the start point position
        xT = mdp_params.target_point; % the target point position
        varargin = mdp_params.opt_sim;
        
        [x, ~, ~, ~, ~] = Simulation(x0, xT, mdp_data.handle, varargin);
        x = x';
        R_dist = 0;
        for j = 1: length(x)
            R_dist = R_dist + cartrbfevalreward(reward, x(j,:));
        end
        
        R_length = trajectory_length_reward(reward, x);
        
        mdp_data.grid_reward(i,end-1:end) = [R_dist, R_length];
        
    end
end
    
% manually defined parameters
rho = [5,    6, 3,   4,   4];
sf =  [1.25, 1, 1.6, 1.4, 1.5];

sim_result = cell(length(rho));

% package the test result
test_result = struct('mdp', mdp);

% use predefined rho and sf combination.
for i = 1:length(rho)
	mdp_params.opt_sim.obstacle{1}.rho = rho(i);
	mdp_params.opt_sim.obstacle{1}.sf = sf(i);
    
	x0 = mdp_params.start_point; % the start point position
	xT = mdp_params.target_point; % the target point position
	varargin = mdp_params.opt_sim;

	[x, xd, t, xT, x_obs] = Simulation(x0, xT, mdp_data.handle, varargin);
    
    % package the sim result, which will be package inside the test result
    % struct
    sim_result{i}.x = x';
    sim_result{i}.xd = xd';
    sim_result{i}.t = t';
    sim_result{i}.xT = xT;
    sim_result{i}.x_obs = x_obs;
    
    % calcualte the reward
    R_length = trajectory_length_reward(reward, sim_result{i}.x);
    R_dist = 0;
    for j = 1: length(x)
        R_dist = R_dist + cartrbfevalreward(reward, sim_result{i}.x(j,:));
    end
    
    test_result.R{i} = [R_length; R_dist];
    
    max_demo_result(test_result);
    
end

test_result.sim_result = sim_result;
