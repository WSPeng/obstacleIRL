% Run IRL test with specified algorithm and example.
function [test_result, mdp_params, reward] = runtest_naive(algorithm,algorithm_params,...
    mdp,mdp_params,test_params)

read_mat = 0;

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

mdp_params.bounds = [10;10]; % the bound of working space

% Construct MDP and features.
[mdp_data, mdp_params, reward] = feval(strcat(mdp,'build'), mdp_params);

% 
if mdp_params.naive_table.bool
    bounds = [mdp_params.naive_table.rho_bound,...
              mdp_params.naive_table.sf_bound];
    cells = mdp_params.naive_table.cell;
    % state_grid first col is rho, the second col is sf.
    state_grid = buildgrid(bounds, cells);
    
    mdp_data.grid_reward = [state_grid, zeros(length(state_grid),2)];
    
    sim_result = cell(length(mdp_data.grid_reward),1);
    
    if read_mat
	    for i = 1: length(state_grid)
            if mod(i,100) == 0
	            disp('state_grid loop')
	            disp(i)
	        end
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
	        
	        mdp_data.grid_reward(i,end-1:end) = [R_dist, R_length]; % 3 and 4 column
            
            % store the trajectory
            sim_result{i}.x = x;
        end
        
        savem = mdp_data.grid_reward;
	    save('grid_reward.mat','savem');
        save('grid_trajectory.mat','sim_result')
        
    else
		savem = load('grid_reward.mat');
        mdp_data.grid_reward = [];
        mdp_data.grid_reward = savem.savem;
	
    end
end
    
% manually defined parameters
rho = [5,    6, 3,   4,   4];
sf =  [1.25, 1, 1.6, 1.4, 1.5];
% 2.5223 correspondting to 3.8 and 1.32, which is pretty good?


% another set of paramters?
% rho = [8,    7.5, 7,   7.8,   7.6];%, 8];
% sf =  [1.55, 1.6, 1.5, 1.48,  1.59];%, 1.6];


sim_result = cell(length(rho),1);

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
    
    test_result.R{i} = [R_dist; R_length]; % R_length anmd R_dist are all one number.
    
end

% the IRL

theta = max_demo_reward(mdp_data, test_result);

test_result.sim_result = sim_result;
