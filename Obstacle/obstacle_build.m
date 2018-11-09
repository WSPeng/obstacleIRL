function [mdp_data, mdp_params, reward] = obstacle_build(mdp_params)

% Fill in default parameters.
mdp_params = obstacle_defaultparameter(mdp_params);

% function handle is the dynamic system
% no action bound
fn_handle = @(x) [-x(1,:);...
					zeros(1,size(x,2))]; %defining the function handle

% build mdp data struct
mdp_data = struct('handle', fn_handle);

% build features
center = mdp_params.obstacle_pos;

reward = struct('type','sum',...
    'dist_type', 'rbf', ...
    'pos', center,...
    'r', 1,...
    'width',1);
