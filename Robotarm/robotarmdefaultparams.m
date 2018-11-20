% Fill in default parameters for the robotarm example.
function mdp_params = robotarmdefaultparams(mdp_params)

% Create default parameters.
default_params = struct(...
    'seed',0,...
    'size',10,...
    'links',3,...
    'linklen',[2.0 2.0 2.0],...
    'linkmass',[100.0 100.0 100.0],...
    'objects',40,...
    'c1_1prob',0.1,...
    'c1_2prob',0.2,...
    'c1',6,...
    'c2',6,...
    'rbf_features',1,... % 1 means yes ?
    'fixed_pattern',1,...
    'feature_type','grid',... % other feature type are like grid simple and cart
    'step_cost',10.0,...
    'feature_radius',0.5); % it was 1.0

% Set parameters.
mdp_params = filldefaultparams(mdp_params,default_params);
