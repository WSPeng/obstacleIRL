% Set default general parameters for the test.
function test_params = setdefaulttestparams(test_params)

% Create default parameters.
default_params = struct(...
    'verbosity',1,...
    'training_samples',16);
    
% Set parameters.
test_params = filldefaultparams(test_params,default_params);
