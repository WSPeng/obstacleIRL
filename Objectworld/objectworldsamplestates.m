% Sample some random states in the Objectworld.
function s = objectworldsamplestates(n,mdp_data)

% s = bsxfun(@times,rand(n,2),mdp_data.bounds) * mdp_data.sensor_basis;

% s = [0, 2.2];

s = [0, 2.2 + 0.4*rand(1)];