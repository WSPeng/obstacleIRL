% Visualize MDP state space with given IRL test solution.
function visualize(test_result, mdp_params, reward)

% Draw reward for ground truth.
fig(1) = figure('name','draw','position',[100 550 560 420]);
sp = figure(1); % sp is the figure
% set(gca,'position',[0 0 0.5 1.0]); % it divide the plot to two pieces
feval(strcat(test_result.mdp,'draw'), sp, reward, test_result, ...
    mdp_params);

disp(test_result.R);

% Draw reward for IRL result.
% sp2 = figure(2);
% set(gca,'position',[0.5 0 0.5 1.0]);
% feval(strcat(test_result.mdp,'draw'), test_result.reward, test_result, ...
%    test_result.mdp_params);

hold off;
