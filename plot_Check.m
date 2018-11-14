savem = load('grid_reward.mat');
AA = savem.savem;
fig_n = get(gcf,'Number');


% plot the reward rbf to obstacle
figure(fig_n)
[ux, ax, bx] = unique(AA(:,1));
[uy, ay, by] = unique(AA(:,2));
[X, Y] = ndgrid(ux, uy);
Z = accumarray( [bx(:),by(:)], AA(:,3), [],[], NaN );
% surf(X,Y,Z);
contourf(X,Y,Z)
%colormap default
cmap = parula(102400);
colormap(cmap)

% plot the reward of length
figure(fig_n+1)
% [ux, ax, bx] = unique(AA(:,1));
% [uy, ay, by] = unique(AA(:,2));
% [X, Y] = ndgrid(ux, uy);
Z = accumarray( [bx(:),by(:)], AA(:,4), [],[], NaN );
% surf(X,Y,Z);
contourf(X,Y,Z)


% plot the reward combination
figure(fig_n+2)
idx = 1.8484;
reward_total = AA(:,3) + idx*AA(:,4);
Z = accumarray( [bx(:),by(:)], reward_total, [],[], NaN );
% surf(X,Y,Z);
contourf(X,Y,Z)


% plot the trajectory in obstacle space
sim_result = load('grid_trajectory.mat');
a.sim_result = sim_result.sim_result;
sp = figure(fig_n+3);

mdp_params.bounds = [10;10]; % the bound of working space

% Construct MDP and features.
[mdp_data, mdp_params, reward] = obstacle_build(mdp_params);

obstacle_draw(sp, [], a, mdp_params);
