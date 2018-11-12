% Draw single objectworld with specified reward function.
function obstacle_draw(sp, reward, example_samples, mdp_params)

% Initialize window.
% VMARGIN = 1.0;
% HMARGIN = 1.0;
% axis([-HMARGIN  mdp_params.bounds(1)+HMARGIN  -VMARGIN  mdp_params.bounds(2)+VMARGIN]);

% Expand axes.
% set(gca,'position',[0 0 1 1]);
% set(gca,'color','none','xgrid','off','ygrid','off','visible','off');
% set(gca,'xtick',[]);
% set(gca,'ytick',[]);

% daspect([1 1 1]);
hold on;

% Draw the reward function.
% Create regular samples.
% STEP_SIZE = 0.2;
% x = -HMARGIN:STEP_SIZE:(mdp_params.bounds(1)+HMARGIN);
% y = -VMARGIN:STEP_SIZE:(mdp_params.bounds(2)+VMARGIN);
% [X,Y] = meshgrid(x,y);
% pts = [X(:) Y(:)];
% R = feval(strcat(reward.type,'evalreward'),reward,mdp_params,[],zeros(size(pts,1),mdp_params.udims),pts * mdp_params.sensor_basis,[],[],[],[]);

%%
% Draw the ecllipsoid
ploting.fig = sp;

sp = plot_results('i', ploting, mdp_params.start_point, mdp_params.target_point, ...
    mdp_params.opt_sim.obstacle);

%%
% Draw the paths.
hold on

% col = ones(1,3)*;

if ~isempty(example_samples.sim_result)
    for i=1:length(example_samples.sim_result)
        % all the points in trajectory is included in the sim_result
        pts = example_samples.sim_result{i}.x;
        % x is the position, xd is the velocity
        
        % Plot the points.
        width_factor = 1;
        sp.fig;
        
        % plot(pts(:,1),pts(:,end),'-','color',col,'marker','.','markersize',14*width_factor,'linewidth',1.5);
        plot(pts(:,1),pts(:,end),'-','marker','.','markersize',14*width_factor,'linewidth',0.5);
        
    end
    %len1 = strcat('rho ',num2str(mdp_params.opt_sim.obstacle{1}.rho), 'sf', num2str(mdp_params.opt_sim.obstacle{1}.sf));
    %len2 = strcat('rho ',num2str(mdp_params.opt_sim.obstacle{1}.rho), 'sf', num2str(mdp_params.opt_sim.obstacle{2}.sf));
    %len3 = strcat('rho ',num2str(mdp_params.opt_sim.obstacle{3}.rho), 'sf', num2str(mdp_params.opt_sim.obstacle{3}.sf));
    %len4 = strcat('rho ',num2str(mdp_params.opt_sim.obstacle{4}.rho), 'sf', num2str(mdp_params.opt_sim.obstacle{4}.sf));
    %len5 = strcat('rho ',num2str(mdp_params.opt_sim.obstacle{5}.rho), 'sf', num2str(mdp_params.opt_sim.obstacle{5}.sf));

    % len1 = strcat('rho ',num2str(5), 'sf', num2str(1.25));
    % len2 = strcat('rho ',num2str(6), 'sf', num2str(1));
    % len3 = strcat('rho ',num2str(3), 'sf', num2str(1.6));
    % len4 = strcat('rho ',num2str(4), 'sf', num2str(1.4));
    % len5 = strcat('rho ',num2str(4), 'sf', num2str(1.5));

    len1 = strcat('rho ',num2str(8), 'sf', num2str(1.55));
    len2 = strcat('rho ',num2str(7.5), 'sf', num2str(1.6));
    len3 = strcat('rho ',num2str(7), 'sf', num2str(1.5));
    len4 = strcat('rho ',num2str(7.8), 'sf', num2str(1.48));
    len5 = strcat('rho ',num2str(7.6), 'sf', num2str(1.59));
    
    legend('','','','','','',len1, len2, len3, len4, len5, 'theoptimal')
end

% Finished.
hold off;
