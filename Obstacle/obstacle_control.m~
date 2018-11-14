function [x, xd, t, xT, x_obs] = obstacle_control(x0, xT, fn_handle, varargin)
% Call obs_modulation_ellipsoid

if isempty(varargin)
    options = check_options();
else
    options = check_options(varargin{1}); % Checking the given options, and add ones are not defined.
end

d = size(x0,1); %dimension of the model, which is 2-D
if isempty(xT)
    xT = zeros(d,1);
end


%% setting initial values
nbSPoint = size(x0,2); %number of starting points. This enables to simulatneously run several trajectories

if isfield(options,'obstacle') && ~isempty(options.obstacle) %there is obstacle
    obs_bool = true;
    obs = options.obstacle;
    for n=1:length(obs)
        x_obs{n} = obs{n}.x0;
        if ~isfield(obs{n},'extra')
            obs{n}.extra.ind = 2;
            obs{n}.extra.C_Amp = 0.01;
            obs{n}.extra.R_Amp = 0.0;
        end
    end
    b_contour = zeros(1,nbSPoint);
end

%initialization
for i=1:nbSPoint
    x(:,1,i) = x0(:,i);
end

xd = zeros(size(x));

if size(xT) == size(x0)
    XT = xT;
end
            
t=0; %starting time
k=0;
%% Simulation
i=1;
while true
    
    % xd is the speed
    % from the control law .. 
    xd(:,i,:)=reshape(fn_handle(squeeze(x(:,i,:))-XT),[d 1 nbSPoint]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This part if for the obstacle avoidance module
    
    if obs_bool
        xd_obs = zeros(d,nbSPoint);
        
        % after adding perturbation
        for j=1:nbSPoint
            % calling the function obs_modulation_ellipsoid .. 
            [xd(:,i,j), b_contour(j)] = obs_modulation_ellipsoid(x(:,i,j),xd(:,i,j),obs,b_contour(j),xd_obs);
        end

    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Integration
    
    x(:,i+1,:) = x(:,i,:) + xd(:,i,:)*options.dt;

    t(i+1)=t(i)+options.dt;
    
    xd_3last = xd(:,max([1 i-3]):i,:);
    xd_3last(isnan(xd_3last)) = 0;
    
    %Checking the convergence
    if all(all(all(abs(xd_3last)<options.tol))) || i>options.i_max-2
        i=i+1;
        
        x(:,end,:) = [];
        t(end) = [];
        
        if i>options.i_max-2
            fprintf('Simulation stopped since it reaches the maximum number of allowed iterations i_max = %1.0f\n',i)
            fprintf('Exiting without convergence!!! Increase the parameter ''options.i_max'' to handle this error.\n')
        end
        break
    end
    i=i+1;
    
    if ~b_contour
        k = k + 1;
        disp(k)
    end
end

disp('ok')