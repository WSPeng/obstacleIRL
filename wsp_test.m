%% preparing the obstacle avoidance module
%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end

%% obstacle definition
clear obs;
obs{1}.a = [1.2 1.2;0.4 1];
obs{1}.p = [2 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-8;0];  % position of the obstacle
obs{1}.sf = [1.2;1.2]; % the safety factor
obs{1}.th_r = 0*pi/180;
obs{1}.rho = 1;

%% drawing stream line
disp('   - reactivity    (.rho)')
disp('   - Tail Effect   (.tailEffect)')
disp('To have a better visualisation, we consider a simple dynamical systems:')
disp('  xd = [-x(1,:);0];')

% function handle is the dynamic system
% fn_handle = @(x) [10-x(1,:);zeros(1,size(x,2))]; %defining the function handle
% fn_handle = @(x) [10-x(1,:);2.2-x(2,:)];

% define moving in constant speed.
fn_handle = @move_costant_v;

%x0 = [0*ones(1,20);linspace(0,10,20)]; %set of initial points, the first row are x coordinate
x0 = [0,0*ones(1,2);2.2,linspace(0,10,2)];

% A set of parameters that should be defined for the simulation
opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.obstacle = []; %no obstacle is defined
fig = [];
if (false)
    fig(1) = figure('name','Third demo: Streamlines of the original DS','position',[100 550 560 420]);
    opt_sim.figure = fig(1);
    [x,~,~,~,~] = Simulation(x0,[],fn_handle,opt_sim); % target point empty.
    pause
end


disp(' ')
disp(' ')
disp('one obstacle.')
obs{1}.x0 = [5;2];
opt_sim.obstacle = obs;
fig(1) = figure('name','Third demo: Streamlines of the modulated DS','position',[660 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause

% change the safety factor
disp('change the safety factor')
obs{1}.sf = [2;2];
opt_sim.obstacle = obs;
fig(2) = figure('name','change safety factor','position',[100 550 560 420]);
opt_sim.figure = fig(2);
Simulation(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause

% Changing the reactivity
disp(' ')
disp(' ')
disp('Now let us change the obstacle property so that the motion reacts')
disp('earlier to the presence of the robot. For this, we increase the')
disp('reactivity factor from 1 to 3. We add the following line:')
disp('   obs{1}.rho = 3')
obs{1}.sf = [1.2;1.2];
obs{1}.rho = 3;
obs{1}.tailEffect = true;
disp('press any key to draw the streamlines ...')
pause
opt_sim.obstacle = obs;
fig(3) = figure('name','Third demo: Effect of the reactivity parameter','position',[100 50 560 420]);
opt_sim.figure = fig(3);
Simulation(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause

% Removing the taileffect
disp(' ')
disp(' ')
disp('By setting the tailEffect property to the false, the motion is only')
disp('modified when it approaches the obstacle.')
obs{1}.tailEffect = false;
opt_sim.obstacle = obs;
fig(4) = figure('name','Third demo: Removing the tail effect','position',[660 50 560 420]);
opt_sim.figure = fig(4);
Simulation(x0,[],fn_handle,opt_sim);
disp('press any key to exit.')
pause
close(fig);


function y = move_costant_v(x)
    % y = [10-x(1,:);2.2-x(2,:)];
    % y = [10/size(x,2)*ones(1,size(x,2));...
    %     zeros(1,size(x,2))];
    y = [10-x(1,:)+0.01;...
        2.2-x(2,:)];
end