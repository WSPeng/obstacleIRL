% temporary function, only for testing the dynamic function

obs{1}.a = [1.2 1.2;0.4 1];
obs{1}.p = [2 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [5;2];  % position of the obstacle
obs{1}.sf = [1.2;1.2]; % the safety factor
obs{1}.th_r = 0*pi/180;
obs{1}.rho = 1;

fn_handle = @move_costant_v;

x0 = [0; 2.2];

% A set of parameters that should be defined for the simulation
opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.obstacle = obs;

obstacle_control(x0, [], fn_handle, opt_sim)


function y = move_costant_v(x)
    % y = [10-x(1,:);2.2-x(2,:)];
    % y = [10/size(x,2)*ones(1,size(x,2));...
    %     zeros(1,size(x,2))];
    y = [10-x(1,:)+0.01;...
         2.2-x(2,:)];
end