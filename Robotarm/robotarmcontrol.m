% Evaluate controls on the robot arm and return gradients and Hessians.
function [states,A,B,invB,dxdu,d2xdudu] = robotarmcontrol(mdp_data, xi, u)
% note, u is on rho and sf space.
% the x is cartisian space, which means 
% states 1x4 matrix 
% the xi input is inital state

x1_T = 10;
x2_T = 2.2;

if size(xi,1) > 1
    a = 1;
end

if isempty(mdp_data.obs_params.opt_sim)
    options = check_options();
else
    options = check_options(mdp_data.obs_params.opt_sim); 
end

x0 = mdp_data.obs_params.x0; % the start point
d = size(x0,1); %dimension of the model
obs = options.obstacle;

for n=1:length(obs)
    x_obs{n} = obs{n}.x0;
    if ~isfield(obs{n},'extra')
        obs{n}.extra.ind = 2;
        obs{n}.extra.C_Amp = 0.01;
        obs{n}.extra.R_Amp = 0.0;
    end
end

b_contour = 0;

XT = [0;0]; % the target point 

% unpack the variable from state
states = xi;
rho = states(:,3);
sf = ones(2,1)*states(:,4); 

fn_handle = mdp_data.obs_params.fn_handle;

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u,1); % the length of control input

% Integrate velocities.
x = states(:, 1:2)';
xd_obs = [0;0];

if length(u) <= 2 % here to differentiate between only 1 input and a series input
    
    xd = fn_handle(x-XT);
    
    rho = rho + u(:,1);
    sf = sf + u(:,2);

    % if out of bound
    if rho<mdp_data.sbounds(1,3)
        rho = mdp_data.sbounds(1,3);
    elseif rho>mdp_data.sbounds(2,3)
        rho = mdp_data.sbounds(2,3);
    end
    if sf<mdp_data.sbounds(1,4)
        sf = mdp_data.sbounds(1,4);
    elseif sf>mdp_data.sbounds(2,4)
        sf = mdp_data.sbounds(2,4);
    end
    
    obs{1}.rho = rho;
    obs{1}.sf = sf;
    
    % get new xd from modulation
    [xd, b_contour] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is empty

    % get new x
    x = x + xd*options.dt; % should I times that ?

    % put back to state
    states(:, 1:2) = x;
    states(:, 3:4) = [rho, sf(1)];
    
else
    % if have series of inputs
    for k = 1:length(u)
        
        rho = rho + u(k,1);
        sf = sf + u(k,2);
        
        % if out of bound
        if rho<mdp_data.sbounds(1,3)
            rho = mdp_data.sbounds(1,3);
        elseif rho>mdp_data.sbounds(2,3)
            rho = mdp_data.sbounds(2,3);
        end
        if sf<mdp_data.sbounds(1,4)
            sf = mdp_data.sbounds(1,4);
        elseif sf>mdp_data.sbounds(2,4)
            sf = mdp_data.sbounds(2,4);
        end
    
        
        obs{1}.rho = rho;
        obs{1}.sf = sf;        
        
        xd = fn_handle(x-XT);
        
        % get new xd from modulation
        [xd, b_contour] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is empty
        
        % get new x
        x = x + xd*options.dt;
        
        states(k, 1:2) = x;
        states(k, 3:4) = [rho, sf(1)];
    end
end

% Now compute the Jacobian.
if nargout > 1
    if mdp_data.complex
        % First, compute A and B matrices.
        A = zeros(Dx,Dx,T);
        B = zeros(Dx,Du,T);
        invB = zeros(Du,Dx,T);
        for t=1:T
            A(:,:,t) = [eye(Du) eye(Du); zeros(Du,Du) eye(Du)];
            B(:,:,t) = bsxfun(@rdivide,[eye(Du); eye(Du)],mdp_data.linkmass);
            invB(:,:,t) = pinv(B(:,:,t));
        end
    else
            
        % calculate the d_lambda_d_states first
        
        d_lambda = zeros(Dx,2, T);
        % d_lambda = [d_lamba_d_x, d_lamda_d_y, d_lambda_d_rho, d_lambda_d_sf]
        
        % then calculate A and B matrices
        A = zeros(Dx,Dx,T);
        B = zeros(Dx,Du,T);
        invB = zeros(Du,Dx,T);
        
        states_ = [xi;states]; % for the for loop
        for t = 1:T
            x1 = states_(t, 1);
            x2 = states_(t, 2);
            rho = states_(t, 3);
            sf = states_(t, 4);
            a = mdp_data.obs_params.opt_sim.obstacle{1}.a;
            
            % subtracte the center of the obstacle
            x1 = x1 - mdp_data.obs_params.opt_sim.obstacle{1}.x0(1);
            x2 = x2 - mdp_data.obs_params.opt_sim.obstacle{1}.x0(2);
            
            % d_lambda1/d_x1
            d_lambda(1, 1, t) = 2*x1/(rho*sf^2*a(1)^2)*...
                ((x1/sf/a(1))^2 + (x2/sf/a(2))^2)^(-1/rho-1) ;
            % d_lambda1/d_x2
            d_lambda(2, 1, t) = 2*x2/(rho*sf^2*a(2)^2)*...
                ((x1/sf/a(1))^2 + (x2/sf/a(2))^2)^(-1/rho-1) ;
            % d_lambda1/d_rho
            d_lambda(3, 1, t) = -((x1/sf/a(1))^2 + (x2/sf/a(2))^2)^(-1/rho)...
                *log((x1/sf/a(1))^2 + (x2/sf/a(2))^2)*rho^(-2);
            % d_lambda1/d_sf
            d_lambda(4, 1, t) = -2*a(1)/rho*(x1^2+x2^2)^(-1/rho)...
                *(sf*a(1))^(2/rho-1);
            
            % d_lambda2/d_x1
            d_lambda(1, 2, t) = -d_lambda(1, 1, t);
            % d_lambda2/d_x2
            d_lambda(2, 2, t) = -d_lambda(2, 1, t);
            % d_lambda2/d_rho
            d_lambda(3, 2, t) = -d_lambda(3, 1, t);
            % d_lambda2/d_sf
            d_lambda(4, 2, t) = -d_lambda(4, 1, t);

            % calculate the lambda1 and lambda2
            lambda1 = 1 - ((x1/sf/a(1))^2 + (x2/sf/a(2))^2)^(1/rho);
            lambda2 = -lambda1 + 2;
            
            % A and B elements
            dx1_dx1 = x1_T/2*sf^2*d_lambda(1, 2, t) - sf^2/2*lambda2 -...
                sf^2*x1/2*d_lambda(1, 2, t) + ...
                sf^2*a(2)^2/2*(x2_T/x2-1)*x1/a(1)^2*(d_lambda(1,1,t)-d_lambda(1,2,t))...
                + sf^2*a(2)^2/2*(x2_T/x2-1)*1/a(1)^2*(lambda1-lambda2);
            
            dx1_dx2 = x1_T/2*sf^2*d_lambda(2, 2, t) - ...
                sf^2*x1/2*d_lambda(2, 2, t) - ...
                sf^2*a(2)^2/2*x1/a(1)^2*(x2_T*x2^(-2))*(lambda1-lambda2) + ...
                sf^2*a(2)^2/2*x1/a(1)^2*(x2_T/x2-1)*(d_lambda(2,1,t)-d_lambda(2,2,t));
            
            dx1_drho = x1_T/2*sf^2*d_lambda(3,1,t) - sf^2*x1/2*d_lambda(3,1,t)...
                + sf^2*a(2)^2/2*(x2_T/x2-1)*x1/a(1)^2*(d_lambda(3,1,t) - d_lambda(3,2,t));
            
            dx1_dsf = x1_T*sf*lambda2 + x1_T/2*sf^2*d_lambda(4, 2, t) - ...
                sf*x1*lambda2 - sf^2*x1/2*d_lambda(4, 2, t) + ...
                sf*a(2)^2*(x2_T/x2-1)*x1/a(1)^2*(lambda1-lambda2) + ...
                sf^2*a(2)^2/2*(x2_T/x2-1)*x1/a(1)^2*(d_lambda(4,1,t)-d_lambda(4,2,t));
            
            
            dx2_dx1 = 1/2*(x2_T-x2)*sf^2*d_lambda(1,1,t);
            
            dx2_dx2 = -1/2*sf^2*lambda1 + 1/2*(x2_T-x2)*sf^2*d_lambda(1,2,t);
            
            dx2_drho = 1/2*(x2_T-x2)*sf^2*d_lambda(3,1,t);
            
            dx2_dsf = (x2_T-x2)*sf*lambda1 + 1/2*(x2_T-x2)*sf^2*d_lambda(4,1,t);
            
            A(:,:,t) = [dx1_dx1+1, dx1_dx2, dx1_drho, dx1_dsf;
                        dx2_dx1, dx2_dx2+1, dx2_drho, dx2_dsf;
                        0,0,1,0;
                        0,0,0,1];
            
            % B(:,:,t) = [1,0; 0,1; 1,0; 0,1];
            B(:,:,t) = [dx1_drho, dx1_dsf;
                        dx2_drho, dx2_dsf;
                        1,0;
                        0,1];
            
            invB(:,:,t) = pinv(B(:,:,t));
        end
    end
    
    
    
    
    if nargout >= 5
        % Now build the Jacobian out of these matrices.
        dxdu = zeros(Du*T,Dx*T);
        uidx = (0:T:(T*(Du-1)));
        xidx = (0:T:(T*(Dx-1)));
        for c=1:T
            % First, compute the top part of this block row.
            for r=1:(c-1)
                dxdu(r + uidx, c + xidx) = dxdu(r + uidx, (c-1) + xidx)*A(:,:,c)';
            end
            % Now write the diagonal block.
            dxdu(c + uidx, c + xidx) = B(:,:,c)';
        end
    end
end






% The Hessian is zero.
% Note that this is only strictly true in the kinematic case. In the
% inertial case, this is not true, but we can use it as an approximation in
% order to keep using the linear-time algorithm.
if nargout >= 6
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
end


