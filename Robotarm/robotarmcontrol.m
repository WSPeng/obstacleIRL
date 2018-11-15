% Evaluate controls on the robot arm and return gradients and Hessians.
function [states,A,B,invB,dxdu,d2xdudu] = robotarmcontrol(mdp_data,xi,u)
% note, u is on rho and sf space.
% the x is cartisian space, which means 
% states 1x4 matrix 
% the x input is inital state
COMPLEX = mdp_data.complex;

if ~COMPLEX
    if isempty(mdp_data.obs_params.opt_sim)
        options = check_options();
    else
        options = check_options(mdp_data.obs_params.opt_sim); % Checking the given options, and add ones are not defined.
    end
    x0 = mdp_data.obs_params.x0;
    d = size(x0,1); %dimension of the model
    nbSPoint=size(x0,2);
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
    
    b_contour = 0;
    
    XT = [0;0];
    
    obs{1}.rho = xi(:,3);
    obs{1}.sf = ones(2,1)*xi(:,4);
    
    fn_handle = mdp_data.obs_params.fn_handle;
    
    states = xi;
end

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u,1);

% Go step by step and evaluate equations of motion.
% TODO: implement the equations of motion here so that we can have a proper
% dynamic robot arm
%{
B = cell(1,T);
xc = x;
for t=1:T,
    % Compute forward kinematics.
    [ptx,pty,jacx,jacy,jjacx,jjacy] = robotarmfk(xc,mdp_data);
    
    % Compute diagonal mass.
    M = sum(bsxfun(@times,bsxfun(@minus,ptx,ptx').^2 + bsxfun(@minus,pty,pty').^2,mdp_data.linkmass),2);
    
    % Compute inertial term.
end;
%}

% Allocate space.



% Integrate velocities.
if COMPLEX
    % for understanding, states(:,3:4) = ..
    states(:,(Du+1):Dx) = bsxfun(@plus, cumsum(bsxfun(@rdivide,u,mdp_data.linkmass),1),...
                                xi(1,(Du+1):Dx));
else
    % get the xd from original 
    x = states(:, 1:2)';
    xd_obs = [0;0];
    if length(u) <= 2
        
        xd = fn_handle(x-XT);
        
        obs{1}.rho = obs{1}.rho + u(:,1);
        obs{1}.sf = obs{1}.sf + u(:,2);
    
        % get new xd from modulation
        [xd, b_contour] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is empty
    
        % get new x
        x = x + xd*options.dt; % should I times that ?
    
        % put back to state
        states(:, 1:2) = x;
        states(:, 3:4) = [obs{1}.rho, obs{1}.sf(1)];
    else
        for k = 1:length(u)
                        
            obs{1}.rho = obs{1}.rho + u(k,1);
            obs{1}.sf = obs{1}.sf + u(k,2);
            
            xd = fn_handle(x-XT);
            
            % get new xd from modulation
            [xd, b_contour] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is empty
            
            % get new x
            x = x + xd*options.dt;
            
            states(k, 1:2) = x;
            states(k, 3:4) = [obs{1}.rho, obs{1}.sf(1)];
        end
    end
end

if COMPLEX
    % Integrate positions.
    states(:,1:Du) = bsxfun(@plus,cumsum(states(:,(Du+1):Dx),1),xi(1,1:Du));

    % Put positions in range -pi to pi.
    states(:,1:Du) = mod(states(:,1:Du)+pi,2.0*pi)-pi;
end

% Now compute the Jacobian.
if nargout > 1
    % First, compute A and B matrices.
    A = zeros(Dx,Dx,T);
    B = zeros(Dx,Du,T);
    invB = zeros(Du,Dx,T);
    for t=1:T
        A(:,:,t) = [eye(Du) eye(Du); zeros(Du,Du) eye(Du)];
        B(:,:,t) = bsxfun(@rdivide,[eye(Du); eye(Du)],mdp_data.linkmass);
        invB(:,:,t) = pinv(B(:,:,t));
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


