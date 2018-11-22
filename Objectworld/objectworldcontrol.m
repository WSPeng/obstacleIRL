% Evaluate controls on the objectworld environment and return gradients and
% Hessians.
function [states,A,B,invB,dxdu,d2xdudu] = objectworldcontrol(mdp_data,x,u)
% note, u is on rho and sf space. -> action space
% the x is cartisian space, -> state space 

if isempty(mdp_data.obs_params.opt_sim)
    options = check_options();
else
    options = check_options(mdp_data.obs_params.opt_sim); 
end
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

XT = [0;0]; % the target point, % dosen't matter at all

% unpack the variable from state
states = x;
x = x';
fn_handle = mdp_data.obs_params.fn_handle;
xd_obs = [0;0];

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u,1);

if length(u) <= 2 % here to differentiate between only 1 input and a series input
    
    xd = fn_handle(x-XT);
    
    rho = u(:,1);
    sf = u(:,2);
    
    obs{1}.rho = rho;
    obs{1}.sf = sf;
    
    % get new xd from modulation
    [xd, ~] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is empty

    % get new x
    x = x + xd*options.dt;

    % put back to state
    states = x';
    
else
  % if have series of inputs
    for k = 1:length(u)
        
        rho = u(k,1);
        sf = u(k,2);
    
        obs{1}.rho = rho;
        obs{1}.sf = sf;        
        
        xd = fn_handle(x-XT);
        
        % get new xd from modulation
        [xd, b_contour] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is empty
        
        % get new x
        x = x + xd*options.dt;
        
        states(k,:) = x';
    end
end

%%
% Now compute the Jacobian.
if nargout >= 2
    
    % calculate the d_lambda_d_states first
    d_lambda = zeros(Dx,2, T);
    d_E = zeros(Du, Du, Dx);
    d_invE = zeros(Du, Du, Dx);
    d_D = zeros(Du, Du, Dx);

    % d_lambda = [d_lamba_d_x, d_lamda_d_y, d_lambda_d_rho, d_lambda_d_sf]

    % then calculate A and B matrices
    A = zeros(Dx,Dx,T);
    B = zeros(Dx,Du,T);
    invB = zeros(Du,Dx,T);

    % some constant 
    dyn = [0.3;0]; % related to dynamics
    a = mdp_data.obs_params.opt_sim.obstacle{1}.a;

    states_ = [x';states]; % for the for loop
    for t = 1:T
        x1 = states_(t, 1);
        x2 = states_(t, 2);
        rho = u(t,1);
        sf = u(t,2);

        % subtracte the center of the obstacle
        x1 = x1 - mdp_data.obs_params.opt_sim.obstacle{1}.x0(1);
        x2 = x2 - mdp_data.obs_params.opt_sim.obstacle{1}.x0(2);

        % calculate the lambda1 and lambda2
        lambda1 = 1 - ((x1/sf/a(1))^2 + (x2/sf/a(2))^2)^(1/rho);
        lambda2 = -lambda1 + 2;
        D = [lambda1, 0; 0, lambda2];

        % calculate the d_lambda
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

        % some constant
        E = 2/sf^2*[x1/a(1)^2, x2/a(2)^2;...
                    x2/a(2)^2, -x1/a(1)^2]; % with the front constant related to sf..
        const_1 = (x1^2/a(1)^4 + x2^2/a(2)^4);
        invE = 1/const_1*E;

        % compute the derivative in matrix multiplcation form
        % d_E/d_x1
        d_E(:,:,1) = 2/sf^2*[1/a(1)^2, 0; 0 -1/a(1)^2];
        % d_E/d_x2
        d_E(:,:,2) = 2/sf^2*[0, 1/a(2)^2; 1/a(2)^2 0];
        % d_E/d_rho
        d_E(:,:,3) = 0;
        %d_E/d_sf
        d_E(:,:,4) = -2/sf*E;

        % d_invE/d_x1
        d_invE(:,:,1) = -2*x1/a(1)^4*(const_1)^(-2)*E + 1/const_1*d_E(:,:,1);
        % d_invE/d_x2
        d_invE(:,:,2) = -2*x2/a(2)^4*(const_1)^(-2)*E + 1/const_1*d_E(:,:,2);
        % d_invE/d_rho
        d_invE(:,:,3) = 0; 
        % d_invE/d_sf
        d_invE(:,:,4) = -2/sf*1/const_1*E;

        % d_D/d_x1
        d_D(:,:,1) = [d_lambda(1,1,t), 0; 
                    0 d_lambda(1,2,t)];
        % d_D/d_x2
        d_D(:,:,2) = [d_lambda(2,1,t), 0; 
                    0 d_lambda(2,2,t)];
        % d_D/d_rho
        d_D(:,:,3) = [d_lambda(3,1,t), 0; 
                    0 d_lambda(3,2,t)];
        % d_D/d_sf
        d_D(:,:,4) = [d_lambda(4,1,t), 0; 
                    0 d_lambda(4,2,t)];

        % together
        % x1 and x2 / x1
        dM_dx1 = d_E(:,:,1)*D*invE*dyn + E*d_D(:,:,1)*invE*dyn + ...
            E*D*d_invE(:,:,1)*dyn;
        dM_dx2 = d_E(:,:,2)*D*invE*dyn + E*d_D(:,:,2)*invE*dyn + ...
            E*D*d_invE(:,:,2)*dyn;
        dM_drho = d_E(:,:,3)*D*invE*dyn + E*d_D(:,:,3)*invE*dyn + ...
            E*D*d_invE(:,:,3)*dyn;
        dM_dsf = d_E(:,:,4)*D*invE*dyn + E*d_D(:,:,4)*invE*dyn + ...
            E*D*d_invE(:,:,4)*dyn;

        % make things clear
        dx1_dx1 = dM_dx1(1) + 1;
        dx2_dx1 = dM_dx1(2);
        dx1_dx2 = dM_dx2(1);
        dx2_dx2 = dM_dx2(2) + 1;
        dx1_drho = dM_drho(1);
        dx2_drho = dM_drho(2);
        dx1_dsf = dM_dsf(1);
        dx2_dsf = dM_dsf(2);

        % pack together
        % A is derivative with respect to previous state
        % B is derivative with respect to action
        A(:,:,t) = [dx1_dx1, dx1_dx2;
                    dx2_dx1, dx2_dx2+1];

        % B(:,:,t) = [1,0; 0,1; 1,0; 0,1];
        B(:,:,t) = [dx1_drho, dx1_dsf;
                    dx2_drho, dx2_dsf];

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

% Now compute the Hessian.
if nargout >= 6
    % First compute the Hessian of points to thrust.
    %{
    d2pdtdt = zeros(2*T,2*T,2*T);
    for i=1:2
        for t=1:T
            %mat = diag([6.0*u(1:t,i);zeros(T-t,1)]);
            mat = zeros(T,T);
            d2pdtdt((i-1)*T + (1:T),(i-1)*T + (1:T),(i-1)*T+t) = mat;
        end
    end
    %}
    
    % Now convert the Hessian to take us from states to controls.
    % TODO: if for whatever reason this is nonzero, it should be
    % implemented here.
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
end
