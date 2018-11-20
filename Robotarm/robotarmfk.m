% Perform forward kinematics to determine Cartesian positions of end
% effectors.
function [ptx,pty,jac_x,jac_y,jjacx,jjacy] = robotarmfk(states,mdp_data)
if mdp_data.complex
    COMPLEX = 1;
else
    COMPLEX = 0;
end
% Notes:
% states: 128x4
% mdp_data
% 

% Run forward kinematics.
T = size(states,1); % T is the block size, for computation efficiency
if COMPLEX
    N = mdp_data.links;
else
    % N = 1;
    N = mdp_data.links;
end

ptx = zeros(T,N);
pty = zeros(T,N);

if COMPLEX
    ptx0 = 0.5*ones(T,1)*mdp_data.bounds(1); % the base coordinate of robot arm
    pty0 = 0.5*ones(T,1)*mdp_data.bounds(2); % the base coordinate
    ptxp = ptx0;
    ptyp = pty0;
    a = zeros(T,1);
else
    % compute the points
    % the first column of ptx and pty are the middle joint.
    ptx = [zeros(T,1), states(:,1)];
    pty = [zeros(T,1), states(:,2)];
end

if COMPLEX
    for i=1:N
        % Increment angles.
        a = a + states(:,i);
        
        % Compute positions.
        ptx(:,i) = ptxp + mdp_data.linklen(i)*sin(a);
        pty(:,i) = ptyp + mdp_data.linklen(i)*cos(a);
        
        % Increment running positions.
        ptxp = ptx(:,i);
        ptyp = pty(:,i);
    end
end

% Compute Jacobians.

if ~COMPLEX
    if nargout > 2
        jac_x = zeros(T,N,N);
        jac_y = zeros(T,N,N);
        for i = 1:T
            jac_x(i,:,:) = eye(N);
            jac_y(i,:,:) = eye(N);
        end
    end
else
    if nargout > 2
        % These are preceding (pivot) points.
        fpx = [ptx0 ptx(:,1:(N-1))];
        fpy = [pty0 pty(:,1:(N-1))];
        
        % Compute full Jacobians (without zero entries).
        jac_x = bsxfun(@minus,pty,permute(fpy,[1 3 2]));
        jac_y = bsxfun(@plus,-ptx,permute(fpx,[1 3 2]));
        
        % Now zero out those entries where i_3 > i_2.
        mask = permute(tril(ones(N,N)),[3 1 2]);
        jac_x = bsxfun(@times,jac_x,mask);
        jac_y = bsxfun(@times,jac_y,mask);
    end
end

% Compute second derivatives.

if ~COMPLEX
    if nargout > 4
        jjacx = zeros(T,N,N,N);
        jjacy = zeros(T,N,N,N);
    end
else
    if nargout > 4
        % Construct index matrix.
        idx = tril(repmat((1:N)',1,N)) + triu(repmat(1:N,N,1)) - diag(1:N);
        jjacx = reshape(jac_y(:,:,idx),[T N N N]);
        jjacy = reshape(-jac_x(:,:,idx),[T N N N]);
        
    end
end