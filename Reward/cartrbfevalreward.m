% Gaussian rbf in Cartesian space
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    cartrbfevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)
%    cartrbfevalreward(reward,states)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Here the states is the position given by Simulation function
pts = states;

% Compute distances.
d = bsxfun(@minus, reward.pos, pts);

% Compute value.
r = reward.r(1)*exp(-0.5*reward.width*sum(d.^2,2)); % the rbf equation

if nargout >= 2
    % Compute gradient.
    drdx = (reward.width*bsxfun(@times,d,r))*mdp_data.sensor_pseudoinverse';
    g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end

if nargout >= 3
    % Gradients with respect to controls are always zero.
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
end

if nargout >= 6
    % drdx has already been computed.
    d2rdxdx = zeros(T,Dx,Dx);
    for t=1:T
        D = mdp_data.sensor_pseudoinverse*...
            ((reward.width^2)*bsxfun(@times,d(t,:),d(t,:)')*r(t) - reward.width*r(t)*eye(2))*...
            mdp_data.sensor_pseudoinverse';
        d2rdxdx(t,:,:) = D;
    end
end

if nargout >= 7
    % Compute gfull.
    % Convert gradient to T x TD matrix.
    drdxmat = zeros(T,T*Dx);
    for i=1:Dx
        drdxmat(:,(i-1)*T + (1:T)) = diag(drdx(:,i));
    end

    % Compute gradient with respect to controls.
    gfull = drdxmat * dxdu';
    
    % Compute Hfull.
    Hfull = zeros(T,T*Du,T*Du);
    for t=1:T
        idxs = (0:(Dx-1))*T + t;
        D = mdp_data.sensor_pseudoinverse*...
            ((reward.width^2)*bsxfun(@times,d(t,:),d(t,:)')*r(t) - reward.width*r(t)*eye(2))*...
            mdp_data.sensor_pseudoinverse';
        Hfull(t,:,:) = dxdu(:,idxs) * D * dxdu(:,idxs)' + sum(bsxfun(@times,permute(drdx(t,:),[1 3 2]),d2xdudu(:,:,idxs)),3);
    end
end
