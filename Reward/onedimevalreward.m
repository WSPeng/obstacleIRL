% Evaluate interstate distance reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    onedimevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% This reward add hard constraint for rho and sf, which are control inputs
% r = reward.r * sum(u(:,reward.idx).^2,2);
r = reward.r * x(:,1);

if nargout >= 2
    % Compute gradient with respect to controls.
    g = zeros(T,Du);
end

if nargout >= 3
    drdu = zeros(T,Du);
    drdu(:,reward.idx) = g(:,1:2);
    d2rdudu = zeros(T,Du,Du);
end

if nargout >= 5
    drdx = zeros(T,Dx);
    drdx(:,1) = ones(T,1);
    d2rdxdx = zeros(T,Dx,Dx);
end

if nargout >= 7
    % Compute gradient with respect to controls.
    gfull = zeros(T,T*Du);
    
    for i = reward.idx
        gfull(:,(i-1)*T + (1:T)) = diag(ones(T,1)) * reward.r;
    end
    
    % Compute Hessian.
    % Construct prototypical constant Hessian.
    hessmat = zeros(T,T,T);
    for i=1:T
        hessmat(i,i,i) = 2;
    end
    hessmat = reward.r * hessmat;
    Hfull = zeros(T,T*Du,T*Du);
    for d=reward.idx
        strt = (d-1)*T;
        Hfull(1:T,(strt+1):(strt+T),(strt+1):(strt+T)) = ...
            hessmat;
    end
end
