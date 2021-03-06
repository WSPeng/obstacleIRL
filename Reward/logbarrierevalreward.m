% Evaluate interstate distance reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    logbarrierevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% This reward add hard constraint for rho and sf, which are control inputs
% r = reward.r * sum(u(:,reward.idx).^2,2);
a = 5;
l1 = log(u(:,1)-0.9) + log((-u(:,1)+8));
l2 = log(u(:,2)-0.9) + log(-u(:,2)+1.6); 
r = 1/reward.r * (l1 + l2*a); 

if nargout >= 2
    % Compute gradient with respect to controls.
    g = zeros(T,Du);
    % g(:,reward.idx) = 2.0 * reward.r * u(:,reward.idx);
    g(:,1) = 1/reward.r * ((u(:,1)-0.9).^(-1) - (-u(:,1)+8).^(-1));
    g(:,2) = 1/reward.r * ((u(:,2)-0.9).^(-1) - (-u(:,2)+1.6).^(-1));
end

if nargout >= 3
    drdu = zeros(T,Du);
    %drdu(:,reward.idx) = 2.0 * reward.r * u(:,reward.idx);
    drdu(:,reward.idx) = g(:,1:2);
    d2rdudu = zeros(T,Du,Du);
    mask = zeros(Du,1);
    mask(reward.idx,1) = 1;
    for t=1:T
        % d2rdudu(t,:,:) = reward.r * 2.0 * diag(mask);
        d2rdudu(t,:,:) = -1/reward.r * [((u(t,1)-0.9).^(-2) - (-u(t,1)+8).^(-2)),0;...
                                        0,((u(t,2)-0.9).^(-2) - (-u(t,2)+1.6).^(-2))];
    end
end

if nargout >= 5
    drdx = zeros(T,Dx);
    d2rdxdx = zeros(T,Dx,Dx);
end

if nargout >= 7
    % Compute gradient with respect to controls.
    gfull = zeros(T,T*Du);
    
    for i = reward.idx
        gfull(:,(i-1)*T + (1:T)) = diag(2.0*u(:,i)) * reward.r;
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
