% Evaluate interstate distance reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    hardevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

if isempty(x)
    % If there is no initial state, this is just a visualization call.
    r = 0;
    g = [];
    drdu = [];
    d2rdudu = [];
    drdx = [];
    d2rdxdx = [];
    gfull = [];
    Hfull = [];
    return;
end

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% This reward add hard constraint for rho and sf, which are control inputs
% r = reward.r * sum(u(:,reward.idx).^2,2);
a = 1;
upper_sf = 3.0;
l1 = log(u(:,1)-0.9);% + log(-u(:,1)+8);
l2 = log(u(:,2)-0.9);% + log(-u(:,2)+upper_sf); 
r = 1/reward.r * (l1 + l2*a); 

if nargout >= 2
    % Compute gradient with respect to controls.
    g = zeros(T,Du);
    % g(:,reward.idx) = 2.0 * reward.r * u(:,reward.idx);
    g(:,1) = 1/reward.r * ((u(:,1)-0.9).^(-1));% - (-u(:,1)+8).^(-1));
    g(:,2) = 1/reward.r * a * ((u(:,2)-0.9).^(-1));% - (-u(:,2)+upper_sf).^(-1));
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
%         d2rdudu(t,:,:) = 1/reward.r * [-((u(t,1)-0.9).^(-2)+(-u(t,1)+8).^(-2)), 0;...
%                                         0, -a* ((u(t,2)-0.9).^(-2)+(-u(t,2)+upper_sf).^(-2))];
        d2rdudu(t,:,:) = 1/reward.r * [-((u(t,1)-0.9).^(-2)), 0;...
                                       0, -a*(u(t,2)-0.9).^(-2)];
    end
end

if nargout >= 5
    drdx = zeros(T,Dx);
    d2rdxdx = zeros(T,Dx,Dx);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% useless for linear case..
if nargout >= 7
    % Compute gradient with respect to controls.
    gfull = zeros(T,T*Du);
    
    for i = 1:2
        % gfull(:,(i-1)*T + (1:T)) = diag(2.0*u(:,i)) * reward.r;
        gfull(:,(i-1)*T + (1:T)) = diag(g(:,i));
    end
    
    % Compute Hessian.
    % Construct prototypical constant Hessian.
    hessmat = zeros(Du,Du,T,T,T);
    for ui = 1:Du
        for uj = 1:Du    
            for i=1:T
                % like the diagonal in 3D,
                hessmat(ui,uj,i,i,i) = d2rdudu(i,ui,uj);
            end
        end
    end
    hessmat = reward.r * hessmat;
    Hfull = zeros(T,T*Du,T*Du);
    for d = 1:2
        strt = (d-1)*T;
        Hfull(1:T,(strt+1):(strt+T),(strt+1):(strt+T)) = ...
            hessmat(d,d,:,:,:);
    end
end
