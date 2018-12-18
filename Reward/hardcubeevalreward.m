% Evaluate interstate distance reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    hardcubeevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

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
[l1, l1g, l1gg] = barr(1 - u(:,1));
[l2, l2g, l2gg] = barr(0.9 - u(:,2));
% the upper bound 
%[l3, l3g, l3gg] = barr(u(:,1) - 2000);
%[l4, l4g, l4gg] = barr(u(:,2) - 2000);

r = reward.r * sum((l1 + l2*a), 2); 
% r = reward.r * sum((l1 + l2*a + l3 + l4*a), 2); 

if nargout >= 2
    % Compute gradient with respect to controls.
    g = zeros(T,Du);
    % g(:,reward.idx) = 2.0 * reward.r * u(:,reward.idx);
%     g(:,1) = reward.r * (l1g + l3g);
%     g(:,2) = reward.r * a * (l2g + l4g);
    g(:,1) = reward.r * (l1g);
    g(:,2) = reward.r * a * (l2g);
end

if nargout >= 3
    drdu = zeros(T,Du);
    %drdu(:,reward.idx) = 2.0 * reward.r * u(:,reward.idx);
    drdu(:,1:2) = g(:,1:2);
    d2rdudu = zeros(T,Du,Du);
    mask = zeros(Du,1);
    for t = 1:T
%         mask(1,1) = l1gg(t) + l3gg(t);
%         mask(2,1) = a*l2gg(t) + a*l4gg(t);
        mask(1,1) = l1gg(t);
        mask(2,1) = a*l2gg(t);
        d2rdudu(t,:,:) = reward.r * diag(mask);
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

end


function [r, rg, rgg] = barr(u)
    u(u<0) = 0;
    k = 100;
    r = k*u.^3;
    if nargout >=2
        rg = k*3*u.^2;
    end
    if nargout >=3
        rgg = k*6*u;
    end       
end
