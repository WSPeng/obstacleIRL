% Build grid for discretization.
function [state_vals, act_vals] = buildgrid_obs(mdp_data, cells)

abounds = mdp_data.abounds;

D = size(abounds,2);
SPLITS = cells+1;
step_size = (abounds(2,:)-abounds(1,:))/cells;

% Create cell array of positions.
if D == 1
    vcell = cell(SPLITS,1);
else
    vcell = cell(SPLITS*ones(1,D));
end
all_inds = 1:SPLITS^D;  % index
all_subs = zeros(SPLITS^D,D);  %subscripts
for k=1:D
    sub_mat = repmat(1:SPLITS,horzcat(SPLITS*ones(1,k-1),1,SPLITS*ones(1,D-k)));
    all_subs(:,k) = sub_mat(:);
    vk = bounds(1,k)+(0:(SPLITS-1))*step_size(k);
    for i=all_inds
        vcell{i}(k) = vk(all_subs(i,k));
    end
end

% Convert to array.
act_vals = cell2mat(vcell(:));

%% build the state_vals
% sbounds is useless

% start from the start_point
x0 = mdp_data.start_point;

% the output is 9^5 x 4 matrix | state_vals
points = cell(n_decision,1);
for i = 1:n_decision
    if i ==1 
        pre_point = x0;
    else
        pre_point = 
    end
        point = zeros(9^i, 4);
    
end


function after_merge = merge_points(points)


