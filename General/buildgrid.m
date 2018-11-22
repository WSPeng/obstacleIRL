% Build grid for discretization.
function vals = buildgrid(mdp_data, bounds, cells, quad)

if quad
    bounds = signbounds.*sqrt(abs(bounds));
end

D = size(bounds,2);
SPLITS = cells+1;
step_size = (bounds(2,:)-bounds(1,:))/cells;

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
vals = cell2mat(vcell(:));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% remove the grid inside the ecllipsoid
if mdp_data.abounds(2,1) ~= 8
    vcell2 = {};        
    a = mdp_data.obs_params.opt_sim.obstacle{1}.a;
    x0 = mdp_data.obs_params.opt_sim.obstacle{1}.x0;
    for i = 1:size(vals, 1)
        x1 = vcell{i}(1);
        x2 = vcell{i}(2);
        if (((x1-x0(1))/a(1))^2 + ((x2-x0(2))/a(2))^2) >= 1
            vcell2{end+1,1} = vcell{i};
        end
    end
    
    vals = cell2mat(vcell2);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if quad
    vals = sign(vals).*(vals.^2);
end
