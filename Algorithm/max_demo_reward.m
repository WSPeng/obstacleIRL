function theta = max_demo_reward(mdp_data, test_result)
% max theta to max difference of optimal to demos/
% theta*R_dist + theta*R_length

R_dist = mdp_data.grid_reward(:,3);
R_length = mdp_data.grid_reward(:,4);
matrix_R = cell2mat(test_result.R)';
R_dist_example = matrix_R(:,1);
R_length_example = matrix_R(:,2);


% search on ratio of two theta
for idx = linspace(0.01,100,200) 
    R = R_dist + idx*R_length;
    R = -R;
    R_example = R_dist_example + idx*R_length_example;
    R_example = -R_example';
    % for any R > R_example calculate the difference to examples
    % and the length(R) should be small
    
    R_ = R(R >= max(R_example));
    
    if length(R_)<1/8*length(R_dist) && ~isempty(R_)
        disp(length(R_));
        disp(idx);
        % after determine the feasible thetas, max to get the optimal one
        % and calculate the distance of global optimal one to examples in
        % reward space.
        
        % [mesh_e,mesh_r] = meshgrid(R_example, R_);
        % dist_sum = sum(mesh_r - mesh_e,2);
        % [D,I] = min(dist_sum);
        % I_ = R == R_(I);
        % disp(mdp_data.grid_reward(I_,:));
        
        % find the max reward index
        I_max = R == max(R_);
        disp(mdp_data.grid_reward(I_max,:))
        
        % calculate the distance
        mesh_max = max(R_)*ones(5,1);
        dist = sum(mesh_max - R_example')/sum(abs(R_example))*100;
        disp(dist)
        disp('00')
        disp((max(R_)-R_example)/(max(R_)-min(R)))
        
        figure;
        axis([0,8,0.9,1.6])
        hold on
        plot_example()
        [~,xxx] = intersect(R,max(R_),'stable');
        % [~,xxx] = intersect(R,R_,'stable');
        plot(mdp_data.grid_reward(xxx,1),mdp_data.grid_reward(xxx,2),'o')
        
    end
    
    % minimize the distance
    
    
end


theta = ones(2,1);

function plot_example()
    rho = [5,    6, 3,   4,   4];
    sf =  [1.25, 1, 1.6, 1.4, 1.5];
    % rho = [8,    7.5, 7,   7.8,   7.6];
    % sf =  [1.55, 1.6, 1.5, 1.48,  1.59];
    
    plot(rho,sf,'*')