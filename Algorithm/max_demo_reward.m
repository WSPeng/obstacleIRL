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
    
    if length(R_)<1/10*length(R_dist) && ~isempty(R_)
        disp(length(R_));
        disp(idx);
        % the feasible theta, then need to find out the optimal point
        % for any feasible, calculate the difference
        [mesh_e,mesh_r] = meshgrid(R_example, R_);
        dist_sum = sum(mesh_r - mesh_e,2);
        [~,I] = max(dist_sum);
        
        I_ = R == R_(I);
        disp(mdp_data.grid_reward(I_,:));
    end
    
end







for i = 1 : length()
    
    
    
end

theta = ones(2,1);

