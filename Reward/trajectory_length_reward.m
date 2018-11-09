function [r] = trajectory_length_reward(reward, states)

% INPUT
% states: d x T

% the states here are the vector of trajectory it traveled.

% the length of trajectory
T = length(states);

% Convert states to Cartesian space. Here the states is the position given
% by Simulation function
pts = states;

d = diff(pts);

% Compute value.
r = reward.r(2)*sum(sqrt(sum(d.^2,2))); % the rbf equation