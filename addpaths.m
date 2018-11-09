% Add necessary paths for subdirectories.

if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end

if isempty(regexp(path,['General' pathsep], 'once'))
    addpath([pwd, '/General']);
end

if isempty(regexp(path,['Obstacle' pathsep], 'once'))
    addpath([pwd, '/Obstacle']);
end

if isempty(regexp(path,['Reward' pathsep], 'once'))
    addpath([pwd, '/Reward']);
end

if isempty(regexp(path,['Algorithm' pathsep], 'once'))
    addpath([pwd, '/Algorithm']);
end