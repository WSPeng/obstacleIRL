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

if isempty(regexp(path,['Auglag' pathsep], 'once'))
    addpath([pwd, '/Auglag']);
end

if isempty(regexp(path,['GPIRL' pathsep], 'once'))
    addpath([pwd, '/GPIRL']);
end

if isempty(regexp(path,['Laplace' pathsep], 'once'))
    addpath([pwd, '/Laplace']);
end

if isempty(regexp(path,['MaxEnt' pathsep], 'once'))
    addpath([pwd, '/MaxEnt']);
end

if isempty(regexp(path,['Robotarm' pathsep], 'once'))
    addpath([pwd, '/Robotarm']);
end

if isempty(regexp(path,['Utilities' pathsep], 'once'))
    addpath([pwd, '/Utilities']);
end
addpath Utilities/minFunc
addpath Utilities/plot2svg

addpath FastHess
addpath Test
addpath Objectworld