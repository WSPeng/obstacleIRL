% Sample some random states in the robot arm domain.
function s = robotarmsamplestates(n,mdp_data)

if mdp_data.complex
    s = [(rand(n,mdp_data.links)*pi*2 - pi) zeros(n,mdp_data.links)];
else
    s = [0, 2.2, 8*rand(), 0.7*rand()+0.9];
end