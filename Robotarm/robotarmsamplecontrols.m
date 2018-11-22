% Sample some random controls for robot arm.
function u = robotarmsamplecontrols(n,mdp_data)

if mdp_data.complex
    u = randn(n,mdp_data.udims)*5.0;
else
    %u = randn(n,2)*[8, 0;0, 0.7] + ones(n,2)*[0,0;0,0.9];
    u = randn(n,2)*[0.5*2, 0;0, 0.1*2] - ones(n,2)*[0.5 0;0 0.1];
end