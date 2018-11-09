x = linspace(-1,1,100);
y = sqrt(1-x.^2);
plot(x,y)
axis equal
hold on
y = sqrt(1-(x).^2)/2;
plot(x,y)
% syms x2 y2
% eqn = x2^2 + 2*x2*y2 + y2^2 == 1;
% solx = solve(eqn, y2)
