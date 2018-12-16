clear all
%% log
% u = 0.9:0.000001:1.6; 
% u = u';
% l1 = log(u(:,1)-0.9) + log((-u(:,1)+1.6));
% l = 1/-10000*l1;
% plot(u(:,1), l)

%% rbf
figure
d = -5:0.2:5;
width = 1;
[X,Y] = meshgrid(d,d);
% r = exp(-0.5*width*(d.^2));
% E = eye(2);
E = [0.5,0.9;0,1];
for i = 1:size(X,1)
    for j = 1:size(X,2)
        D = [X(i,j);Y(i,j)];
        r(i,j) = exp(-0.5*width*D'*E*D);
    end
end
% plot(d,r)
surf(X,Y,r)


%% soft relu
figure
x = -2:0.1:10;
straight = 1:0.1:10;
plot(straight, straight, '-.')
hold on
y = log(1 + exp(x-1)) + 1;
plot(x,y)

der = 1./(1 + exp(-x+1));
plot(x,der)

%% cube

u = 0.0:0.001:3.0;
l1 = barr(1-u) + barr(u-2);
plot(u, l1)

function r = barr(u)
    u(u<0) = 0;
    r = 100*u.^3;
end

