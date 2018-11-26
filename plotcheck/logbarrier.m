
u = 0.9:0.000001:1.6; 
u = u';
l1 = log(u(:,1)-0.9) + log((-u(:,1)+1.6));
l = 1/-10000*l1;
plot(u(:,1), l)

%% rbf
figure
d = -5:0.01:5;
width = 1;
r = exp(-0.5*width*(d.^2));
plot(d,r)