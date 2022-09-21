%-1000*sqrt(sum(this.track_error.^2)) - 10*norm([0.1;0.1] -this.X(4:5));


x = linspace(0,0.9899,1000);

y = linspace(0,0.01,1000);

figure
plot(x,x.^2)
hold on
plot(y,10000*y.^2)
legend('Tracking Error','Velocity Error')