FS = 32;
 
f1 = figure('color','w');
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;


subplot(2,2,1)
plot(B(1,:),B(2,:),'--')
hold on
plot(X(1,:),X(2,:),'--')
plot(X(1,1),X(2,1),'*g');
xlim([0.1 0.7])
ylim([0.1 0.7])
box on 
title('Trajectory')
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
%legend('Desired','Actual','starting point')

subplot(2,2,2)
plot(kappa)
hold on
plot(mom)
%xlim([6,18])
%ylim([-20,20])
box on 
title('Trajectory')
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
%legend('Desired','Actual','starting point')



subplot(2,2,3)
hold on
plot(Ac(1,:))
plot(Ac(2,:))
plot(Ac(3,:))
plot(Ac(4,:))
hold off
title('Cable Tension')
xlabel('Time','Interpreter','latex')
ylabel('$\tau$','Interpreter','latex')
%legend('$\tau_{1}$','$\tau_{2}$','$\tau_{3}$','$\tau_{4}$','Interpreter','latex')

subplot(2,2,4)
hold on
plot(X(7,:))
plot(X(8,:))
plot(X(9,:))
plot(X(10,:))
hold off
title('Slider Position')
xlabel('Time','Interpreter','latex')
ylabel('$\l_{s}$','Interpreter','latex')
%legend('$\tau_{1}$','$\tau_{2}$','$\tau_{3}$','$\tau_{4}$','Interpreter','latex')


print(f1,'ResultsFinalDecoup','-depsc')
%%
