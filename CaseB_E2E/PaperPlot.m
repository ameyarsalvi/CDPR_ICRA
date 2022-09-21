FS = 32;
 
f1 = figure('color','w');
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;


subplot(2,2,1)
plot(B(1,:),B(2,:),'--')
hold on
plot(Xm,Ym)
%xlim([6,18])
%ylim([-20,20])
box on 
title('Trajectory')
xlim([0.1 0.8])
ylim([0.1 0.8])
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
%legend('Desired','Actual')

subplot(2,2,2)
plot(X(11,:))
hold on
plot(X(12,:))
title('Wrench Quality & Manipulability')
xlabel('Time','Interpreter','latex')
ylabel('Pose Quality','Interpreter','latex')
%legend('$\kappa$','$\lambda$','Interpreter','latex')

subplot(2,2,3)
hold on
plot(X(13,:))
plot(X(14,:))
plot(X(15,:))
plot(X(16,:))
hold off
title('Actual Tension')
xlabel('Time','Interpreter','latex')
ylabel('Cable Tension','Interpreter','latex')
%legend('$\tau_{1}$','$\tau_{2}$','$\tau_{3}$','$\tau_{4}$','Interpreter','latex')

subplot(2,2,4)
hold on
plot(X(17,:))
plot(X(18,:))
plot(X(19,:))
plot(X(20,:))
hold off
title('Actual Slider Positions')
xlabel('Time','Interpreter','latex')
ylabel('Slider Position','Interpreter','latex')
%legend('$L_{s1}$','$L_{s2}$','$L_{s3}$','$L_{s4}$','Interpreter','latex')

print(f1,'ResultsE2E','-depsc')
%%
