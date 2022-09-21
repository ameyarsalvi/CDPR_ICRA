FS = 32;
 
f1 = figure('color','w');
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;


subplot(2,2,1)
plot(B(1,:),B(2,:),'--')
hold on
plot(X(1,:),X(2,:),'-')
xlim([0.1,0.8])
ylim([0.2,0.6])
box on 
title('Trajectory','Interpreter','latex')
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
%legend('Desired','Actual','starting point')

subplot(2,2,2)
plot(env.K_vec)
hold on
plot(env.M_vec)
title('Wrench Quality and Manipulability' ,'Interpreter','latex')
%xlim([0.1,0.8])
ylim([0,1.1])
xlabel('Time','Interpreter','latex')
%ylabel('Pose Quality','Interpreter','latex')
%legend('$\kappa$','$\lambda$','Interpreter','latex')
box on 


subplot(2,2,3)
hold on
plot(Ac(1,:))
plot(Ac(2,:))
plot(Ac(3,:))
plot(Ac(4,:))
hold off
title('Cable Tensions $(\mathbf{\tau})$','Interpreter','latex')
xlabel('Time','Interpreter','latex')
%ylabel('Cable Tensions $(\mathbf{\tau})$','Interpreter','latex')
%legend('$\tau_{1}$','$\tau_{2}$','$\tau_{3}$','$\tau_{4}$','Interpreter','latex')
box on 

subplot(2,2,4)
hold on
plot(X(7,:))
plot(X(8,:))
plot(X(9,:))
plot(X(10,:))
hold off
title('Slider Positions $(\mathbf{l_{s}})$','Interpreter','latex')
xlabel('Time','Interpreter','latex')
%ylabel('Slider Position','Interpreter','latex')
box on 


print(f1,'ResultsOptimT','-depsc')
%%
