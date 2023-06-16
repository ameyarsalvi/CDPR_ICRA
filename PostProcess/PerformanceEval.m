
load("wskAA.mat")
load("wskBB.mat")
load("wskCC.mat")
load("wskDD.mat")

%%
BAR = [M1_OT,M1_E,M1_OTT,M1_D];
K_mean = [M2_OT,M2_E,M2_OTT,M2_D];
M_mean = [M3_OT,M3_E,M3_OTT,M3_D];

figure
bar(BAR)

figure
bar(K_mean)

figure
bar(M_mean)

%%
figure
x = 1:1:3;
y = [M1_OT M1_E M1_D];
err = [S1_OT S1_E S1_D];
errorbar(x,y,err,'vertical','or')

hold on
x2 = 5:1:7;
y2 = [M2_OT M2_E M2_D];
err2 = [S2_OT S2_E S2_D];
errorbar(x2,y2,err2,'vertical','ob')

hold on
x3 = 9:1:11;
y3 = [M3_OT M3_E M3_D];
err3 = [S3_OT S3_E S3_D];
errorbar(x3,y3,err3,'vertical','ok')


xlim([0 12])
ylim([0 1])

%%

figure
x = 1:5:11;
y = [M1_OT M2_OT M3_OT];
err = [S1_OT S2_OT S3_OT];
errorbar(x,y,err,'vertical','or')

hold on
x2 = 2:5:12;
y2 = [M1_E M2_E M3_E];
err2 = [S1_E S2_E S3_E];
errorbar(x2,y2,err2,'vertical','ob')

hold on
x3 = 3:5:13;
y3 = [M1_D M2_D M3_D];
err3 = [S1_D S2_D S3_D];
errorbar(x3,y3,err3,'vertical','ok')


xlim([0 14])
ylim([0 1])


%%

FS = 32;
 
f1 = figure('color','w');
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;

subplot(3,1,1)
%x = 1:6:13;
x=1:1:4;
y = [M1_OT M2_OT M3_OT];
err = [S1_OT S2_OT S3_OT];
errorbar(x,y,err,'dr','CapSize',10)
 xlim([0.5 4.5])
% ylim([0 1])

subplot(3,1,2)
%hold on
%x2 = 2:6:14;
x2=1:1:4;
y2 = [M1_E M2_E M3_E];
err2 = [S1_E S2_E S3_E];
errorbar(x2,y2,err2,'vertical','db')
% xlim([0.5 3.5])
% ylim([0 1])

subplot(3,1,3)
%hold on
x3 = 3:6:15;
%x3=1:1:3;
y3 = [M1_OTT M2_OTT M3_OTT];
err3 = [S1_OTT S2_OTT S3_OTT];
errorbar(x3,y3,err3,'vertical','dm')

%subplot(3,1,3)
hold on
x4 = 4:6:16;
%x3=1:1:3;
y4 = [M1_D M2_D M3_D];
err4 = [S1_D S2_D S3_D];
errorbar(x4,y4,err4,'vertical','dk')


xlim([0.5 16.5])
ylim([0 1.1])
box on 
title('RMSE, $\kappa$, $\lambda$','Interpreter','latex')
%xlabel('X [m]','Interpreter','latex')
%ylabel('Y [m]','Interpreter','latex')

%%

FS = 40;
 
f1 = figure('color','w');
ax = gca;
ax.FontName= 'Times New Roman';
ax.FontSize = FS;

subplot(3,3,1)
%x = 1:6:13;
x=1:1:4;
y = [M1_OT M1_E M1_OTT M1_D];
err = [S1_OT S1_E S1_OTT S1_D];
errorbar(x,y,err,'dr','CapSize',10)
 xlim([0.5 4.5])
% ylim([0 1])
box on
xticks(1:1:4)
xticklabels({'\pi_{1}','\pi_{2}','\pi_{3}','\pi_{4}'})
title(' Tracking RMSE','Interpreter','latex')

subplot(3,3,2)
%hold on
%x2 = 2:6:14;
x2=1:1:4;
y2 = [M2_OT M2_E M2_OTT M2_D];
err2 = [S2_OT S2_E S2_OTT S2_D];
errorbar(x2,y2,err2,'vertical','db')
 xlim([0.5 4.5])
% ylim([0 1])
box on
%set(gca,'XTick',[])
xticks(1:1:4)
xticklabels({'\pi_{1}','\pi_{2}','\pi_{3}','\pi_{4}'})
title('$\kappa$','Interpreter','latex')



subplot(3,3,3)
%hold on
%x3 = 3:6:15;
x3=1:1:4;
y3 = [M3_OT M3_E M3_OTT M3_D];
err3 = [S3_OT S3_E S3_OTT S3_D];
errorbar(x3,y3,err3,'vertical','dm')
 xlim([0.5 4.5])
box on
xticks(1:1:4)
xticklabels({'\pi_{1}','\pi_{2}','\pi_{3}','\pi_{4}'})
title('$\lambda$','Interpreter','latex')


% xlim([0.5 16.5])
% ylim([0 1.1])
 

%xlabel('X [m]','Interpreter','latex')
%ylabel('Y [m]','Interpreter','latex')

%print(f1,'Comparison','-depsc')