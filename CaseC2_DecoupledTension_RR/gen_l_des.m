

clc
clear

load('onlyslidersagent2.mat','agent')
generatePolicyFunction(agent)

P0 = [0.4;0.6];
P1 = [0.2;0.4];
P2 = [0.6;0.2];

[B,B_dot] = bezier_curve(P0,P1,P2,0.05,5);

policy = coder.loadRLPolicy("agentData.mat");

for i = 1:1:size(B,2)-1
 l_des(:,i) = getAction(policy,[B(1,i+1),B(2,i+1),0.5]);
end

for i = 1:1:size(B,2)-1

model.m = 0.1; %kg
model.dx = 0.5;
model.dy = 0.5;
model.dz = 0.01;
model1.fail = 0;
m1 = fourPRPR(model,model1);
m1.plot_cables([B(1,i+1),B(2,i+1),0.5],l_des(:,i))
hold on;
plot(B(1,:),B(2,:),'-')
hold off;
end