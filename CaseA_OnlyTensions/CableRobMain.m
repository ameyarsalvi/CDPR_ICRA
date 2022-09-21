%% full trajectory learning for trajectory tracking
%% Description
% 1. The trajectories are defined as Bezier curves. Only tensions
% manipulated.

clc
clear all
%% Data
nsteps = 200;
nepisodes = 500;

inputs.train = true;
inputs.P0 = [0.2;0.6];
inputs.P1 = [0.3;0.5];
inputs.P2 = [0.4;0.3];
inputs.nsteps = nsteps;
inputs.Ts = 0.05;

%% Validate Environment
env=CDPRENV(inputs);
validateEnvironment(env);


observationInfo = getObservationInfo(env);
numObservations = observationInfo.Dimension(1);
actionInfo = getActionInfo(env);
numActions = actionInfo.Dimension(1);

%%

%L = 90; % number of neurons
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(50,'Name','fc1')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu1')
    fullyConnectedLayer(50,'Name','fc3')
    reluLayer('Name','relu2')
    fullyConnectedLayer(10,'Name','fc2')
    fullyConnectedLayer(1,'Name','fc9')];
    
actionPath = [featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(50,'Name','fc10')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
    
criticNetwork = connectLayers(criticNetwork,'fc10','add/in2');



%%

criticOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-3);

critic = rlQValueRepresentation(criticNetwork,observationInfo,actionInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

%%

actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    reluLayer('Name','relu1')
    fullyConnectedLayer(50,'Name','fc1')
    reluLayer('Name','relu2')
    fullyConnectedLayer(20,'Name','fc7')
    reluLayer('Name','relu3')
    fullyConnectedLayer(4,'Name','fc8')
    tanhLayer('Name','tanh1')
    
    scalingLayer('Name','ActorScaling1','Scale',[0.45;0.45;0.45;0.45],'Bias',[0.55;0.55;0.55;0.55])];
    %scalingLayer('Name','ActorScaling1','Scale',actionInfo.UpperLimit-actionInfo.LowerLimit,'Bias',(actionInfo.UpperLimit-actionInfo.LowerLimit)/2)];

%%

actorOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-3);
actor = rlDeterministicActorRepresentation(actorNetwork,observationInfo,actionInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling1'},actorOptions);

%%

%agentOptions = rlDDPGAgentOptions(...
agentOptions = rlTD3AgentOptions(...
    'SampleTime',inputs.Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',10000,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',512);

agentOptions.ExplorationModel.VarianceMin = 0.0001;
agentOptions.ExplorationModel.Variance = (0.2)^2*ones(4,1);
agentOptions.ExplorationModel.VarianceDecayRate = 1e-3;

%%

agent = rlTD3Agent(actor,critic,agentOptions);
%%
maxepisodes = nepisodes;
maxsteps = nsteps;
trainingOpts = rlTrainingOptions('MaxEpisodes',maxepisodes,'MaxStepsPerEpisode',maxsteps,'Verbose',true,'StopTrainingCriteria','EpisodeCount','StopTrainingValue',maxepisodes,'Plots',"training-progress");

trainOpts.UseParallel = true;
trainOpts.ParallelizationOptions.Mode = 'async';
trainOpts.ParallelizationOptions.StepsUntilDataIsSent = 32;
trainOpts.ParallelizationOptions.DataToSendFromWorkers = 'Experiences';

%%
trainingStats = train(agent,env,trainingOpts);

%%

save("CDPR_Agent_OnlyT.mat",'agent')

%% Validate Environment
inputs.train = false;

inputs.P0 = [0.2;0.5];
inputs.P1 = [0.4;0.2];
inputs.P2 = [0.7;0.4];

env=CDPRENV(inputs);

simOpts = rlSimulationOptions('MaxSteps',nsteps);
experience = sim(env,agent,simOpts);

%%
X = experience.Observation.ActualEndEffectorPosition_Velocity_PositionError_VelocityError.Data;
X = squeeze(X);
Act = experience.Action.DesiredCableTensions.Data;
Ac = squeeze(Act);
kappa =env.K_vec;
mom = env.M_vec;


%% For Plotting

[B,B_dot] = bezier_curve(env.P0,env.P1,env.P2,env.Ts,env.Tf);
%%

Xm = X(1,:);
Ym = X(2,:);
ErrorX = X(5,:);
ErrorY = X(6,:);



f2 = figure('color','w');
plot(B(1,:),B(2,:),'-')
hold on
plot(Xm,Ym,'--')
plot(Xm(1),Ym(1),'*g');
title('Trajectory')
xlabel('X')
ylabel('Y')
legend('Desired','Actual','starting point')

f3 = figure('color','w');
plot(ErrorX)
hold on
plot(ErrorY)
title('Error')
xlabel('Time')
ylabel('Error')
legend('E1','E2')


f4 = figure('color','w');
hold on
plot(Ac(1,:))
plot(Ac(2,:))
plot(Ac(3,:))
plot(Ac(4,:))
hold off
title('Desired Tension')
xlabel('Time')
ylabel('Action')
legend('T1','T2','T3','T4')

f5 = figure('color','w');
plot(kappa)
hold on
plot(mom)
title('Wrench Quality and Manipulability')
xlabel('Time')
ylabel('Value')
legend('K','M')
%%

model.m = 0.1; %kg
model.dx = 0.5;
model.dy = 0.5;
model.dz = 0.01;
model1.fail = 0;
m1 = fourPRPR(model,model1);

myVideo = VideoWriter('only-tension_V1');
myVideo.FrameRate = 10;
myVideo.Quality = 50;
open(myVideo);

f1 = figure();
f1.Position = [100 100 1200 675];

for i = 1:1:(size(X,2)-1)
    %subplot(4,4,[1,2,3,5,6,7,9,10,11,13,14,15])
    subplot(3,3,[1,2,4,5,7,8])
    m1.plot_cables(X(1:3,i),[0;0;0;0])
    hold on
    plot(B(1,1:i),B(2,1:i),'-','LineWidth',1.25)
    plot(Xm(1:i),Ym(1:i),'-','LineWidth',1.25)
    hold off
    box on 
    t=title('\textbf{Trajectory Tracking}','Interpreter','latex');
    t.FontSize = 12;
    xlabel('X [m]','Interpreter','latex')
    ylabel('Y [m]','Interpreter','latex')
    lgd=legend('','','','','','','Desired','Realized','Interpreter','latex');
    lgd.FontSize = 10;
    lgd.ItemTokenSize  =[5 1];

    subplot(3,3,3)
    hold on
    plot(Ac(1,1:i),'-r')
    plot(Ac(2,1:i),'-b')
    plot(Ac(3,1:i),'-g')
    plot(Ac(4,1:i),'-k')
    hold off
    box on
    t=title('\textbf{Cable Tension}','Interpreter','latex');
    t.FontSize = 12;
    xlabel('Time [s]','Interpreter','latex')
    ylabel('$\tau$ [N]','Interpreter','latex')
    lgd=legend('$\tau_{1}$','$\tau_{2}$','$\tau_{3}$','$\tau_{4}$','Interpreter','latex');
    lgd.FontSize = 10;
    lgd.ItemTokenSize  =[3 1];

    subplot(3,3,6)
    hold on
    plot(kappa(1:i),'-r')
    plot(mom(1:i),'-b')
    hold off
    box on
    t=title('\textbf{Pose Quality}','Interpreter','latex');
    t.FontSize = 12;
    xlabel('Time [s]','Interpreter','latex')
    %ylabel('Y [m]','Interpreter','latex')
    lgd=legend('$\kappa$','$\lambda$','Interpreter','latex');
    lgd.FontSize = 10;
    lgd.ItemTokenSize  =[3 1];


    frame = getframe(gcf);
    writeVideo(myVideo,frame);
end

close(myVideo);

