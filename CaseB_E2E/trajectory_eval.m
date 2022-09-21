
%% Evaluating the Agent

clc
clear all
close all

inputs.train = false;
inputs.P0 = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2];
inputs.P1 = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2];
inputs.P2 = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2];
nsteps = 200;
inputs.nsteps = nsteps;
inputs.Ts = 0.05;
env=CDPRENV(inputs);
validateEnvironment(env);
env.Tf = nsteps*inputs.Ts;
env.Ts = inputs.Ts;

observationInfo = getObservationInfo(env);
numObservations = observationInfo.Dimension(1);
actionInfo = getActionInfo(env);
numActions = actionInfo.Dimension(1);
%rng(0)
%%
%load('CDPR_Agent_BezTrajB.mat');

%% Select a set of random waypoints in the workspace and draw bezier curves between them
%waypoints = 0.4*rand(2,5) + 0.2*ones(2,5)
waypoints = [0.3 0.4 0.45 0.5 0.6;0.3 0.5 0.45 0.3 0.5];

nw = size(waypoints,2);
env.Ts
env.Tf
%[B,B_dot] = bezier_curve(waypoints,env.Ts,env.Tf);

%%
Xe_current_vec = [];
Error = [];
Tensions = [];
Slider_pos = [];
Xe_traj = [];
t = [0];

for i = 3:2:nw
    env.P0 = waypoints(:,i-2);
    env.P1 = waypoints(:,i-1);
    env.P2 = waypoints(:,i);

    [B,B_dot] = bezier_curve(env.P0,env.P1,env.P2,env.Ts,env.Tf);

    simOpts = rlSimulationOptions('MaxSteps',nsteps);
    experience = sim(env,agent,simOpts);

    new_states = experience.Observation.ActualEndEffectorPosition_Velocity_PositionError_VelocityError_.Data;
    new_states_ = squeeze(new_states);
    X = new_states_(1:8,:);
    kappa = new_states_(9,:);
    joint_states = experience.Action.DesiredCableTensions_DesiredSliderPositions.Data;
    joint_states = squeeze(joint_states);

    cable_tension = joint_states(1:4,:);
    slider_position = joint_states(5:8,:);

    total_steps = env.steps;
    t = [t,t(end)+[2:total_steps]*inputs.Ts];

    Xe_traj = [Xe_traj,B(:,2:end)];
    Xe_current_vec = [Xe_current_vec, X(1:4,2:end)];
    Error = [Error, X(5:6,2:end)];
    Tensions = [Tensions, cable_tension];
    Slider_pos = [Slider_pos,slider_position];



end


%%

f1 = figure('color','w');
plot(Xe_traj(1,:),Xe_traj(2,:),'-')
hold on
plot([env.P0(1),env.P1(1),env.P2(1)],[env.P0(2),env.P1(2),env.P2(2)],'*')
plot(Xe_current_vec(1,:),Xe_current_vec(2,:),'--')
plot(Xe_current_vec(1,1),Xe_current_vec(2,1),'*')
title('Trajectory')
xlabel('X')
ylabel('Y')
legend('Desired','waypoints','Actual','Initial position')
%set(f1,'Position',[2000 10 540 500])


f3 = figure('color','w');
plot(Error(:,:)')
title('Error')
xlabel('Time')
ylabel('State Error')
legend('Ex','Ey')
%set(f3,'Position',[2500 10 540 500])


f4 = figure('color','w');
hold on
plot(Tensions(1,:))
plot(Tensions(2,:))
plot(Tensions(3,:))
plot(Tensions(4,:))
hold off
title('Desired Tension')
xlabel('Time')
ylabel('Action')
legend('T1','T2','T3','T4')
set(f4,'Position',[3000 10 540 500])

f5 = figure('color','w');
hold on
plot(Slider_pos(1,:))
plot(Slider_pos(2,:))
plot(Slider_pos(3,:))
plot(Slider_pos(4,:))
hold off
title('Desired Slider Positions')
xlabel('Time')
ylabel('Action')
legend('Ls1','Ls2','Ls3','Ls4')
set(f5,'Position',[3000 10 540 500])



%% Smoothing and applying the cable tension

% Common model parameters
model.m = 0.1; %kg
model.k0 = [5;5;5;5]; % N/m^2
model.dx = 0.5;
model.dy = 0.5;
model.dz = 0.01;
model1.fail = 0;
m1 = fourPRPR(model,model1);

try
clear X_dyn X_dyn_vec
catch
end

Tk = [0.1;0.1;0.1;0.1];
Lsk = [0.1;0.1;0.1;0.1];

X_dyn(:,1) = [waypoints(:,1)-0.01;0;0;0];
l = [0;0;0;0];
dt_dyn = 0.001;
Ts = 0.01;
Tk_vec = [];
Lsk_vec = [];
ek0 = [0;0;0;0];
ik0 = [0;0;0;0];
X_dyn_vec = [];
e = [0;0;0;0];


for j = 1:1:length(t) -1

    Tk_vec = [Tk_vec,Tk];
    Lsk_vec = [Lsk_vec,Lsk];
    Td = Tensions(:,j);
    ek = (Td - Tk);
    ik = ik0 + Ts*e;
    e = 0.1*(Td - Tk) + 0*ik;
    Tk = Tk + e;
    ik0 = ik;
    ek0 = ek;

    Ls = Slider_pos(:,j);
    Lsk = Lsk + 0.2*(Ls - Lsk);
    X_dyn = m1.fwd_dynamics(X_dyn(:,end),Lsk,Tk,dt_dyn,Ts);
    X_dyn_vec = [X_dyn_vec,X_dyn];


    m1.plot_cables(X_dyn(1:3,end),Lsk)
    hold on;
    plot(X_dyn_vec(1,:),X_dyn_vec(2,:))
    plot(waypoints(1,:),waypoints(2,:),'*b')
    plot(Xe_current_vec(1,1:j),Xe_current_vec(2,1:j),'--')
    hold off;


end

f5 = figure('color','w');
hold on
plot(0:Ts:tf,Tk_vec(1,:))
plot(0:Ts:tf,Tk_vec(2,:))
plot(0:Ts:tf,Tk_vec(3,:))
plot(0:Ts:tf,Tk_vec(4,:))
hold off
title('Tension')
xlabel('Time')
ylabel('Action')
legend('T1','T2','T3','T4')
