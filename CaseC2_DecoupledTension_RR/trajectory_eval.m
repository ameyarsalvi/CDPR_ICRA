
%% Evaluating the Agent

clc
clear all
close all

inputs.train = false;
inputs.Xe = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2;0.8*rand(1)];
nsteps = 10;
inputs.nsteps = nsteps;
inputs.Ts = 0.05;
env=CDPRENV(inputs);
validateEnvironment(env);

observationInfo = getObservationInfo(env);
numObservations = observationInfo.Dimension(1);
actionInfo = getActionInfo(env);
numActions = actionInfo.Dimension(1);
rng(0)
%%
load('onlyslidersagent.mat');

%% Select a set of random waypoints in the workspace and draw bezier curves between them
waypoints = 0.4*rand(2,9) + 0.2*ones(2,9);
nw = size(waypoints,2);

%[B,B_dot] = bezier_curve(waypoints,env.Ts,env.Tf);

%%

Kappa = [];
Slider_pos = [];
Xe_traj = [];
t = [0];


model.m = 0.1; %kg
model.dx = 0.5;
model.dy = 0.5;
model.dz = 0.01;
model1.fail = 0;
m1 = fourPRPR(model,model1);

l0 = [0;0;0;0];
phi = 0.8;
for i = 3:2:nw
    P0 = waypoints(:,i-2);
    P1 = waypoints(:,i-1);
    P2 = waypoints(:,i);

    [B,B_dot] = bezier_curve(P0,P1,P2,env.Ts,5);



    for j = 1:1:size(B,2)
        phi = atan2(B_dot(2,j),B_dot(1,j));
        
        if phi < -1.54
            phi = phi + pi;
        elseif phi > 3.09 
            phi = phi - pi;
        elseif phi < -0.05 && phi >= -1.54
            phi = -0.05;
        elseif phi > 1.6 && phi <= 3.09
            phi = 1.6;
        end
        env.Xe = [B(:,j);phi];

        simOpts = rlSimulationOptions('MaxSteps',nsteps);
        experience = sim(env,agent,simOpts);

        new_states = experience.Observation.X_desSensitivity.Data;
        new_states_ = squeeze(new_states);
        X = new_states_(1:3,:);
        %kappa = new_states_(4,:);
        kappa = env.K_vec;
        joint_states = experience.Action.DesiredSliderPositions.Data;
        joint_states = squeeze(joint_states);

        slider_position = joint_states(1:4,:);
        
        total_steps = env.steps;
        t = [t,t(end)+[2:total_steps]*inputs.Ts];
        
        Slider_pos = [Slider_pos,slider_position];
        Kappa = [Kappa;kappa];
        Xe_traj = [Xe_traj,B];
        
        l_des = slider_position(:,end);
        
        %for steps = 1:1:size(slider_position,2)
        
        Delta_l = (l_des - l0);
        k = abs(6*Delta_l); % 0.3;
        l0 = l0 + Delta_l.*(ones(4,1)-exp(-env.Ts./k));
    
        phi = m1.fwd_kinematics(B(:,j),l0);
        Xe = [B(:,j);phi];
        m1.plot_cables(Xe,l0);
        hold on;
        plot(B(1,:),B(2,:),'-');
        hold off;
        %end

    end




end


%%

f1 = figure('color','w');
plot(Xe_traj(1,:),Xe_traj(2,:),'-')
title('Trajectory')
xlabel('X')
ylabel('Y')

f3 = figure('color','w');
plot(Kappa)
title('Sensitivity')
xlabel('Time')
ylabel('Sensitivity')
%set(f3,'Position',[2500 10 540 500])


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
%set(f5,'Position',[3000 10 540 500])



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
