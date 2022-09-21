clc
clear all
close all

inputs.X = [0.43;0.43;0.43;0;0;0];
inputs.desired_x = [0.431;0.431;0.431;0;0;0];
inputs.train = false;
%inputs.prev_X = [0.43;0.43;0.43;0;0;0];
env=CDPRENV(inputs);
validateEnvironment(env);


observationInfo = getObservationInfo(env);
numObservations = observationInfo.Dimension(1);
actionInfo = getActionInfo(env);
numActions = actionInfo.Dimension(1);
rng(0)
%%
load('CDPR_Agent_VelCon_FCirc_CT.mat');
%load('CDPR_Agent.mat');
load('x_traj_cubic_1.mat')
waypoints = x_traj;

% Common model parameters
model.m = 0.1; %kg
model.k0 = [5;5;5;5]; % N/m^2
model.dx = 0.5;
model.dy = 0.5;
model.dz = 0.01;
model1.fail = 0;
m1 = fourPRPR(model,model1);

l = [0;0;0;0];
dt_dyn = 0.001;

% 
Xe_current_vec = [];
Error = [];
Tensions = [];
traj_error = [];
t = [0]; % time array starts with 0 s;
total_steps(1) = 0;
env.X = [waypoints(:,1)-0.01;0;0;0];
a = waypoints(1:2,1) - inputs.X(1:2);

Ts = agent.AgentOptions.SampleTime; % Time taken by one step


try
clear X_dyn X_dyn_vec
catch
end

X_dyn(:,1) = [waypoints(:,1)-0.01;0;0;0];
X_dyn_vec = [];

Tk = [0.1;0.1;0.1;0.1];
Tk_vec(:,1) = Tk;
ek0 = [0;0;0;0];

for i = 1:1:length(waypoints)

    env.X = X_dyn(:,end);
    env.desired_x = [waypoints(:,i);0;0;0];

    % Determine tensions to move to des x
    simOpts = rlSimulationOptions('MaxSteps',100);
    experience = sim(env,agent,simOpts);
    
    X_ = experience.Observation.ActualEndEffectorPositionAndVelocityAndDesiredPosition.Data;
    X_current = squeeze(X_);
    cable_tensions = experience.Action.DesiredCableTensions.Data;
    cable_tensions = squeeze(cable_tensions);
        
    total_steps = env.steps;
    t = [t,t(end)+[2:total_steps]*Ts];

    for j = 1:1:total_steps -1

        Tk_vec = [Tk_vec,Tk];
        Td = cable_tensions(:,j);
        ek = (Td - Tk);
        Tk = Tk + 0.01*(Td - Tk) + 0.01*(ek - ek0);
        ek0 = ek;
        X_dyn = m1.fwd_dynamics(X_dyn(:,end),l,Tk,dt_dyn,Ts);
        X_dyn_vec = [X_dyn_vec,X_dyn];

    end
    
    Xe_current_vec = [Xe_current_vec, X_current(1:4,2:end)];

    Error = [Error, X_current(5:6,2:end)];
    Tensions = [Tensions, cable_tensions];
    
    %Distance error
    len = size(X_current(1:4,2:end),2);
    a = [repmat(a,1,len);zeros(1,len)];
    b = [(X_current(1:2,2:end) - waypoints(1:2,i));zeros(1,len)];
    cr = cross(a,b);
    te = abs(cr(3,:))./norm(a);

    traj_error = [traj_error,te];

    try
    a = waypoints(1:2,i+1) - waypoints(1:2,i);
    catch
    end


    m1.plot_cables(X_dyn(1:3,end),l)
    hold on;
    plot(X_dyn_vec(1,:),X_dyn_vec(2,:))
    plot(waypoints(1,:),waypoints(2,:),'*b')
    plot(Xe_current_vec(1,:),Xe_current_vec(2,:),'--')
    plot(X_current(1,1),X_current(2,1),'*r')
    hold off;

end

tf = agent.AgentOptions.SampleTime*(length(Xe_current_vec));


f4 = figure('color','w');
hold on
plot(0:Ts:tf,Tk_vec(1,:))
plot(0:Ts:tf,Tk_vec(2,:))
plot(0:Ts:tf,Tk_vec(3,:))
plot(0:Ts:tf,Tk_vec(4,:))
hold off
title('Tensions')
xlabel('Time')
ylabel('Action')
legend('T1','T2','T3','T4')

