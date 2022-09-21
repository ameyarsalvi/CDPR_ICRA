close all
clear

% Dynamic model parameters
model.m = 0.1; %kg
model.dx = 0.5;
model.dy = 0.5;
model.dz = 0.01;
model1.fail = 0;
m1 = fourPRPR(model,model1);

sim.dt_dyn = 0.001;
sim.Ts = 0.05;
sim.tf = 5;

%% Randomized initial position, constant actions
X_dyn = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2;-0.4*rand(1) + 0.2;0;0;0];
X_dyn_vec = [];
kappa_vec = [];
l_des = [0.1;0.1;0.1;0.1];
l = [0;0;0;0];
T_des = [2;4;2;4];
T = [0.1;0.1;0.1;0.1];
%theta = [0.1;0.11;0.1;0.11];
for i = 0:sim.Ts:sim.tf

    %l_p_x = m1.prismatic_length(X_dyn(1:3,end),l);
    %cable_tensions = 10*eye(4)*(l_p_x - theta);

    [X_dyn,T,l] = m1.fwd_dynamics(X_dyn(:,end),l(:,end),l_des,T(:,end),T_des,sim.dt_dyn,sim.Ts);
    X_dyn_vec = [X_dyn_vec,X_dyn];
    kappa = m1.sensitivity_kappa(X_dyn,l);
    kappa_vec = [kappa_vec,kappa];

    m1.plot_cables(X_dyn(:,end),l(:,end))
    hold on;
    plot(X_dyn_vec(1,:),X_dyn_vec(2,:),'-')
    hold off;
end

% m1.plot_cables(X_dyn_vec(:,end),l)
% hold on;
% plot(X_dyn_vec(1,:),X_dyn_vec(2,:),'-')

figure(1)
plot(X_dyn_vec(1:3,:)')

figure(2)
plot(kappa_vec')

%% Randomized initial position and random actions

vf = 0.03;
X0 = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2;-0.4*rand(1) + 0.2];    %[mm;mm;rad]        

theta_rand = 2*pi*rand(1); % Fixed distance - Desired point on perimeter
des_X0 = X0 + [0.0143;0.0143;0.05].*[cos(theta_rand);sin(theta_rand);-2*rand(1) + 1]; %mm

dist = norm(des_X0(1:2) - X0(1:2)); %mm
u = [(des_X0(1) - X0(1))/dist; (des_X0(2) - X0(2))/dist]; % desired unit velocity vector
theta = atan2(u(2),u(1));
phi = theta + -1*rand(1) + 0.5;
randomized_velocity_vec = [cos(phi);sin(phi)]; % initial randomized unit velocity vector

X = [X0;vf*randomized_velocity_vec;0];
X_des = [des_X0;vf*u;0];

% Randomized cable tensions and slider positions
l_des = 0.570*[rand(1);rand(1);rand(1);rand(1)];
T_des = [0.9*rand(1)+0.1;0.9*rand(1)+0.1;0.9*rand(1)+0.1;0.9*rand(1)+0.1];

l = [0;0;0;0];
T = [0.1;0.1;0.1;0.1];

try
    clear X_dyn
catch
end

X_dyn = X;
X_dyn_vec = [];
T_des_vec = [];
l_des_vec = [];
T_vec = [];
l_vec = [];
kappa_vec = [];

for i = 0:sim.Ts:sim.tf

    %l_p_x = m1.prismatic_length(X_dyn(1:3,end),l);
    %cable_tensions = 10*eye(4)*(l_p_x - theta);
    %T_des = [0.9*rand(1)+0.1;0.9*rand(1)+0.1;0.9*rand(1)+0.1;0.9*rand(1)+0.1];
    %l_des = 0.570*[rand(1);rand(1);rand(1);rand(1)];

    [X_dyn,T,l] = m1.fwd_dynamics(X_dyn(:,end),l(:,end),l_des,T(:,end),T_des,sim.dt_dyn,sim.Ts);
    X_dyn_vec = [X_dyn_vec,X_dyn];
    T_vec = [T_vec,T];
    T_des_vec = [T_des_vec,T_des.*ones(4,size(T,2))];
    l_des_vec = [l_des_vec,l_des.*ones(4,size(T,2))];

    l_vec = [l_vec,l];
    kappa = m1.sensitivity_kappa(X_dyn,l);
    kappa_vec = [kappa_vec,kappa];
    
%     m1.plot_cables(X_dyn(:,end),l(:,end))
%     hold on;
%     plot(X_dyn_vec(1,:),X_dyn_vec(2,:),'-')
%     hold off;
end

figure(1)
m1.plot_cables(X_dyn_vec(:,end),l(:,end))
hold on;
plot(X_dyn_vec(1,:),X_dyn_vec(2,:),'-')

figure(2)
plot(X_dyn_vec(1:3,:)')

figure(3)
plot(kappa_vec')

figure(3)
plot(T_vec(1,:)')
hold on;
plot(T_des_vec(1,:)')
hold off;

figure(4)
plot(l_vec(1,:)')
hold on;
plot(l_des_vec(1,:)')
hold off;

%% Random actions, but record desired bezier positions

try
    clear X_dyn
catch
end

X_des_vec = [];
X_dyn_vec = [];
kappa_vec = [];

% Randomized cable tensions and slider positions
l = 0.570*[rand(1);rand(1);rand(1);rand(1)];
cable_tensions = [0.9*rand(1)+0.1;0.9*rand(1)+0.1;0.9*rand(1)+0.1;0.9*rand(1)+0.1];
joint_state = [cable_tensions;l];

nsteps = 500;
Tf = nsteps*Ts;

dt = sim.Ts/Tf; % Normalized timestep for Bezier curves

P0 = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2];
P1 = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2];
P2 = [0.4*rand(1) + 0.2;0.4*rand(1) + 0.2];

t = [0,dt];  % inital time and current time in the episode. (normalized)
B = (1-t).*((1-t).*P0 + t.*P1) + t.*((1-t).*P1 + t.*P2); % desired position
B_dot = t.*(2*P0 - 4*P1 + 2*P2) + (-2*P0 + 2*P1); % desired velocity vector (normalized)
B_dot = (B_dot*dt)/sim.Ts;

phi = m1.fwd_kinematics(B(:,1),joint_state(5:8));
X = [B(:,1);phi;B_dot(:,1);0]; % Initial state

for steps = 1:1:nsteps

    % Initial and final position on B(t)
    if steps == 1
        t = [0,dt];  % inital time and current time in the episode. (normalized)
        B = (1-t).*((1-t).*P0 + t.*P1) + t.*((1-t).*P1 + t.*P2); % desired position
        B_dot = t.*(2*P0 - 4*P1 + 2*P2) + (-2*P0 + 2*P1); % desired velocity vector (normalized)
        B_dot = (B_dot*dt)/sim.Ts;

        phi = m1.fwd_kinematics(B(:,1),joint_state(5:8));
        X = [B(:,1);phi;B_dot(:,1);0]; % Initial state
        X_init = X;
        des_phi = atan2(B_dot(2,2),B_dot(1,2));
        des_phi_dot = (X(3) - des_phi)/sim.Ts;
        X_des = [B(:,2);des_phi;B_dot(:,2);des_phi_dot]; % Desired state for the current timestep
    else
        t = steps*dt;  % current time in the episode. (normalized)
        B = (1-t).*((1-t).*P0 + t.*P1) + t.*((1-t).*P1 + t.*P2); % desired position
        B_dot = t*(2*P0 - 4*P1 + 2*P2) + (-2*P0 + 2*P1); % desired velocity vector (normalized)
        B_dot = (B_dot*dt)/sim.Ts;

        des_phi = atan2(B_dot(2),B_dot(1));
        des_phi_dot = (X(3) - des_phi)/sim.Ts;
        X_des = [B;des_phi;B_dot;des_phi_dot]; % Desired state for the current timestep
        % Current position X will be the final state of the robot from
        % the last timestep
    end

    X = m1.fwd_dynamics(X(:,end),l,cable_tensions,sim.dt_dyn,sim.Ts);
    X_des_vec = [X_des_vec,X_des];
    X_dyn_vec = [X_dyn_vec,X];
    kappa = m1.sensitivity_kappa(X,l);
    kappa_vec = [kappa_vec,kappa];

%     m1.plot_cables(X_dyn_vec(:,end),l)
%     hold on;
%     plot(X_dyn_vec(1,:),X_dyn_vec(2,:),'-')
%     hold off;

end

figure(1)
m1.plot_cables(X(:,end),l)
hold on;
plot(X(1,:),X(2,:),'-')
hold off

figure(2)
plot(X_des_vec(1,:),X_des_vec(2,:),'*')

figure(3)
plot(X_dyn_vec(1:3,:)')
hold on;
%plot(kappa_vec')




