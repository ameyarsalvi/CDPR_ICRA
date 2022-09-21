function new_states = traj_tracking(joint_states,X_k,Ts)

    
    % Simulation properties
    sim.dt_dyn = 0.001;
    sim.tf = Ts;
    sim.t = 0:sim.dt_dyn:sim.tf;

    % Common model parameters
    model.m = 0.1; %kg
    model.dx = 0.5;
    model.dy = 0.5;
    model.dz = 0.01;
    model1.fail = 0;
    m1 = fourPRPR(model,model1);
    
    cable_tensions = joint_states(1:4);
    l = [0;0;0;0]; 
    
    % Add model noise to the plant at some point
    
    X_dyn0 = X_k;
    
    X_dyn = m1.fwd_dynamics(X_dyn0,l,cable_tensions,sim.dt_dyn,sim.tf);

    new_states = [X_dyn(:,end)];
        
end