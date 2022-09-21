function [new_states,l0] = traj_tracking(joint_states,X_dyn0,X_des,l0,Ts)

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
    
    T_des = joint_states(1:4);
    
    % Add model noise to the plant at some point
    %X_dyn0 Current state
    %X_des Desired state

    % Determine slider position based on desired state
    [ls_star,fval] = m1.minimize_objective(X_des,l0); 
    % if fval is negative, the system is infeasible

    [X_dyn,l1] = m1.fwd_dynamics(X_dyn0,l0,ls_star,T_des,sim.dt_dyn,sim.tf);
    
    new_states = X_dyn(:,end);
    l0 = l1(:,end);
        
end