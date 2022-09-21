function [new_states,T0,l0,kappa,mom] = traj_tracking(joint_states,X_k,T0,ls0,Ts)

    
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
    l_des = joint_states(5:end); 
    
    % Add model noise to the plant at some point
    
    X_dyn0 = X_k;
    

    [X_dyn,l1] = m1.fwd_dynamics(X_dyn0,ls0,l_des,T_des,sim.dt_dyn,sim.tf);
    %(obj,X_dyn,ls0,ls_dot0,ls_ref,T_des,dt,tf)
    
    %kappa = m1.sensitivity_kappa(X_dyn,l1);
    [kappa,mom] = m1.poseQuality(X_dyn,l1);

    new_states = [X_dyn(:,end)];
    T0 = T_des(:,end);
    l0 = l1(:,end);
   
        
end