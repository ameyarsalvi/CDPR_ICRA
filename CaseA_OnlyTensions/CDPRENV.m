
classdef CDPRENV < rl.env.MATLABEnvironment
    
    
    %% Properties (set properties' attributes accordingly)
    properties
        % Initialize system state (From outside)
        X % Current state, updated at every step. 
        X_des
        X_init % Initial value of X
        u % desired velocity direction vector
        Reward = 0;
        steps = 1;
        episodes = 1;
        track_error
        velocity_error
        current_joints;
        vf = 0.03; % mm/s
        R0 =0;
        R1;
        R2;
        R3;
        R4;
        R5;
        train;
        Ts = 0.01
        Tf
        P0
        P1
        P2
        nsteps 
        B
        B_dot
        T0
        l0
        kappa
        mom
        K_vec
        M_vec


    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = CDPRENV(inputs)
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([8,1]);
            ObservationInfo.Name = 'Actual End Effector Position, Velocity, Position Error, Velocity Error';
            ObservationInfo.Description = 'x_e y_e x_e_dot y_e_dot error_x_e error_y_e error_x_dot_e error_y_dot_e error_phi_dot_e';
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([4,1],'LowerLimit',[0.1;0.1;0.1;0.1] ,'UpperLimit',[1;1;1;1]); % N
            ActionInfo.Name = 'Desired Cable Tensions';
            ActionInfo.Description = 'T1 T2 T3 T4';
            
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            
            this.train = inputs.train;
            this.nsteps = inputs.nsteps;
            this.Ts = inputs.Ts;
            this.Tf = this.nsteps*this.Ts;

            this.P0 = inputs.P0;
            this.P1 = inputs.P1;
            this.P2 = inputs.P2;
            
            % Initialize property values and pre-compute necessary values
            %updateActionInfo(this);
        end
        %End of initialization function
        
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            
            LoggedSignals = [];
                                  
            joint_state = double(Action);
            this.current_joints = joint_state;
            
            %model.m = 0.1; %kg
            %model.dx = 0.5;
            %model.dy = 0.5;
            %model.dz = 0.01;
            %model1.fail = 0;
            %m1 = fourPRPR(model,model1);

            %if this.steps == 1
                %phi = m1.fwd_kinematics(this.B(:,1),[0;0;0;0]);
                %this.X = [this.B(:,1);phi;this.B_dot(:,1);0]; % Initial state
                %this.X_init = this.X;
                %des_phi = m1.fwd_kinematics(this.B(:,2),[0;0;0;0]);
                %des_phi_dot = (des_phi - phi)/this.Ts;
                %this.X_des = [this.B(:,2);this.B_dot(:,2)]; % Desired state for the current timestep
            %else
                %des_phi = m1.fwd_kinematics(this.B(:,this.steps+1),[0;0;0;0]);
                %des_phi_dot = (this.X(3) - des_phi)/this.Ts;
                %this.X_des = [this.B(:,this.steps+1);this.B_dot(:,this.steps+1)]; % Desired state for the current timestep
                % Current position this.X will be the final state of the robot from
                % the last timestep
            %end

            this.X_des = [this.B(:,this.steps);this.X_init(3);this.B_dot(:,this.steps);0];

            model.m = 0.1; %kg
            model.dx = 0.5;
            model.dy = 0.5;
            model.dz = 0.01;
            model1.fail = 0;
            m1 = fourPRPR(model,model1);
%             m1.plot_cables(this.X(1:6),[0;0;0;0])
%             hold on;
%             plot(this.B(1,:),this.B(2,:),'-')
%             hold off

          
            new_states = traj_tracking(joint_state,this.X,this.Ts);
            this.X = real(new_states(1:6));

            [this.kappa,this.mom] = m1.poseQuality(this.X,[0;0;0;0]);
            this.K_vec = [this.K_vec;this.kappa];
            this.M_vec = [this.M_vec;this.mom];

            dist_error = norm(this.X_des(1:2) - this.X(1:2)); % Distance between the current and desired position
            this.R0 = this.R0 - dist_error;

            % angle between the velocity vectors. 
            %cr = cross([this.X_des(4:5);0],[this.X(4:5);0]);
            %vel_angle_error = atan2(abs(cr(3)),abs(dot(this.X_des(4:5),this.X(4:5)))); 

            this.track_error = this.X_des(1:2) - this.X(1:2); % [2x1] vector
            this.velocity_error = this.X_des(4:5)- this.X(4:5); % [2x1] vector
            
            Observation = [this.X(1:2);this.X(4:5);this.track_error;this.velocity_error]; % 8 by 1 vector

            % Get reward
            Reward = getReward(this);
            
            %Print Values
            fprintf('T = [%0.2f %0.2f %0.2f %0.2f]\t X_e = [%0.2f %0.2f]\t distance_error = %0.2f\t X_init = [%0.2f %0.2f]\t X_des = [%0.2f %0.2f]\t Rewards = [%0.2f %0.2f %0.2f %0.2f] \n' ...
                ,joint_state(1),joint_state(2),joint_state(3),joint_state(4),this.X(1),this.X(2),dist_error,this.X_init(1),this.X_init(2),this.X_des(1),this.X_des(2),this.R0,this.R1,this.R2,this.R3)
             
            % We're charting complete trajectories, no need for termination
            % condition

            % Terminate if robot out of workspace bounds?
            if sum(this.X(1:2) > 0.7) > 0 || sum(this.X(1:2) < 0.1) > 0 
                this.Reward = this.Reward - 1000000;
            end

            if this.steps == this.nsteps
                IsDone = true;
            else
                IsDone = false;
            end

            this.steps = this.steps + 1;
                
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
           
            if this.train == true
                %%% For Training
                % Generate random Bezier curves within workspace
                this.P0 = [0.4*rand(1) + 0.23;0.4*rand(1) + 0.23];
                this.P1 = [0.4*rand(1) + 0.23;0.4*rand(1) + 0.23];
                this.P2 = [0.4*rand(1) + 0.23;0.4*rand(1) + 0.23];
                
                %%%%%%%%%%%%%%%%%
            else
                %%% For Inference (user defined)
            
                this.P0 = this.P0;
                this.P1 = this.P1;
                this.P2 = this.P2;
                %%%%%%%%%%%%%%%%% 
            end

            [this.B,this.B_dot] = bezier_curve(this.P0,this.P1,this.P2,this.Ts,this.Tf);

%             plot([this.P0(1),this.P1(1),this.P2(1)],[this.P0(2),this.P1(2),this.P2(2)],'*')
%             hold on;
%             plot(this.B(1,:),this.B(2,:),'-')

            %model.m = 0.1; %kg
           % model.dx = 0.5;
            %model.dy = 0.5;
            %model.dz = 0.01;
            %model1.fail = 0;
            %m1 = fourPRPR(model,model1);

            phi = 0.3; %m1.fwd_kinematics(this.B(:,1),[0;0;0;0]);
            this.X = [this.B(:,1);phi;this.B_dot(:,1);0]; % Initial state
            this.X_init = this.X;
            %des_phi = 0.3; %m1.fwd_kinematics(this.B(:,2),[0;0;0;0]);
            %des_phi_dot = (des_phi - phi)/this.Ts;
            this.X_des = [this.B(:,2);phi;this.B_dot(:,2);0]; % Desired state for the current timestep 

            %[this.l0,fval] = m1.minimize_objective(this.X(1:3),[0;0.1;0.3;0.1]);

            %m1.plot_cables(this.X(1:3),this.l0)

%             Jw = m1.structureMatrix(this.X,[0;0;0;0]);
%     
%             % Desired tension in the new position
%             f_o = [0;0;0];
% 
%             tau_min = [0.1;0.1;0.1;0.1];
%             tau_pos_init = [0.1;0.1;0.1;0.1];
%             options = optimset('Display', 'off','LargeScale','on','Algorithm','interior-point');
%             this.T0 = fmincon(@m1.minimize_tension, tau_pos_init ,[],[], Jw,-f_o, tau_min,[],[],options);
            
            this.track_error = this.X_des(1:2) - this.X_init(1:2);
            this.velocity_error =  this.X_des(4:5) - this.X_init(4:5);
                        
            Observation = [this.X_init([1,2,4,5]);this.track_error;this.velocity_error]; % 8 by 1 vector

            InitialObservation = Observation;
            
            this.R0 = 0;
            this.Reward = 0;
            this.steps = 1;
            this.episodes = this.episodes +1;
                
        end
        %End of Reset function
        
    end
    %End of Necessary Methods
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Reward function
        function Reward = getReward(this)


            % let the desired velocity be: 0.03 m/s
            
            % 1: with only position
            % 2: with position and velocity
            % 3: Both above with changing the noise
            %Each for 1000 eps
            
%             if this.episodes >1
%                 weight = [0 200 2 0]; %[Tracking Velocity]
%             else
%                 weight = [0 2000 10 100]; %[Tracking Velocity]
%             end

            weight =[2000 20 2];

            cable_tensions = this.current_joints(1:4); 
            

            this.R1 = -norm(this.track_error); 
            this.R2 = -norm(this.velocity_error);
            this.R3 = -1*(cable_tensions - [0.1;0.1;0.1;0.1])'*(cable_tensions - [0.1;0.1;0.1;0.1]); % minimize tension
            % kappa tends from 0 to 1 where 1 is favourable and generally
            % hard to get, and zero is extremely unfavorable.
            %this.R6 = -abs(this.steps - nstep_des); % penalize taking more steps than required

            rew = [this.R1;this.R2;this.R3];
            
            reward = weight*rew;           
            
            this.Reward = reward;
            Reward = this.Reward;
            
        end   
    end
    
    methods (Access = protected)
        end
    end

