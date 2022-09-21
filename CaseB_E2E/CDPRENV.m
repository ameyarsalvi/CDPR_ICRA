
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
        track_error_prev = zeros(2,1)
        velocity_error
        kappa
        mom
        current_joints;
        vf = 0.03; % mm/s
        R0 = 0;
        R1;
        R2;
        R3;
        R4;
        R5;
        R6;
        train;
        Ts = 0.05
        Tf
        P0
        P1
        P2
        nsteps 
        B
        B_dot
        T0
        l0
        prev_slider = [0;0;0;0]
        ls_dot0


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
            ObservationInfo = rlNumericSpec([20,1]);
            ObservationInfo.Name = 'Actual End Effector Position, Velocity, Position Error, Velocity Error, Sensitivity, T1 T2 T3 T4 Ls1 Ls2 Ls3 Ls4';
            ObservationInfo.Description = 'x_e y_e phi_e x_e_dot y_e_dot phi_e_dot error_x_e error_y_e error_x_dot_e error_y_dot_e  kappa mom T1 T2 T3 T4 Ls1 Ls2 Ls3 Ls4';
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([8,1],'LowerLimit',[0.1;0.1;0.1;0.1;0;0;0;0] ,'UpperLimit',[1;1;1;1;0.570;0.570;0.570;0.570]); % N
            ActionInfo.Name = 'Desired Cable Tensions, Desired slider positions';
            ActionInfo.Description = 'T1 T2 T3 T4 Ls1 Ls2 Ls3 Ls4';
            
            
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
            

            this.X_des = [this.B(:,this.steps);this.X_init(3);this.B_dot(:,this.steps);this.X_init(6)]; % Desired state for the current timestep

            

            
            [new_states,this.T0,this.l0,this.kappa,this.mom] = traj_tracking(joint_state,this.X,this.T0,this.l0,this.Ts);
            
            this.X = real(new_states(1:6));


            dist_error = norm(this.X_des(1:2) - this.X(1:2)); % Distance between the current and desired position
            this.R1 = this.R1 - dist_error;
            mono_dec = norm(this.track_error) - norm(this.track_error_prev);
            if mono_dec >0
                this.R2 = -10;
            else
                this.R2 = 10;
            end

           

            this.track_error = this.X_des(1:2) - this.X(1:2); % [2x1] vector
            this.velocity_error = this.X_des(4:5)- this.X(4:5); % [2x1] vector

            Observation = [this.X;this.track_error;this.velocity_error;this.kappa;this.mom;this.T0;this.l0]; % 25 by 1 vector

            % Get reward
            Reward = getReward(this);
            
            %Print Values
            fprintf('T = [%0.2f %0.2f %0.2f %0.2f]\t Ls = [%0.2f %0.2f %0.2f %0.2f]\t X_e = [%0.2f %0.2f]\t distance_error = %0.2f\t X_init = [%0.2f %0.2f]\t X_des = [%0.2f %0.2f]\t Rewards = [%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f] \n' ...
                ,joint_state(1),joint_state(2),joint_state(3),joint_state(4),joint_state(5),joint_state(6),joint_state(7),joint_state(8),this.X(1),this.X(2),dist_error,this.X_init(1),this.X_init(2),this.X_des(1),this.X_des(2),this.R0,this.R1,this.R2,this.R3,this.R4,this.R5(1))
             



            if this.steps == this.nsteps %(sum(this.X(1:2) > 0.7) > 0 || sum(this.X(1:2) < 0.1) > 0) == 1 
                IsDone = true;
            else
                IsDone = false;
            end

            this.steps = this.steps + 1;
            this.prev_slider = this.l0;
                
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
           
            if this.train == true
                %%% For Training
                % Generate random Bezier curves within workspace
                this.P0 = [(0.1*randn(1) + 0.33); (0.1*randn(1) + 0.33)];
                this.P1 = [(0.2+0.1*randn(1) + 0.33) ; (0.1*randn(1) + 0.33)];
                this.P2 = [0.1*randn(1) + 0.33;(0.2+0.1*randn(1) + 0.33)];
                
                %%%%%%%%%%%%%%%%%
            else
                %%% For Inference (user defined)
            
                this.P0 = this.P0;
                this.P1 = this.P1;
                this.P2 = this.P2;
                %%%%%%%%%%%%%%%%% 
            end

            [this.B,this.B_dot] = bezier_curve(this.P0,this.P1,this.P2,this.Ts,this.Tf);

            
            model.m = 0.1; %kg
            model.dx = 0.5;
            model.dy = 0.5;
            model.dz = 0.01;
            model1.fail = 0;
            m1 = fourPRPR(model,model1);


            phi = pi/4;% + ((-pi/3)*rand(1))+(pi/6);
            this.X = [this.B(:,1);phi;this.B_dot(:,1);0]; % Initial state
            this.X_init = this.X;
            this.X_des = [this.B(:,2);this.X_init(3);this.B_dot(:,2);0]; % Desired state for the current timestep 
           
            this.track_error = this.X_des(1:2) - this.X_init(1:2);
            this.velocity_error =  this.X_des(3:4) - this.X_init(4:5);
            this.kappa = 1; % Since we don't know what slider positions are we have to assume a value for kappa during reset
            this.mom =1;  

            [this.l0,fval] = m1.minimize_objective(this.X,[0;0;0;0]);

            Jw = m1.structureMatrix(this.X,this.l0);
            z = abs(null(Jw));
            this.T0 = z*(0.1/min(z));

     
            Observation = [this.X_init;this.track_error;this.velocity_error;this.kappa;this.mom;this.T0;this.l0]; % 25 by 1 vector

            InitialObservation = Observation;
            

            this.ls_dot0 =[0;0;0;0];
            this.R0 = 0;
            this.R1 =0;
            this.R2 =0;
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

            sat = max(0.001,norm(this.track_error));
            w1 = 1*((1/sat));
                weight = [2000000 0 0 10000 10 100 100];

            cable_tensions = this.current_joints(1:4); 
            
            this.R0 = -norm(this.track_error); 
            this.R3 = -norm(this.velocity_error);
            this.R4 = -1*(cable_tensions - [0.1;0.1;0.1;0.1])'*(cable_tensions - [0.1;0.1;0.1;0.1]); % minimize tension
            this.R5 = -(1-this.kappa); % -(1/kappa -1) to penalize it harder? 
            this.R6 = -(1-this.mom);
   

            rew = [this.R0;this.R1;this.R2;this.R3;this.R4;this.R5;this.R6];
            
            reward = weight*rew;  
            
            
            this.Reward = reward;
            Reward = this.Reward;
            
        end   
    end
    
    methods (Access = protected)
        end
    end

