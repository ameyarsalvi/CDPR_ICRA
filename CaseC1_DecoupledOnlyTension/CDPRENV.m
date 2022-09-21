
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
        kappa
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
        Ts = 0.01
        Tf
        P0
        P1
        P2
        nsteps 
        B
        B_dot
        K_vec
        X_vec
        M_vec
        slideragent
        l0
        policy
        l_vec
        rew_wt;
        track_error_prev=[0;0];
        ls_dot0;
        mom;
        T0;
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
            ObservationInfo = rlNumericSpec([12,1]);
            ObservationInfo.Name = 'Current End Effector Position, Velocity, ls01 ls02 ls03 ls04';
            ObservationInfo.Description = 'x_e y_e x_e_dot y_e_dot x_error y_error ls01 ls02 ls03 ls04';
            
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
            this.rew_wt = inputs.reward_weights;
                     
            %this.policy = coder.loadRLPolicy("agentData.mat");

            % Initialize property values and pre-compute necessary values
            %updateActionInfo(this);
        end
        %End of initialization function
        
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            
            LoggedSignals = [];
                                  
            joint_state = double(Action);
            this.current_joints = joint_state;
    
            phi = this.X_init(3);
            phi_dot = 0;

            %if this.steps == 1
            %    model.m = 0.1; %kg
            %    model.dx = 0.5;
            %    model.dy = 0.5;
            %    model.dz = 0.01;
            %    model1.fail = 0;
            %    m1 = fourPRPR(model,model1);
            %    [this.l0,fval] = m1.minimize_objective(this.X_des,[0;0;0;0]);
            %end

            this.X_des= [this.B(:,this.steps+1);phi;this.B_dot(:,this.steps+1);phi_dot];


            input = [this.X_des(1:3);this.kappa;this.mom;this.l0];

            ls_star = evaluatePolicy(input);

            [new_states,this.l0,this.T0,this.kappa,this.mom] = traj_tracking(joint_state,this.X,this.X_des,this.l0,this.Ts,this.kappa,this.mom,ls_star,this.T0);
            
            this.X = real(new_states);
            this.X_vec = [this.X_vec,this.X];
            this.K_vec = [this.K_vec;this.kappa];
            this.M_vec = [this.M_vec;this.mom];

            dist_error = norm(this.X_des(1:2) - this.X(1:2)); % Distance between the current and desired position
            this.R1 = this.R1 - dist_error;
            mono_dec = norm(this.track_error) - norm(this.track_error_prev);
            if mono_dec >0
                this.R2 = -10;
            else
                this.R2 = 10;
            end
            
            this.track_error = this.X_des(1:3) - this.X(1:3); % [2x1] vector
            this.velocity_error = this.X_des(4:6)- this.X(4:6); % [2x1] vector

            Observation = [this.X(1:3);this.X(4:6);this.track_error(1:2);this.l0]; % 10 by 1 vector

            % Get reward
            Reward = getReward(this);

            this.track_error_prev = this.track_error;
            
            %Print Values
            %fprintf('T = [%0.2f %0.2f %0.2f %0.2f]\t Ls = [%0.2f %0.2f %0.2f %0.2f]\t X_e = [%0.2f %0.2f %0.2f]\t distance_error = %0.2f\t X_des = [%0.2f %0.2f %0.2f]\t Rewards = [%0.2f %0.2f %0.2f %0.2f] \n' ...
            %    ,joint_state(1),joint_state(2),joint_state(3),joint_state(4),this.l0(1),this.l0(2),this.l0(3),this.l0(4),this.X(1),this.X(2),this.X(3),dist_error,this.X_des(1),this.X_des(2),this.X_des(3),this.R0,this.R1,this.R2,this.R3)
             
            % We're charting complete trajectories, no need for termination
            % condition

            % Penalize if robot out of workspace bounds?

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
%                 this.P0 = [(0.1*randn(1) + 0.33); (0.1*randn(1) + 0.33)];
%                 this.P1 = [(0.2+0.1*randn(1) + 0.33) ; (0.1*randn(1) + 0.33)];
%                 this.P2 = [0.1*randn(1) + 0.33;(0.2+0.1*randn(1) + 0.33)];

                this.P0 = [(0.4*rand(1) + 0.23); (0.4*rand(1) + 0.23)];
                this.P1 = [(0.4*rand(1) + 0.23) ; (0.4*rand(1) + 0.23)];
                this.P2 = [(0.4*rand(1) + 0.23);(0.4*rand(1) + 0.23)];

                %phi =  pi/4 ;

                %%%%%%%%%%%%%%%%%
            else
                %%% For Inference (user defined)
            
                this.P0 = this.P0;
                this.P1 = this.P1;
                this.P2 = this.P2;
                %%%%%%%%%%%%%%%%% 

                
            end

            [this.B,this.B_dot] = bezier_curve(this.P0,this.P1,this.P2,this.Ts,this.Tf);
            phi = pi/4;


            this.X = [this.B(1:2,1); phi; this.B_dot(1:2,1); 0];
            this.X_init = this.X;
            this.X_des = this.X;
            model.m = 0.1; %kg
            model.dx = 0.5;
            model.dy = 0.5;
            model.dz = 0.01;
            model1.fail = 0;
            m1 = fourPRPR(model,model1);
            [this.l0,fval] = m1.minimize_objective(this.X_des,[0;0;0;0]);

            Observation = [this.X(1:3); this.X(4:6); 0;0; this.l0]; % 12 by 1 vector

            InitialObservation = Observation;
            
            this.track_error_prev =[0;0];
            this.R1 = 0;
            this.R2 =0;
            %this.prev_slider = 0.570*rand(4,1);
            this.Reward = 0;
            this.steps = 1;
            this.episodes = this.episodes +1;
            this.kappa =1;
            this.mom =1;

            this.K_vec = [];
            this.M_vec =[];
            this.ls_dot0 =[0;0;0;0];
            this.T0 =[0.1;0.1;0.1;0.1];
                
        end
        %End of Reset function
        
    end
    %End of Necessary Methods
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Reward function
        function Reward = getReward(this)

            %R0 = tracking error norm
            %R1 = Integral of tracking error norm
            %R2 = Monotonic error change
            %R3 = velocity error norm
            %R3 = Minimize Cable Tensions
            %R4 = Maximize Kappa
            
            
            
            %weight = [1 1 1 1 1 1].*[this.rew_wt];
            weight = [2000000 0 0 200 2 0];
            

            cable_tensions = this.current_joints(1:4); 
            
            this.R0 = -norm(this.track_error(1:2)); 
            this.R1 = -norm(this.track_error(3)); 
            this.R2 = -norm(this.velocity_error(3));
            this.R3 = -norm(this.velocity_error(1:2));
            this.R4 = -1*(cable_tensions - [0.1;0.1;0.1;0.1])'*(cable_tensions - [0.1;0.1;0.1;0.1]); % minimize tension
            %this.R5 = -(1-this.kappa); % -(1/kappa -1) to penalize it harder? 
            this.R5 =0;

            rew = [this.R0;this.R1;this.R2;this.R3;this.R4;this.R5];
            %keyboard
            reward = weight*rew;           
            
            this.Reward = reward;
            Reward = this.Reward;
            
        end   
    end
    
    methods (Access = protected)
        end
    end

