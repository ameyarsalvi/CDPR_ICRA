classdef trajectory
    properties (Constant)
    end
    properties
        t0 = 0;
        tf = 10;
        dt = 0.01;
    end
    methods
        function [x1,x2] = sine_wave(obj,x0,alpha,A,omega,theta,t)
            %added t
            %alpha = 0.02; %constant
            %A = 0.02; % Amplitude
            %omega = 0.5; % frequency
            
            %t = obj.t0:obj.dt:obj.tf;
            R_theta = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            
            X_des = x0(1:2) + R_theta*[alpha*t; A*sin(omega*t)];

            X_dot_des = R_theta*[alpha*ones(1,length(t)); A*omega*cos(omega*t)];

            phi_traj = atan2(X_dot_des(2,:),X_dot_des(1,:));

            X_ddot_des = R_theta*[zeros(1,length(t)); -A*omega*omega*sin(omega*t)];

            phi_dot_traj = (X_dot_des(1,:).*X_ddot_des(2,:) - X_dot_des(2,:).*X_ddot_des(1,:))./(X_dot_des(1,:).^2);

            x1 = [X_des;phi_traj];
            x2 = [X_dot_des;phi_dot_traj];
        end
        
        function x_traj = sine_wave_waypoints(obj,x0,xf,waypoints)
            % Create waypoints for trajectory
            theta = atan2((xf(2)-x0(2)),(xf(1)-x0(1)));
            R_theta = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            % create sin
            x1 = 0;
            x2 = cos(theta)*(xf(1)-x0(1)) + sin(theta)*(xf(2)-x0(2));
            x = linspace(x1,x2,waypoints);
            y = 0.1*sin((2*pi/x2)*x);
            phi = atan(-cos(x));
            x_traj = [x0 + R_theta*[x;y];theta+phi];
        end
        
        function [x1,x2] = append_traj(obj,x0,x1,x2)
            % x0 is the initial point the new trajectory should start from
            % xf is the initial point of the old trajectory
            xf = x1(:,1);
            d = sqrt((xf(2) - x0(2))^2 + (xf(1) - x0(1))^2);
            % Let avg velocity = 
            v = 0.02; % m/s
            tfa = d/v;
            t = obj.t0:obj.dt:tfa;
            x1_app = [linspace(x0(1),x1(1),length(t));
                      linspace(x0(2),x1(2),length(t));
                      linspace(x0(3),x1(3),length(t))];
            x2_app = [((x1(1) - x0(1))/tfa)*ones(1,length(t));
                      ((x1(2) - x0(2))/tfa)*ones(1,length(t));
                      ((x1(3) - x0(3))/tfa)*ones(1,length(t))];
                  
            x1 = [x1_app,x1];
            x2 = [x2_app,x2];
            obj.tf = obj.tf + tfa;
        end
        
        function x_traj = append_waypoints()
            % x0 is the initial point the new trajectory should start from
            % x_goal is the initial point of the old trajectory
            
        
        end
    end
end
        
