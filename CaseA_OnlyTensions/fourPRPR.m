
classdef fourPRPR < PRPRcommon
   properties
        fail
   end
   methods
       function obj = fourPRPR(model,fail)
           obj = obj@PRPRcommon(model);
           obj.fail = fail.fail;
       end
          
      function plot_cables(obj, X, l)      

        [EA_P,EB_P,EC_P,ED_P] = obj.platform_coordinates(X);
        [A3,B3,C3,D3] = obj.slider_positions(l); 
        
        figure(1);
        plot([obj.A1(1),obj.A2(1),obj.B1(1),obj.B2(1),obj.C1(1),obj.C2(1),obj.D1(1),obj.D2(1),obj.A1(1)],[obj.A1(2),obj.A2(2),obj.B1(2),obj.B2(2),obj.C1(2),obj.C2(2),obj.D1(2),obj.D2(2),obj.A1(2)],'b','LineWidth',1);
        hold on;
        
        % Plot ee
        plot([EA_P(1),EB_P(1),EC_P(1),ED_P(1),EA_P(1)],[EA_P(2),EB_P(2),EC_P(2),ED_P(2),EA_P(2)],'k','LineWidth',1);

        % Plot cables
        plot([A3(1),EA_P(1)],[A3(2),EA_P(2)],'-.k','LineWidth',1.5);
        plot([B3(1),EB_P(1)],[B3(2),EB_P(2)],'-.k','LineWidth',1.5);
        plot([C3(1),EC_P(1)],[C3(2),EC_P(2)],'-.k','LineWidth',1.5);
        plot([D3(1),ED_P(1)],[D3(2),ED_P(2)],'-.k','LineWidth',1.5);

        axis equal
        hold off
      end

     function [kappa,mom] = poseQuality(obj,X_vec,l)
        Jw = obj.structureMatrix(X_vec(1:3,end),l(:,end));
        z = null(Jw);
        if min(z) > 0
            kappa = min(z)/max(z);
        elseif max(z) < 0
            kappa = max(z)/min(z);
        else
            %kappa = min(z)/max(z);
            kappa = 0;
        end

        %Jw = obj.structureMatrix(X_vec(1:3,end),l(:,end));
        s = svd(Jw(1:2,:));
        mom = min(s)/max(s);

        if kappa < 0
            %mom = -mom;
            mom = 0;
        end
 
      end 



      function [theta,l_new,tau_pos] = joint_state(obj,X,X_dot,theta0,l0,dt,tau_pos_init)

         
        % New desired position of the end-effector
        X_new = X + X_dot*dt;
        
        % Convert spatial velocity to body velocity
        phi_e = X(3);
        pRo = [cos(phi_e) -sin(phi_e); sin(phi_e) cos(phi_e)]; %rot_z
        p = X(1:2);      
        Adgpo = [pRo [p(2); -p(1)]; 0 0 1]; 
        X_dot_b = Adgpo\X_dot;
        
        
        % Desired slider position for optimal manipulability ellipsoid
        l_des = obj.minimize_objective(X_new,l0,theta0);
        % Realistic slider input that can be provided 
        l_dot = (l_des-l0)/dt;
        
        l_dot(l_dot>0.05) = 0.05;
        l_dot(l_dot<-0.05) = -0.05;

        l_new = l0 + l_dot*dt;
          
        % New desired prismatic length
        Jw = obj.structureMatrix(X,l_new);
        l_p_x = obj.prismatic_length(X,l_new);
        
        l_p_x_dot = -Jw'*X_dot;
        l_p_x_new = l_p_x + l_p_x_dot*dt;
        
        Jw_new = obj.structureMatrix(X_new,l_new);

        % Desired tension in the new position
        f_o = [0;0;0];%0.1*[X_dot_b(1:2);0]; % desired wrench on ee is applied in the direction of the desired velocity.
        
        tau_min = [0.5;0.5;0.5;0.5];
        options = optimset('Display', 'off','LargeScale','on','Algorithm','interior-point');
        tau_pos = fmincon(@obj.minimize_tension, tau_pos_init ,[],[], Jw_new,-f_o, tau_min,[],[],options);
               
        k = obj.k0;

        theta = (k.*l_p_x_new)./(tau_pos + k);

        theta_dot = (theta - theta0)/dt;

        theta_dot(theta_dot>0.05) = 0.05;
        theta_dot(theta_dot<-0.05) = -0.05;

        theta = theta0 + theta_dot*dt;

      end

      function tau = minimize_tension(obj,x)
          tau=norm(x,2);
      end
                        
      

      function [l1,fval] = minimize_objective(obj, x1i,l_init)

      % Maximize the manipulability ellipsoid.
        lb = [0;0;0;0];
        ub = [obj.base_span; obj.base_span; obj.base_span; obj.base_span];

        Aineq = [];
        bineq = [];
        Aeq = [];
        Beq = [];

        fnonlcon = @(l)ensure_wrench_closure(l);

        [l1,fval] = fmincon(@(l)objective_fn(l),l_init,Aineq,bineq,Aeq,Beq,lb,ub,fnonlcon);

        function objf = objective_fn(l)
            Jw = obj.structureMatrix(x1i,l);
            z = null(Jw);
            if min(z) > 0
                objf = -min(z)/max(z);
            elseif max(z) < 0
                objf = -max(z)/min(z);
            else
                objf = 0;
            end
        end

        function [c,ceq] = ensure_wrench_closure(l)
            % Ensure wrench closure
            Jw = obj.structureMatrix(x1i,l);
            z = null(Jw(1:2,:));
            z_bool = z>0;
            ceq = sum(z_bool) - 4;
            c = [];
        end
      end
      
 
   end
end