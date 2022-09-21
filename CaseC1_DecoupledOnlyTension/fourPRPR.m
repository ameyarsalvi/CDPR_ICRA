
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
      
      function [l,fval] = minimize_objective(obj, x1i,l_init)
      % Maximize the manipulability ellipsoid.
        lb = [0;0;0;0];
        ub = [obj.base_span; obj.base_span; obj.base_span; obj.base_span];

        Aineq = [];
        bineq = [];
        Aeq = [];
        Beq = [];

        fnonlcon = @(l)ensure_wrench_closure(l);

        [l,fval] = fmincon(@(l)objective_fn(l),l_init,Aineq,bineq,Aeq,Beq,lb,ub,fnonlcon);

        function objf = objective_fn(l)
            Jw = obj.structureMatrix(x1i,l);
            s = svd(Jw(1:2,:)); %Manipulability or Dexterity 
            objf1 = -min(s)/max(s);
            z = null(Jw); % Wrench Quality
            if sum(z>0)==4 || sum(z<0)==4
                objf2 = -min(abs(z))/max(abs(z));
            else
                objf2 = -min(z)/max(z);
            end
            if objf2 > 0
                objf1 = 0;
            end
            objf = objf1 + objf2;
        end

        function [c,ceq] = ensure_wrench_closure(l)          
            % Ensure wrench closure
            c = [];
            Jw = obj.structureMatrix(x1i,l);
            %z = null(Jw);
            %z_bool = z>0;
            %ceq = sum(z_bool) - 4;

            at1 = -sum(Jw(:,[1,2,3]),2);
            at2 = -sum(Jw(:,[2,3,4]),2);
            at3 = -sum(Jw(:,[3,4,1]),2);
            at4 = -sum(Jw(:,[4,1,2]),2);
            Gd1 = Jw(:,[2,3,4]);
            Gd2 = Jw(:,[3,4,1]);
            Gd3 = Jw(:,[4,1,2]);
            Gd4 = Jw(:,[1,2,3]);
            h1 = Gd1\at1;
            h2 = Gd2\at2;
            h3 = Gd3\at3;
            h4 = Gd4\at4;
            hp1 = sum(h1>=0);
            hp2 = sum(h2>=0);
            hp3 = sum(h3>=0);
            hp4 = sum(h4>=0);
            if hp1 == 3 || hp2 == 3 || hp3 == 3 || hp4 == 3
                ceq = 0;
            else
                ceq = 1;
            end
        end
      end
      
 
   end
end