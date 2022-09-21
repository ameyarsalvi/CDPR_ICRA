classdef PRPRcommon
   properties (Constant)
        base_length = 0.860; % mm
        base_span = 0.570; % mm

        phiA = 0;
        phiB = pi/2;
        phiC = pi;
        phiD = 3*pi/2;
        
        A1 = [PRPRcommon.base_length/2-PRPRcommon.base_span/2;0];
        A2 = [PRPRcommon.base_length/2+PRPRcommon.base_span/2;0];
        
        B1 = [PRPRcommon.base_length;PRPRcommon.base_length/2-PRPRcommon.base_span/2];
        B2 = [PRPRcommon.base_length;PRPRcommon.base_length/2+PRPRcommon.base_span/2];
        
        C1 = [PRPRcommon.base_length/2+PRPRcommon.base_span/2;PRPRcommon.base_length];
        C2 = [PRPRcommon.base_length/2-PRPRcommon.base_span/2;PRPRcommon.base_length];
        
        D1 = [0;PRPRcommon.base_length/2+PRPRcommon.base_span/2];
        D2 = [0;PRPRcommon.base_length/2-PRPRcommon.base_span/2];
        
        b = 0.16;
        h = 0.04;
        
        EA_O = [-PRPRcommon.b/2; -PRPRcommon.h/2];
        EB_O = [PRPRcommon.b/2; -PRPRcommon.h/2];
        EC_O = [PRPRcommon.b/2; PRPRcommon.h/2];
        ED_O = [-PRPRcommon.b/2; PRPRcommon.h/2];
        
        P = [0;0]; 
   end

  properties
       M
       D
   end


  methods 
      function obj = PRPRcommon(model)
           obj.M = [model.m*eye(2) [0;0]; [0 0] PRPRcommon.b*PRPRcommon.h*(PRPRcommon.b^2+PRPRcommon.h^2)/12];
           obj.D = [model.dx 0 0; 0 model.dy 0; 0 0 model.dz];
      end
      
      function [Y,ls0] = fwd_dynamics(obj,X_dyn,ls0,ls_ref,T_des,dt,tf)
        %m = 0.05;
        %d = 0.4;
        %gp = 0.9;
        jd = 1;
        l_dot_max = [0.05;0.05;0.05;0.05];
        l_dot_min = -[0.05;0.05;0.05;0.05];
        %Ls_dyn = [ls0;ls_dot0];
        l_dot = (ls_ref - ls0)/tf;
        l_dot = min(max(l_dot,l_dot_min),l_dot_max);

        q = 0.001;
        r = 0.000002;
        for t_dyn = dt:dt:tf
            %ls_dot = Ls_dyn(5:8,jd);
            %ls_ddot = -(gp/m)*Ls_dyn(1:4,jd) - (d/m)*ls_dot + (gp/m)*ls_ref;
            %Ls_dyn(:,jd+1) = Ls_dyn(:,jd) + [ls_dot;ls_ddot]*dt; 
            %Ls_dyn(5:8,jd+1) = min(max(l_dot_min,Ls_dyn(5:8,jd+1)),l_dot_max);
            ls0 = ls0 + l_dot*dt;
            P_x = obj.structureMatrix(X_dyn(1:3,jd),ls0);
            x_dot = X_dyn(4:6,jd);
            x_ddot = obj.M\(P_x*T_des) - obj.M\obj.D*x_dot ;% + sqrt(q)*randn(3,1)*sqrt(dt); % process noise
            
            X_dyn(1:3,jd+1) = X_dyn(1:3,jd) + x_dot*dt; 
            X_dyn(4:6,jd+1) = x_dot + x_ddot*dt;

            Y(:,jd+1) = X_dyn(:,jd+1);% + sqrt(r)*randn(6,1); % meas noise
            jd = jd+1;
        end
        
        X_dyn = X_dyn(:,2:end);
        Y = Y(:,2:end);
        %ls = Ls_dyn(1:4,2:end);
        %ls_dot1 = Ls_dyn(5:8,2:end);
      end
      
      
      function phi = fwd_kinematics(obj,Xe,l)
        % Determine angle
        phi0 = 0;
        phimin = -pi;
        phimax = pi;

        phi = fmincon(@(phi)potential_energy(phi), phi0,[],[],[],[],phimin,phimax);
        function U = potential_energy(phi)
            current_lp = obj.prismatic_length([Xe;phi],l);
            U = 0.5*(current_lp)'*(current_lp);
        end

      end
      
      function [A3,B3,C3,D3] = slider_positions(obj,l)
        A3 = obj.P + obj.A1 + [cos(obj.phiA) -sin(obj.phiA);sin(obj.phiA) cos(obj.phiA)]*[l(1);0];
        B3 = obj.P + obj.B1 + [cos(obj.phiB) -sin(obj.phiB);sin(obj.phiB) cos(obj.phiB)]*[l(2);0];
        C3 = obj.P + obj.C1 + [cos(obj.phiC) -sin(obj.phiC);sin(obj.phiC) cos(obj.phiC)]*[l(3);0];
        D3 = obj.P + obj.D1 + [cos(obj.phiD) -sin(obj.phiD);sin(obj.phiD) cos(obj.phiD)]*[l(4);0];  
      end

      function [EA_P,EB_P,EC_P,ED_P] = platform_coordinates(obj,X)
        pGo = obj.transformation_matrix(X);

        t1 = pGo*[obj.EA_O;1];
        t2 = pGo*[obj.EB_O;1];
        t3 = pGo*[obj.EC_O;1];
        t4 = pGo*[obj.ED_O;1];

        EA_P = t1(1:2);
        EB_P = t2(1:2);
        EC_P = t3(1:2);
        ED_P = t4(1:2);
      end

      function pGo = transformation_matrix(obj,X)
        phi_P = X(3);
        pGo = eye(3);
        pGo(1:2,1:2) = [cos(phi_P) -sin(phi_P); sin(phi_P) cos(phi_P)]; %rot_z
        pGo(1:2,3) = X(1:2);
      end

      function l_p = prismatic_length(obj,X,l)

        [EA_P,EB_P,EC_P,ED_P] = obj.platform_coordinates(X);       
        [A3,B3,C3,D3] = obj.slider_positions(l); 

        la = sqrt((A3(1)-EA_P(1))^2+(A3(2)-EA_P(2))^2);
        lb = sqrt((B3(1)-EB_P(1))^2+(B3(2)-EB_P(2))^2);
        lc = sqrt((C3(1)-EC_P(1))^2+(C3(2)-EC_P(2))^2);
        ld = sqrt((D3(1)-ED_P(1))^2+(D3(2)-ED_P(2))^2);

        l_p = [la,lb,lc,ld]';

      end
      
      function [theta1a,theta1b,theta1c,theta1d] = joint_angle(obj,X,l)
        [EA_P,EB_P,EC_P,ED_P] = obj.platform_coordinates(X);       
        [A3,B3,C3,D3] = obj.slider_positions(l); 

        va = [1,0]'; vb = [0,1]'; vc = [-1,0]'; vd = [0,-1]';
        ua = [(EA_P(1)-A3(1)),(EA_P(2)-A3(2))]'/(sqrt((EA_P(1)-A3(1))^2 + (EA_P(2)-A3(2))^2));
        ub = [(EB_P(1)-B3(1)),(EB_P(2)-B3(2))]'/(sqrt((EB_P(1)-B3(1))^2 + (EB_P(2)-B3(2))^2));
        uc = [(EC_P(1)-C3(1)),(EC_P(2)-C3(2))]'/(sqrt((EC_P(1)-C3(1))^2 + (EC_P(2)-C3(2))^2));
        ud = [(ED_P(1)-D3(1)),(ED_P(2)-D3(2))]'/(sqrt((ED_P(1)-D3(1))^2 + (ED_P(2)-D3(2))^2));

        theta1a = acos(max(min(dot(ua,va)/(norm(ua)*norm(va)),1),-1));
        theta1b = acos(max(min(dot(ub,vb)/(norm(ub)*norm(vb)),1),-1));
        theta1c = acos(max(min(dot(uc,vc)/(norm(uc)*norm(vc)),1),-1));
        theta1d = acos(max(min(dot(ud,vd)/(norm(ud)*norm(vd)),1),-1));
      end

      function Jw = structureMatrix(obj,X,l)
        [A3,B3,C3,D3] = obj.slider_positions(l); 
        
        phi_e = X(3);
        pRo = [cos(phi_e) -sin(phi_e); sin(phi_e) cos(phi_e)]; %rot_z
                
        la = A3(1:2) - X(1:2) - pRo*obj.EA_O;
        lb = B3(1:2) - X(1:2) - pRo*obj.EB_O;
        lc = C3(1:2) - X(1:2) - pRo*obj.EC_O;
        ld = D3(1:2) - X(1:2) - pRo*obj.ED_O;
        
        ua = [(la/norm(la));0];
        ub = [(lb/norm(lb));0];
        uc = [(lc/norm(lc));0];
        ud = [(ld/norm(ld));0]; 
        
        r1 = [pRo*obj.EA_O;0];
        r2 = [pRo*obj.EB_O;0];
        r3 = [pRo*obj.EC_O;0];
        r4 = [pRo*obj.ED_O;0];
        
        baxua = cross(r1,ua);
        bbxub = cross(r2,ub);
        bcxuc = cross(r3,uc);
        bdxud = cross(r4,ud);
                
        Jw = [ua(1:2), ub(1:2), uc(1:2), ud(1:2);
              baxua(3),bbxub(3),bcxuc(3),bdxud(3)];
        
      end

  end
end