
function [B,B_dot] = bezier_curve(P0,P1,P2,Ts,Tf)
    dt = Ts/Tf;
    t = 0:dt:1;
%     n = size(waypoints,2);
%     for k = 1:1:n
%         j = k-1;
%         for p = 1:1:j+1
%             i = p-1;
%             Pi = waypoints(:,p);
%             vi(:,p) = (((-1)^(i+j)).*Pi)/(factorial(i)*factorial(j-i));
%         end
%         sum_i = sum(vi,2);
%         B_j(:,:,k) = t.*((factorial(n)/factorial(n-j)).*sum_i);
%     end
%     
%     B = sum(B_j,3)

    
    B = (1-t).*((1-t).*P0 + t.*P1) + t.*((1-t).*P1 + t.*P2);
    B_dot = t.*(2*P0 - 4*P1 + 2*P2) + (-2*P0 + 2*P1); 
    B_dot = B_dot*dt/Ts;
end