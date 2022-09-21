
function [B,B_dot] = bezier_curve(P0,P1,P2,Ts,Tf)
    dt = Ts/Tf;
    t = 0:dt:1;  
    B = (1-t).*((1-t).*P0 + t.*P1) + t.*((1-t).*P1 + t.*P2);
    B_dot = t.*(2*P0 - 4*P1 + 2*P2) + (-2*P0 + 2*P1); 
    B_dot = B_dot*dt/Ts;

end