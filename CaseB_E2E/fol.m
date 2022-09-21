clear
close all
Td = rand(4,100);

t = 0;
j = 2;
Tmj(:,1) = zeros(4,1);
Tm_vec = [];
Td_vec = [];

for i = 1:1:100
    T0 =  Tmj(:,end);
    j = 1;
    for td = 0.001:0.001:0.05
        Delta_Td = (Td(:,i)-T0);
        k = abs(0.3*Delta_Td);
        Tmj(:,j) = T0 + Delta_Td.*(ones(4,1)-exp(-td./k));
        %Tmmax = 1;
        %Delta_Td_ = Tmmax/(1 + (1/0.2)*exp(-td/0.2));
        %Delta_Tm =  (1-exp(-td/0.2))*min(Delta_Td_,Delta_Td);
        %Tm(j) = Tm(10*i-10+1) + Delta_Tm;
        %Tm(j) = (1-exp(-td/0.2))*(Td(i));
        

        %keyboard
        
        j = j+1;
    end
    Tm_vec = [Tm_vec,Tmj];
    Td_vec = [Td_vec,Td(:,i).*ones(4,j-1)];
t = t + 0.1;
%keyboard
end

plot(Td_vec(4,2:end))
hold on;
plot(Tm_vec(4,2:end))

