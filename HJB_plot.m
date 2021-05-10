clear
clc

x=[0;0;0;0];
W_c0([1:36],1) = 0.3;
W_a0([1:36],1) = 0.5;
x0=[x;W_c0;W_a0];

%t=linspace(t0,tf,300);
%[T,Y]=ode45('actor_critic',t,x0);

a1=1; a2=0.1; per=2; freq=pi/per; %amplitudes and period

yd(:,1) = a1*sin(0.5*T);
yd(:,2) = a1*cos(0.5*T);

%% error
error(:,1)=Y(:,1)-yd(:,1);
error(:,2)=Y(:,2)-yd(:,2);

variance = 1;
Node = 36;
k=1;
Yita=zeros(2,36);% 2*36
for j = 1:300 
    for i1 = (-1:2:1)% nodes=36
        for i2 = (-1:2:1)
            for i3 = (-1:2:1)
                for i4 = (-1:2:1)
                    Yita(1,k) = i1;
                    Yita(2,k) = i2;
                    Yita(3,k) = i3;
                    Yita(4,k) = i4;
                    k = k+1;
                end
            end
        end
     end
 end
    for i =1:Node
        S(i)=exp(-(error(j,:)'-Yita(:,i))'*(error(j,:)'-Yita(:,i))/variance); % 1*36     
    end
%每一轮都会重新计算
    critic(:,j)=Y(i,7:42)*S';
    actor(:,j)=Y(i,43:77)*S';  
end

%% 评价网络输出

Y(,5:6)

