function [xdot]= actor_critic(t,x)
%% desired trajectory
a1=5; a2=0.1; per=2; freq=pi/per; %amplitudes and period
yd(1)=a1*sin(0.5*t);
%yd(1)=a1;
%yd(2)=a1;
yd(2)=a1*cos(0.5*t);% qd是行向量

%dyd(1)=0;
%dyd(2)=0;
dyd(1)=a1*0.5*cos(0.5*t); 
dyd(2)=-a1*0.5*sin(0.5*t);

%ddyd(1)=0;
%ddyd(2)=0;
ddyd(1)=-a1*0.5^2*sin(0.5*t);
ddyd(2)=-a1*0.5^2*cos(0.5*t);

%%
Node = 256;
W_c=(x(7:262)); % NN nodes = 36
W_a=(x(263:518)); % NN nodes = 36

%% computation of the tracking errors
e=[x(1) x(2)]-yd; e=e'; %保证e是列向量
de=[x(3) x(4)]-dyd; de=de';

%实际误差
z = [e;de];

%% computation of mass inertia matrix
m1=1; m2=1; l1=0.8; l2=0.7; %mass of robot arms and length of links
m11=(m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(x(2));
m12=m2*l2^2+m2*l1*l2*cos(x(2));
m21=m12;
m22=m2*l2^2;
M = [m11 m12; m21 m22];   
%% computation of C matrix
c11=-m1*l1*l2*x(4)*sin(x(2));
c12=-m2*l1*l2*(x(3)+x(4))*sin(x(2));
c21=m2*l1*l2*x(3)*sin(x(2));
c22=0;
C=[c11 c12; c21 c22]*[x(3);x(4)];
%C1=[c11 c12; c21 c22];
 %% computation of G matrix
g1=m1*l1*9.8*cos(x(1))+m2*9.8*(l2*cos(x(1)+x(2))+l1*cos(x(1)));
g2=m2*l2*9.8*cos(x(1)+x(2));
G = [g1;g2];

%% computation of p matrix 4*1
p = [x(3);
     x(4);
     -inv(M)*C-G];% -inv(M)*C-G为p矩阵的第三和第四行

%% computation of q matrix 4*2
q = [0 0;
     0 0;
    inv(M)];

%% calculating control torques
u=[x(5);x(6)];

%% nn parameters
Beta = 100;
kc = 5;
ka = 30;

% 函数的方差为1
variance = 1;
%Node = 256;
k=1;
Yita=zeros(4,Node);% 2*36
for i1 = (-3:2:3)% nodes=36
    for i2 = (-3:2:3)
        for i3 = (-3:2:3)
            for i4 = (-3:2:3)
                Yita(1,k) = i1;
                Yita(2,k) = i2;
                Yita(3,k) = i3;
                Yita(4,k) = i4;
                k = k+1;
            end
        end
    end
end

Z=z; % 4*1
for i =1:Node
    S(i)=exp(-(Z-Yita(:,i))'*(Z-Yita(:,i))/variance); % 1*36
end
%% calculating Sdot_z 4*36
for i =1:Node
    Sdot_z(1,i) = -2*S(i)*(Z(1)-Yita(1,i));
    Sdot_z(2,i) = -2*S(i)*(Z(2)-Yita(2,i));
    Sdot_z(3,i) = -2*S(i)*(Z(3)-Yita(3,i));
    Sdot_z(4,i) = -2*S(i)*(Z(4)-Yita(4,i)); 
end
Sdot_zT = Sdot_z'; % 36*4
%% calculating input
%加入权重项 numda = 0.01 
u = -Beta*q'*z-0.5*q'*Sdot_z*W_a;
% calculating matrix
Q = q*q';
%temp = (p-Beta*Q*z-0.5*Q*Sdot_z*W_a-dyd);
Epi = Sdot_zT*(p-Beta*Q*z-0.5*Q*Sdot_z*W_a-[dyd';ddyd']); %36*1
Theta = Sdot_zT*q*q'*Sdot_z; %36*36

% updating laws
dW_c = -(kc*Epi)/(1+norm(Epi))*(Epi'*W_c-(Beta^2-1)*z'*Q*z+2*Beta*z'*(p-[dyd';ddyd'])+1/4*W_a'*Theta*W_a); %36*1
dW_a = 0.5*Sdot_zT*Q*z-ka*Theta*W_a+(kc/(4*(1+norm(Epi))))*Theta*W_a*Epi'*W_c;

%% dynamics solve
ydot = p + q*u;
xdot=[ydot(1);ydot(2); %% 输出位置向量
      ydot(3);ydot(4); %% 输出速度向量
      (u(1)-x(3));%%ode will not integrate this, output only
      (u(2)-x(4));%%ode will not integrate this, output only
      dW_c(1:256); dW_a(1:256)
      ];