%%
a1=5; a2=0.1; per=2; freq=pi/per; %amplitudes and period
yd(:,1)=a1*sin(0.5*T);
%yd(:,1)=a1;
%yd(:,2)=a1;
yd(:,2)=a1*cos(0.5*T);

%% error
error(:,1)=Y(:,1)-yd(:,1);
error(:,2)=Y(:,3)-yd(:,2);

%% plot error
figure; 
subplot(2,1,1)
plot(T,error(:,1));
%axis([0 20 -20 20]);
title('error state');
subplot(2,1,2)
plot(T,error(:,2));
%axis([0 20 -20 20]);
title('error state');
%% plot of actual trajectory and desired trajectory
figure;
subplot(2,1,1)
plot(T,Y(:,1),T,yd(:,1));
legend('q1','qd1');
title('desired trajectory qd1 and actual trajectory q1');
subplot(2,1,2)
plot(T,Y(:,3),T,yd(:,2));
title('desired trajectory qd1 and actual trajectory q2');
legend('q2','qd2');