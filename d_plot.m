a1=5; a2=0.1; per=2; freq=pi/per; %amplitudes and period
yd(:,1)=a1*sin(0.5*T);
%yd(:,1)=a1;
%yd(:,2)=a1;
yd(:,2)=a1*cos(0.5*T);

dyd(:,1)=0.5*a1*cos(0.5*T);
dyd(:,2)=-0.5*a1*sin(0.5*T);

errord(:,1)=Y(:,2)-dyd(:,1);
errord(:,2)=Y(:,4)-dyd(:,2);

figure; 
subplot(2,1,1)
plot(T,errord(:,1));
%axis([0 20 -20 20]);
title('error state');
subplot(2,1,2)
plot(T,errord(:,2));
%axis([0 20 -20 20]);
title('error state');

figure;
subplot(2,1,1)
plot(T,Y(:,2),T,dyd(:,1));
legend('dq1','dqd1');
title('desired trajectory dqd1 and actual trajectory dq1');
subplot(2,1,2)
plot(T,Y(:,4),T,dyd(:,2));
title('desired trajectory dqd1 and actual trajectory dq2');
legend('dq2','dqd2');

