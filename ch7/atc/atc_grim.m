%
% plotting
%
figure, plot(X(:,1)/1000,X(:,3)/1000,'r',XHAT(:,1)/1000,XHAT(:,3)/1000,'b')
figure, plot(TIME, X(:,1)-XHAT(:,1),'r',TIME,sqrt(PHAT(:,1)),'b',TIME,-sqrt(PHAT(:,1)),'b'),title('x1')
figure, plot(TIME, X(:,2)-XHAT(:,2),'r',TIME,sqrt(PHAT(:,2)),'b',TIME,-sqrt(PHAT(:,2)),'b'),title('x2')
figure, plot(TIME, X(:,3)-XHAT(:,3),'r',TIME,sqrt(PHAT(:,3)),'b',TIME,-sqrt(PHAT(:,3)),'b'),title('x3')
figure, plot(TIME, X(:,4)-XHAT(:,4),'r',TIME,sqrt(PHAT(:,4)),'b',TIME,-sqrt(PHAT(:,4)),'b'),title('x4')

figure, plot(TIME,WW(:,1),'r',TIME,WW(:,2),'b')
