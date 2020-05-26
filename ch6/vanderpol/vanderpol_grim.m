%
figure, plot(TIME, X(:,1),'r',TIME, XHAT(:,1),'b'),title('x1')
figure, plot(TIME, X(:,2),'r',TIME, XHAT(:,2),'b'),title('x2')
figure, plot(TIME, X(:,3),'r',TIME, XHAT(:,3),'b'),title('x3')
figure, plot(TIME, X(:,4),'r',TIME, XHAT(:,4),'b'),title('x4')
figure, plot(TIME, X(:,5),'r',TIME, XHAT(:,5),'b'),title('x5')
    
figure, plot(TIME, X(:,1)-XHAT(:,1),'r', ...
    TIME,sqrt(PHAT(:,1)),'b',TIME,-sqrt(PHAT(:,1)),'b'),title('x1')
figure, plot(TIME, X(:,2)-XHAT(:,2),'r', ...
    TIME,sqrt(PHAT(:,2)),'b',TIME,-sqrt(PHAT(:,2)),'b'),title('x2')
figure, plot(TIME, X(:,3)-XHAT(:,3),'r', ...
    TIME,sqrt(PHAT(:,3)),'b',TIME,-sqrt(PHAT(:,3)),'b'),title('x3')
figure, plot(TIME, X(:,4)-XHAT(:,4),'r', ...
    TIME,sqrt(PHAT(:,4)),'b',TIME,-sqrt(PHAT(:,4)),'b'),title('x4')
figure, plot(TIME, X(:,5)-XHAT(:,5),'r', ...
    TIME,sqrt(PHAT(:,5)),'b',TIME,-sqrt(PHAT(:,5)),'b'),title('x5')

figure, plot(TIME, Z(:,1)-ZHAT(:,1),'r', ...
    TIME,sqrt(SBAR(:,1)),'b',TIME,-sqrt(SBAR(:,1)),'b'),title('z')