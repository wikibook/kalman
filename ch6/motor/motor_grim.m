%

figure, plot(TIME, X(:,1)*180/pi,'r',TIME, XHAT(:,1)*180/pi,'b'),title('x1')
figure, plot(TIME, X(:,2),'r',TIME, XHAT(:,2),'b'),title('x2')
    
figure, plot(TIME, (X(:,1)-XHAT(:,1))*180/pi,'r', ...
    TIME,sqrt(PHAT(:,1))*180/pi,'b',TIME,-sqrt(PHAT(:,1))*180/pi,'b'),title('x1')
figure, plot(TIME, X(:,2)-XHAT(:,2),'r', ...
    TIME,sqrt(PHAT(:,2)),'b',TIME,-sqrt(PHAT(:,2)),'b'),title('x2')

figure, plot(TIME, U,'r'),title('u')

figure, plot(TIME, (Z(:,1)-ZHAT(:,1))*180/pi,'r', ...
    TIME,sqrt(SBAR(:,1))*180/pi,'b',TIME,-sqrt(SBAR(:,1))*180/pi,'b'),title('z1')
figure, plot(TIME, (Z(:,2)-ZHAT(:,2))*180/pi,'r', ...
    TIME,sqrt(SBAR(:,2))*180/pi,'b',TIME,-sqrt(SBAR(:,2))*180/pi,'b'),title('z2')