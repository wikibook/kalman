%
xr=6374;
x=linspace(xr-10,xr,100)';
y1=xr.*sqrt(1-x.^2./xr^2);
y2=-xr.*sqrt(1-x.^2./xr^2);
figure, plot(X(:,1), X(:,2),'r', XHAT(:,1),XHAT(:,2),'b', x, y1, 'k', x,y2,'k', xr, 0, 'ok'),title('x1-x2')
axis([6365 6510 -100 360])

figure, plot(TIME, X(:,1), 'r', TIME, XHAT(:,1),'b'),title('x1')
figure, plot(TIME, X(:,2), 'r', TIME, XHAT(:,2),'b'),title('x2')
figure, plot(TIME, sqrt(X(:,1).^2+X(:,2).^2), 'r', TIME, sqrt(XHAT(:,1).^2+XHAT(:,2).^2),'b'),title('R')
figure, plot(TIME, X(:,5), 'r', TIME, XHAT(:,5),'b'),title('x5')

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
    TIME,sqrt(SBAR(:,1)),'b',TIME,-sqrt(SBAR(:,1)),'b'),title('r')
figure, plot(TIME, (Z(:,2)-ZHAT(:,2))*180/pi,'r', ...
    TIME,sqrt(SBAR(:,2))*180/pi,'b',TIME,-sqrt(SBAR(:,2))*180/pi,'b'),title('\theta')