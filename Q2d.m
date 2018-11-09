%% AE 410 Assignment -2
% Submitted by : Ram Milan Verma; 150010037
%% Q2.(d)
clear; close all;
in_dev=30; %Missile has an initial deviation of 30 degrees anticlockwise in its velocity direction.
gamaT=120; VT=250; VM=500;
r(1)=10000;
theta(1)=30;
gamaM(1)=theta(1) + in_dev;
Vr0=VT*cosd(gamaT-theta(1))-VM*cosd(gamaM(1)-theta(1));
tf = r(1)*(Vr0 + 2*VM)/(VM^2-VT^2); % time of collision
dt=1e-3;  % integration time step
Ns=floor(tf/dt); % number of samples
t=linspace(0,tf,Ns);
xM(1)=0; yM(1)=0; %missile's initial coordinate assumed to (0,0) WLOG
xT(1)=r(1)*cosd(theta(1)); yT(1)=r(1)*sind(theta(1)); % initial position of target w.r.t. missile
K=10; % gain 
for i=1:Ns
    % rate calculatio step
    rdot= VT*cosd(gamaT-theta(i)) - VM;
    theta_dot=VT*sind(gamaT-theta(i))/r(i); %rad / sec
    aM(i)=VM*theta_dot -K*(gamaM(i)-theta(i)); %First term nullifies missile's fight path angle rate with LOS rate and
    % second term generates lateral acceleration proportional to angular difference; K is the gain
    gamaM_dot=aM(i)/VM; % rad /sec
    % update step
    r(i+1)=r(i) + rdot*dt;
    theta(i+1)=theta(i) + theta_dot*dt*180/pi;
    gamaM(i+1)= gamaM(i) +gamaM_dot*dt*180/pi;
    xM(i+1)=xM(i)+ VM*cosd(gamaM(i+1))*dt;
    yM(i+1)=yM(i)+ VM*sind(gamaM(i+1))*dt;
    xT(i+1)=xT(i)+ VT*cosd(gamaT)*dt;
    yT(i+1)= yT(i)+VT*sind(gamaT)*dt;
    if r(i)<=0 
        break
    end
end
% plotting of trajectory and guidance command
figure
plot(xM,yM,xT,yT), xlabel('x(in m)'),ylabel('y(in m)'),legend('missile', 'target'), title('trajectory'), grid on
annotation('textbox', [0.5, 0.2, 0.1, 0.1], 'String',strcat('tf= ',num2str(i*dt), ' sec'));
dim = [.1 .4 .2 .2];
str = strcat('collision @ x= ',num2str(xM(i)),' ,y= ',num2str(yM(i)));
annotation('textbox',dim,'String',str,'FitBoxToText','on');
figure
plot(t(1:i),aM),xlabel('time(sec)'),ylabel('Missile acceleration(m/sec^2)'),legend('aM'), title('Guidance command'); grid on