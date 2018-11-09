%% AE 410 Assignment -2
% Submitted by : Ram Milan Verma; 150010037
%% Q.2(a) & (b) Pure Pursuit Guidance
%% Wrong way encountered
% tf=31.25; % time of collision
% Ns=100;  % Number of samples of time,t 
% t=linspace(0,tf,Ns);
% gamaT=120; VT=300; VM=500;
% K=10000;
% t=20; v=VM/VT;
% % theta=gamaT - acosd((tf-t)*(VM^2 - VT^2)/(r*VT)-v);
% %fun=@(r) K*((sind(gamaT-theta))^(v-1))/(1+cosd(gamaT-theta))^v - r;
% fun=@(r) K*((sind(gamaT-(gamaT - acosd((tf-t)*(VM^2 - VT^2)/(r*VT)-v))))^(v-1))/(1+cosd(gamaT-(gamaT - acosd((tf-t)*(VM^2 - VT^2)/(r*VT)-v))))^v - r;
% r=fsolve(fun,10000); % this is giving complex solution of r after some time;
% I tried to use another solvers like fzero,lsqnonlin with different
% initial conditions for search but that gives complex solution of r after
% sometime.  So, need to use other way
% and here it is 
%% Q2.(a) & (b) Pure Pursuit Guidance
clear; close all;
gamaT=120; VT=250; VM=500;
r(1)=10000;
theta(1)=30;
tf=r(1)*(VT*cosd(gamaT-theta(1))+VM)/(VM^2-VT^2); % time of collision
dt=1e-3;  % integration time step
Ns=floor(tf/dt); % number of samples
t=linspace(0,tf,Ns);
xM(1)=0; yM(1)=0; %missile's initial coordinate assumed to (0,0) WLOG
xT(1)=10000*cosd(30); yT(1)=10000*sind(30); % initial position of target w.r.t. missile
for i=1:Ns
    % rate calculatio step
    rdot= VT*cosd(gamaT-theta(i)) - VM;
    theta_dot=VT*sind(gamaT-theta(i))/r(i); %rad / sec
    aM(i)=VM*theta_dot;
    % update step
    r(i+1)=r(i) + rdot*dt;
    theta(i+1)=theta(i) + theta_dot*dt*180/pi;
    xM(i+1)=xM(i)+ VM*cosd(theta(i+1))*dt;
    yM(i+1)=yM(i)+ VM*sind(theta(i+1))*dt;
    xT(i+1)=xT(i)+ VT*cosd(gamaT)*dt;
    yT(i+1)= yT(i)+VT*sind(gamaT)*dt;
end
% plotting of trajectory and guidance command
figure
plot(xM,yM,xT,yT), xlabel('x(in m)'),ylabel('y(in m)'),legend('missile', 'target'), title('trajectory'), grid on
annotation('textbox', [0.5, 0.2, 0.1, 0.1], 'String',strcat('tf= ',num2str(tf), ' sec'));
dim = [.1 .4 .2 .2];
str = strcat('collision @ x= ',num2str(xM(i)),' ,y= ',num2str(yM(i)));
annotation('textbox',dim,'String',str,'FitBoxToText','on');
figure
plot(t,aM),xlabel('time(sec)'),ylabel('Missile acceleration(m/sec^2)'),legend('aM'), title('Guidance command'); grid on
%% Q2.(b)
% VT=250; run the code for Q2.(a) with just changing VT=250;