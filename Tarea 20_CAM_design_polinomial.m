close all
clear
clc
%% characteristics of the cam mechanism
ro = 0.05;                       % minimum radius of the cam (m)
L  = 0.06;                       % maximum lift of the follower (m)
B0 = 0;                          % cam's initial angular position
B1 = 60;                         % cam's rotation after dwell (bottom)
B2 = 180;                        % cam's rotation after rise 
B3 = 300;                        % cam's rotation after return
B4 = 360;                        % cam's rotation after dwell (bottom)

beta  = [B1 B2 B3 B4]*pi/180;    % beta angles (rad)
theta = (0:1:360)*pi/180;        % cam's full rotation

%% definition of the angle segments
theta1   = theta(1+B0:1+B1);     % Dwell inferior (0° a 60°)
theta_RF = theta(1+B1:1+B3);     % TRAMO UNIFICADO RISE-FALL (60° a 300°)
theta4   = theta(1+B3:1+B4);     % Dwell inferior (300° a 360°)

delta_RF = (B3 - B1)*pi/180;
%% dwell (bottom)
x1 = zeros(1,length(theta1));
%% rise and return
u_RF = (theta_RF - beta(1)) / delta_RF; 
x_RF = L * (64*u_RF.^3 - 192*u_RF.^4 + 192*u_RF.^5 - 64*u_RF.^6);
x_RF = x_RF(2:end);
%% dwell (bottom)
x4 = zeros(1,length(theta4));
x4 = x4(2:end);
%% Follower displacement and cam profile
x   = [x1,x_RF,x4];                     % follower's displacement (m)
rho = ro+x;                              % cam's profile
%% computation of velocity and acceleration
w       = 1;                             % cam's angular velocity
t       = theta/w;                       % time vector (s)
x_dot   = gradient(x)./gradient(t);      % velocity (m/s)
x_ddot  = gradient(x_dot)./gradient(t);  % acceleration(m/s2)
x_dddot = gradient(x_ddot)./gradient(t); % jerk(m/s3)
%% plot cam profile and kinematic characteristics
figure
% plot the cam profile
subplot(1,2,1)
polarplot(theta,rho,'LineWidth',2);
title('Cam profile')
% plot the follower's displacement vs cam's angular displacement 
subplot(4,2,2)
plot(180*theta/pi,x,'Color','blue','LineWidth',2)
xlim([0,360])
ylim([min(x) max(x)])
xlabel('cam''s rotation angle (deg)')
ylabel('displacement (m)')
% plot the follower's velocity vs cam's angular displacement
subplot(4,2,4) 
plot(180*theta/pi,x_dot,'Color','red','LineWidth',2)
xlim([0,360])
ylim([min(x_dot) max(x_dot)])
xlabel('cam''s rotation angle (deg)')
ylabel('velocity (m/s)')
% plot the follower's acceleration vs cam's angular displacement
subplot(4,2,6) 
plot(180*theta/pi,x_ddot,'Color','green','LineWidth',2)
xlim([0,360])
ylim([min(x_ddot) max(x_ddot)])
xlabel('cam''s rotation angle (deg)')
ylabel('acceleration (m/s2)')
% plot the follower's jerk vs cam's angular displacement
subplot(4,2,8) 
plot(180*theta/pi,x_dddot,'Color','cyan','LineWidth',2)
xlim([0,360])
ylim([min(x_dddot) max(x_dddot)])
xlabel('cam''s rotation angle (deg)')
ylabel('jerk (m/s3)')
%% Animation (make sure that the cam_animation.m file is in the same directory)
%cam_animation(theta,rho,x,x_dot,x_ddot,x_dddot)