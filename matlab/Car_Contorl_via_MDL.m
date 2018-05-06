clear;
clc;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
% 启动vrep的同步模式
vrep.simxSynchronous(clientID,true);

load('MDLg_string1.mat');
%输入轨迹数据
t = 0:0.01:10; %运行十秒钟，以1ms为控制周期

% x = t; y = sin(pi/5*t); theta = atan(pi/5*cos(pi/5*t));
% Vx(1,[1:10001]) = 1; Vy = pi/5*cos(pi/5*t); 
% temp = -1./(1+(pi/5*cos(t)).^(2));
% Vtheta = temp.*sin(pi/5*t)*(pi/5)^2;
% Vrob = cos(theta).*Vx+sin(theta).*Vy;

%小车参数
r = 0.1175; %车轮半径
d = 0.7;    %车轮间距
%时间长度
time = length(t);
%滑模控制器参数
k1 = 1; k2 = 2; k3 = 3;
if (clientID<0)
    disp('Failed connecting to remote API server');    
else
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    %初始化
    %获得被控对象的句柄
    [res,tip_handle] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
    [res,LeftMotor] = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking); 
    [res,RightMotor] = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking);  
end
%n = length(A);
for i = 1:time
    % 计算x
    x(i) = A{1}(1)*F_P{1}{1}(t(i)/B{1}(1))+A{1}(2)*F_P{1}{2}(t(i)/B{1}(1))+A{1}(3)*F_P{1}{3}(t(i)/B{1}(1))+A{1}(4)*F_P{1}{4}(t(i)/B{1}(1))+A{1}(5)*F_P{1}{5}(t(i)/B{1}(1))+A{1}(6)*F_P{1}{6}(t(i)/B{1}(1));
    % 计算y
    if (t(i)<4.980)
        y(i) = A{2}(1,1)*F_P{2}{1}(t(i)/B{2}(1))+A{2}(1,2)*F_P{2}{2}(t(i)/B{2}(1))+A{2}(1,3)*F_P{2}{3}(t(i)/B{2}(1))+A{2}(1,4)*F_P{2}{4}(t(i)/B{2}(1))+A{2}(1,5)*F_P{2}{5}(t(i)/B{2}(1))+A{2}(1,6)*F_P{2}{6}(t(i)/B{2}(1))+A{2}(1,7)*F_P{2}{7}(t(i)/B{2}(1));
    else
        tt = t(i)-4.980;
        y(i) = A{2}(3,1)*F_P{2}{1}(tt/B{2}(3))+A{2}(3,2)*F_P{2}{2}(tt/B{2}(3))+A{2}(3,3)*F_P{2}{3}(tt/B{2}(3))+A{2}(3,4)*F_P{2}{4}(tt/B{2}(3))+A{2}(3,5)*F_P{2}{5}(tt/B{2}(3))+A{2}(3,6)*F_P{2}{6}(tt/B{2}(3))+A{2}(3,7)*F_P{2}{7}(tt/B{2}(3));
    end
    % 计算theta
    if(t(i)<2.480)
        theta(i) = A{3}(1,1)*F_P{3}{1}(t(i)/B{3}(1))+A{3}(1,2)*F_P{3}{2}(t(i)/B{3}(1))+A{3}(1,3)*F_P{3}{3}(t(i)/B{3}(1))+A{3}(1,4)*F_P{3}{4}(t(i)/B{3}(1))+A{3}(1,5)*F_P{3}{5}(t(i)/B{3}(1))+A{3}(1,6)*F_P{3}{6}(t(i)/B{3}(1))+A{3}(1,7)*F_P{3}{7}(t(i)/B{3}(1));
    else if(t(i)<7.460)
            tt = t(i)-2.480;
            theta(i) = A{3}(3,1)*F_P{3}{1}(tt/B{3}(3))+A{3}(3,2)*F_P{3}{2}(tt/B{3}(3))+A{3}(3,3)*F_P{3}{3}(tt/B{3}(3))+A{3}(3,4)*F_P{3}{4}(tt/B{3}(3))+A{3}(3,5)*F_P{3}{5}(tt/B{3}(3))+A{3}(3,6)*F_P{3}{6}(tt/B{3}(3))+A{3}(3,7)*F_P{3}{7}(tt/B{3}(3));
        else
            tt = t(i)-7.460;
            theta(i) = A{3}(5,1)*F_P{3}{1}(tt/B{3}(5))+A{3}(5,2)*F_P{3}{2}(tt/B{3}(5))+A{3}(5,3)*F_P{3}{3}(tt/B{3}(5))+A{3}(5,4)*F_P{3}{4}(tt/B{3}(5))+A{3}(5,5)*F_P{3}{5}(tt/B{3}(5))+A{3}(5,6)*F_P{3}{6}(tt/B{3}(5))+A{3}(5,7)*F_P{3}{7}(tt/B{3}(5));
        end
    end
    % 计算Vrob
    Vrob(i) = A{4}(1)*F_P{4}{1}(t(i)/B{4}(1))+A{4}(2)*F_P{4}{2}(t(i)/B{4}(1))+A{4}(3)*F_P{4}{3}(t(i)/B{4}(1))+A{4}(4)*F_P{4}{4}(t(i)/B{4}(1))+A{4}(5)*F_P{4}{5}(t(i)/B{4}(1))+A{4}(6)*F_P{4}{6}(t(i)/B{4}(1));
    % 计算Vtheta
    Vtheta(i) = A{5}(1)*F_P{5}{1}(t(i)/B{5}(1))+A{5}(2)*F_P{5}{2}(t(i)/B{5}(1))+A{5}(3)*F_P{5}{3}(t(i)/B{5}(1))+A{5}(4)*F_P{5}{4}(t(i)/B{5}(1))+A{5}(5)*F_P{5}{5}(t(i)/B{5}(1))+A{5}(6)*F_P{5}{6}(t(i)/B{5}(1));
    
    [res,Dpos] = vrep.simxGetObjectPosition(clientID,tip_handle,-1,vrep.simx_opmode_oneshot);
    [res,Dangle] = vrep.simxGetObjectOrientation(clientID,tip_handle,-1,vrep.simx_opmode_oneshot);
    X_e = x(i)-Dpos(1);
    Y_e = y(i)-Dpos(2);
    Theta_e = theta(i)-Dangle(3);
    v = Vrob(i)*cos(Theta_e)+X_e;
    w = Vtheta(i)+k2*Vrob(i)*Y_e+k3*Vrob(i)*sin(Theta_e);
    
    vel_right = v/r+d*w/(2*r);
    vel_left = v/r-d*w/(2*r);
    vrep.simxSetJointTargetVelocity(clientID,LeftMotor,vel_left,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,RightMotor,vel_right,vrep.simx_opmode_oneshot);
    
    vrep.simxPauseCommunication(clientID,0);
    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
end

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!
