clear;
clc;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
% 启动vrep的同步模式
vrep.simxSynchronous(clientID,true);

%输入轨迹数据
t = 0:0.01:10; %以10ms为控制周期
x = t; y = sin(pi/5*t); theta = atan(pi/5*cos(pi/5*t));
Vx(1,[1:1001]) = 1; Vy = pi/5*cos(pi/5*t); 
temp = -1./(1+(pi/5*cos(t)).^(2));
Vtheta = temp.*sin(pi/5*t)*(pi/5)^2;
Vrob = cos(theta).*Vx+sin(theta).*Vy;
%Vrob = 1./cos(theta);
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
    %初始化一些变量
    
    %这里为了简便直接用的是延时来控制，改进的办法可以修改lua脚本返回一个状态完成标志位，通过检测标志位来实现控制
    %小车的控制和机械臂的抓取均采用lua实现，matlab只用来做上层规划，也就是扮演lua中GUI控制台的角色

    %vrep.simxSetIntegerSignal(clientID,'state',1,vrep.simx_opmode_oneshot);%start move
    %pause(15);
    %vrep.simxSetIntegerSignal(clientID,'state',0,vrep.simx_opmode_oneshot);%stop move    
end

for i = 1:time
%     pos(1) = x; pos(2) = y; pos(3) = theta;
%     vrep.simSetFloatSignal(clinetID,'position',pos(1),simx_opmode_oneshot);
%     vrep.simSetFloatSignal(clinetID,'position',pos(2),simx_opmode_oneshot);
%     vrep.simSetFloatSignal(clinetID,'position',pos(3),simx_opmode_oneshot);
%     vel(1) = d_x; vel(2) = d_y; vel(3) = d_theta;
%     vrep.simSetFloatSignal(clinetID,'velocity',vel(1),simx_opmode_oneshot);
%     vrep.simSetFloatSignal(clinetID,'velocity',vel(2),simx_opmode_oneshot);
%     vrep.simSetFloatSignal(clinetID,'velocity',vel(3),simx_opmode_oneshot);
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
