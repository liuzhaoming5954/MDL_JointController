clear;
clc;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
% ����vrep��ͬ��ģʽ
vrep.simxSynchronous(clientID,true);

%����켣����
t = 0:0.01:10; %��10msΪ��������
x = t; y = sin(pi/5*t); theta = atan(pi/5*cos(pi/5*t));
Vx(1,[1:1001]) = 1; Vy = pi/5*cos(pi/5*t); 
temp = -1./(1+(pi/5*cos(t)).^(2));
Vtheta = temp.*sin(pi/5*t)*(pi/5)^2;
Vrob = cos(theta).*Vx+sin(theta).*Vy;
%Vrob = 1./cos(theta);
%С������
r = 0.1175; %���ְ뾶
d = 0.7;    %���ּ��
%ʱ�䳤��
time = length(t);
%��ģ����������
k1 = 1; k2 = 2; k3 = 3;
if (clientID<0)
    disp('Failed connecting to remote API server');    
else
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    %��ʼ��
    %��ñ��ض���ľ��
    [res,tip_handle] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
    [res,LeftMotor] = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking); 
    [res,RightMotor] = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking); 
    %��ʼ��һЩ����
    
    %����Ϊ�˼��ֱ���õ�����ʱ�����ƣ��Ľ��İ취�����޸�lua�ű�����һ��״̬��ɱ�־λ��ͨ������־λ��ʵ�ֿ���
    %С���Ŀ��ƺͻ�е�۵�ץȡ������luaʵ�֣�matlabֻ�������ϲ�滮��Ҳ���ǰ���lua��GUI����̨�Ľ�ɫ

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
