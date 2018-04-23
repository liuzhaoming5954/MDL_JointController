vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%����켣����
t = 0:0.001:10;
x = t; y = sin(t); theta = atan(cos(t));
d_x = 1; d_y = cos(t); d_theta = -1/(1+cos(t).^2)*sin(t);

if (clientID<0)
    disp('Failed connecting to remote API server');    
else
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    %��ʼ��
    %��ñ��ض���ľ��
    [res,tip_handle] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
    %[res,tar_handle] = vrep.simxGetObjectHandle(clientID,'tar',vrep.simx_opmode_blocking); %�������dummy��λ�úͷ���ʵ�ֿ���С����λ��
    %[res,UR5_ikTip_handle] = vrep.simxGetObjectHandle(clientID,'UR5_ikTip',vrep.simx_opmode_blocking);
    %[res,UR5_ikTarget_handle] = vrep.simxGetObjectHandle(clientID,'UR5_ikTarget',vrep.simx_opmode_blocking);
    %[res,IKGrasp_pos_handle] = vrep.simxGetObjectHandle(clientID,'IKGrasp_pos',vrep.simx_opmode_blocking); %ͨ������IKGrasp_pos���dummy��λ�ã������е��ץȡ��λ��
    
    %��ʼ��һЩ����
    
    %����Ϊ�˼��ֱ���õ�����ʱ�����ƣ��Ľ��İ취�����޸�lua�ű�����һ��״̬��ɱ�־λ��ͨ������־λ��ʵ�ֿ���
    %С���Ŀ��ƺͻ�е�۵�ץȡ������luaʵ�֣�matlabֻ�������ϲ�滮��Ҳ���ǰ���lua��GUI����̨�Ľ�ɫ
    %vrep.simxSetObjectPosition(clientID,tar_handle,-1,[2,1,0],vrep.simx_opmode_oneshot);
    vrep.simxSetIntegerSignal(clientID,'state',1,vrep.simx_opmode_oneshot);%start move
    pause(15);
    vrep.simxSetIntegerSignal(clientID,'state',0,vrep.simx_opmode_oneshot);%stop move    
end
time = length(t);
for i = 1:time
    pos(1) = x; pos(2) = y; pos(3) = theta;
    vrep.simSetFloatSignal(clinetID,'position',pos(1),simx_opmode_oneshot);
    vrep.simSetFloatSignal(clinetID,'position',pos(2),simx_opmode_oneshot);
    vrep.simSetFloatSignal(clinetID,'position',pos(3),simx_opmode_oneshot);
    vel(1) = d_x; vel(2) = d_y; vel(3) = d_theta;
    vrep.simSetFloatSignal(clinetID,'velocity',vel(1),simx_opmode_oneshot);
    vrep.simSetFloatSignal(clinetID,'velocity',vel(2),simx_opmode_oneshot);
    vrep.simSetFloatSignal(clinetID,'velocity',vel(3),simx_opmode_oneshot);
end

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!
