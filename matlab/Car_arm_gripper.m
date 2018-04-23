vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID<0)
    disp('Failed connecting to remote API server');    
else
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    %初始化
    %获得被控对象的句柄
    [res,tip_handle] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
    [res,tar_handle] = vrep.simxGetObjectHandle(clientID,'tar',vrep.simx_opmode_blocking); %控制这个dummy的位置和方向，实现控制小车的位置
    [res,UR5_ikTip_handle] = vrep.simxGetObjectHandle(clientID,'UR5_ikTip',vrep.simx_opmode_blocking);
    [res,UR5_ikTarget_handle] = vrep.simxGetObjectHandle(clientID,'UR5_ikTarget',vrep.simx_opmode_blocking);
    [res,IKGrasp_pos_handle] = vrep.simxGetObjectHandle(clientID,'IKGrasp_pos',vrep.simx_opmode_blocking); %通过控制IKGrasp_pos这个dummy的位置，代表机械臂抓取的位置
    
    %初始化一些变量
    
    %这里为了简便直接用的是延时来控制，改进的办法可以修改lua脚本返回一个状态完成标志位，通过检测标志位来实现控制
    %小车的控制和机械臂的抓取均采用lua实现，matlab只用来做上层规划，也就是扮演lua中GUI控制台的角色
    %vrep.simxSetObjectPosition(clientID,tar_handle,-1,[2,1,0],vrep.simx_opmode_oneshot);
    vrep.simxSetIntegerSignal(clientID,'state',1,vrep.simx_opmode_oneshot);%start move
    pause(15);
    vrep.simxSetIntegerSignal(clientID,'state',0,vrep.simx_opmode_oneshot);%stop move
    %vrep.simxSetIntegerSignal(clientID,'IK_flag',1,vrep.simx_opmode_oneshot);%开始抓取/释放
    %vrep.simxSetIntegerSignal(clientID,'Grasp_open_flag',1,vrep.simx_opmode_oneshot);%1表示抓取，0表示释放
    %pause(10);
    %vrep.simxSetIntegerSignal(clientID,'IK_flag',1,vrep.simx_opmode_oneshot);
    %vrep.simxSetIntegerSignal(clientID,'Grasp_open_flag',0,vrep.simx_opmode_oneshot);
    %pause(10);
    
end
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!
