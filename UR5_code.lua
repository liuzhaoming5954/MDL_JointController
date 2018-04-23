---完成三个步骤，第一个是运动到指定点，第二个是抓住\释放物体，第三个是举起来

---IK
enableIk=function(enable)
    if enable then
        simSetObjectMatrix(ikTarget,-1,simGetObjectMatrix(ikTip,-1))
        for i=1,#jointHandles,1 do
            simSetJointMode(jointHandles[i],sim_jointmode_ik,1)
        end

        simSetExplicitHandling(ikGroupHandle,0)
    else
        simSetExplicitHandling(ikGroupHandle,1)
        for i=1,#jointHandles,1 do
            simSetJointMode(jointHandles[i],sim_jointmode_force,0)
        end
    end
end

--控制手抓进行抓取，后面两个参数可以省略
setGripperData=function(open,velocity,force)
    if not velocity then
        velocity=0.11
    end
    if not force then
        force=20
    end
    if not open then
        velocity=-velocity
    end
    local data=simPackFloats({velocity,force})
    simSetStringSignal('UR5_rg2GripperData',data)
end

--主循环，在80行左右调用
threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
		Action_flag = simGetIntegerSignal('Action_flag')
		Grasp_open_flag = simGetIntegerSignal('Grasp_open_flag')
		--Grasp_flag = simGetIntegerSignal('Grasp_flag')
		if Action_flag==1 then
			if Fsm_step == 0 then
				Fsm_step =1 
			elseif Fsm_step ==1 then
				enableIk(true)
				
				pos=simGetObjectPosition(IKGrasp_pos,-1)
				quat=simGetObjectQuaternion(IKGrasp_pos,-1)
				
				simRMLMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1]+0.105,pos[2],pos[3]},quat,nil)
				
				simRMLMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2],pos[3]},quat,nil)
				simWait(0.5)
				Fsm_step = 2  --切换状态，正常的状态机需要一个条件语句进行切换，这是是执行完之后自动进入下一个状态
			elseif Fsm_step ==2 then
				enableIk(false)
				if Grasp_open_flag ==0 then
					setGripperData(true)
				else
					setGripperData(false)
				end
				
				simWait(1)				
				Fsm_step = 3
			elseif Fsm_step ==3 then
				simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPosBack,targetVel)
				simWait(1)
				Fsm_step =4
			elseif Fsm_step ==4 then
				simSetIntegerSignal('IK_res',1)
			end
				
		else
			Fsm_step = 0
		end
		simSwitchThread() --释放线程，完成操作，这个是threaded script必须要加入的
        
    end
end

-- 放置初始化代码
simSetThreadSwitchTiming(200) -- set the thread switch timing to the maximum (200 milliseconds)

----arm

Action_flag = simGetIntegerSignal('Action_flag') --判别是否需要执行抓取操作，如果为1表示执行，执行完之后会重置为0（通过IK_res）
Grasp_flag = simGetIntegerSignal('Grasp_flag') --暂时还没用到
Fsm_step = 0 --状态机的当前状态标志位。实现多流程控制的时候一定要学会用状态机，大家可以参考我的例程进行学习。


jointHandles={-1,-1,-1,-1,-1,-1}
for i=1,6,1 do
	jointHandles[i]=simGetObjectHandle('UR5_joint'..i)
end
ikGroupHandle=simGetIkGroupHandle('UR5') --IK group是一个非常重要的手段，用于计算运动学逆解
ikTip=simGetObjectHandle('UR5_ikTip')  --Tip一般表示机械臂上的固连点
ikTarget=simGetObjectHandle('UR5_ikTarget') --Target一般表示机械臂的目标点
IKGrasp_pos = simGetObjectHandle('IKGrasp_pos')
targetPosBack = {0,0,0,0,0,0}
---RML var
vel=180*math.pi/180
accel=40*math.pi/180
jerk=80*math.pi/180
currentVel={0,0,0,0,0,0}
currentAccel={0,0,0,0,0,0}
maxVel={vel,vel,vel,vel,vel,vel}
maxAccel={accel,accel,accel,accel,accel,accel}
maxJerk={jerk,jerk,jerk,jerk,jerk,jerk}
targetVel={0,0,0,0,0,0}

ikMaxVel={0.4,0.4,0.4,1.8}
ikMaxAccel={0.8,0.8,0.8,0.9}
ikMaxJerk={0.6,0.6,0.6,0.8}

Grasp_open_flag = simGetIntegerSignal('Grasp_open_flag')
setGripperData(true)

-- Here we execute the regular thread code:
res,err=pcall(threadFunction)
if not res then
	simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Put some clean-up code here: