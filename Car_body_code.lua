xml = [[
<ui closeable="true" onclose="closeEventHandler" resizable="true">
	<label text="Car control pan" wordwrap="true" />
	<group>
		<label text="Simulation time:" id = "999"  wordwrap="false" />
		
	</group>
	
	<group>
		<label text="State:init" id = "1000" wordwrap="true" />
		<button text="Start move"  onclick = "Start_move" />
		
		<stretch />
	</group>
	
	<group>
		<label text="IK_flag:off" id = "1010" wordwrap="true" />
		<radiobutton text="Loosen" onclick="radiobuttonClick" id="1011" />
        <radiobutton text="Grasp" onclick="radiobuttonClick" id="1012" checked = 'true' />
		<button text="IK_on" onclick = "IK_on" />
	</group>
	
	
	
</ui>
]]


---UI
--按键处理
function Start_move(h)
	if Start_flag then
		Start_flag = false
		state = 0
	else
		Start_flag = true
		state = 1
	end
end

function IK_on(h)
	
	IK_flag = 1	
end


function radiobuttonClick(ui,id)
    local sel={'Loosen','Grasp'}
    simSetIntegerSignal('Grasp_open_flag',id - 1011) -- 0 loosen 1:grasp
	
end

function closeEventHandler(h)
    simAddStatusbarMessage('Window '..h..' is closing...')
    simExtCustomUI_hide(h)
end
-----UI END

---IK


if (sim_call_type==sim_childscriptcall_initialization) then
	MotorHandle_Left=simGetObjectHandle('LeftMotor')
	MotorHandle_Right=simGetObjectHandle('RightMotor')
	tip_handle=simGetObjectHandle('tip')
    tar_handle=simGetObjectHandle('tar')
	ui=simExtCustomUI_create(xml)
	startTime=simGetSimulationTime()
	
	-----
	Left_vel = 0
	Right_vel = 0
	Start_flag = false
	curtime = 0
    state =0  --state也是状态机的状态标志位，这个脚本里实现的状态机是比较完整的
	
	Dpos = {0,0,0}
	Dangle = {0,0,0}
	simSetJointTargetVelocity(MotorHandle_Left,0)
    simSetJointTargetVelocity(MotorHandle_Right,0)
	
	----arm
	IK_flag = 0 --no IK
	jointHandles={-1,-1,-1,-1,-1,-1}
	for i=1,6,1 do
		jointHandles[i]=simGetObjectHandle('UR5_joint'..i)
	end
	ikGroupHandle=simGetIkGroupHandle('UR5')
	ikTip=simGetObjectHandle('UR5_ikTip')
	ikTarget=simGetObjectHandle('UR5_ikTarget')
	---gripper
	Grasp_flag = false
	
end

if (sim_call_type==sim_childscriptcall_actuation) then
	curtime=simGetSimulationTime()-startTime
	
	
	-------control the car
	Dpos = simGetObjectPosition(tar_handle,tip_handle)
	d= math.sqrt( Dpos[1]^2 + Dpos[2]^2 )
	theta = math.atan(Dpos[2],Dpos[1])
	Dangle = simGetObjectOrientation(tar_handle,tip_handle)
    
	Vx = Dpos[1]*2
	Vy = Dpos[2]*10
	Vtheta = Dangle[3]*5
    --Vtheta = 0
	if state ==0 then
		Left_vel=0
		Right_vel = 0
	elseif state ==1 then
		Left_vel =- theta * 5
		Right_vel = theta * 5
		if math.abs(theta)<2*math.pi/180 then
			state = 2 --通过合适的转换条件进行状态切换，这个是状态机的标准写法
		end
	elseif state ==2 then
		Left_vel = d*3 - theta * 5
		Right_vel = d*3 + theta * 5
		if math.abs(Dpos[1])<0.03 and math.abs(Dpos[2])<0.03 then
			state = 3
		end
	elseif state ==3 then
		Left_vel = -Vtheta * 0.25
		Right_vel =Vtheta * 0.25
		if math.abs(Dangle[3])<1*math.pi/180 then
			state = 4
		end
	elseif state ==4 then
		Left_vel=0
		Right_vel = 0
		if math.abs(Dpos[1])>0.1 or math.abs(Dpos[2])>0.1 or math.abs(Dangle[3])>5*math.pi/180 then
			state = 1
		end
	end
    simSetJointTargetVelocity(MotorHandle_Left,Left_vel)
    simSetJointTargetVelocity(MotorHandle_Right,Right_vel)
    --simExtCustomUI_setLabelText(ui,1000,'Right wheel speed:'..Dpos_sum[1])
    --simExtCustomUI_setLabelText(ui,1001,'Left wheel speed:'..Dpos_sum[2])
	--------control the car  END!!
	
	----enable UR5 IK
	
	simSetIntegerSignal('Action_flag',IK_flag) --呼唤UR5的脚本，让UR5自身的脚本完成它自己的功能
	IK_res = simGetIntegerSignal('IK_res')
	simClearIntegerSignal('IK_res')
	if IK_res==1 then
		IK_flag = 0
		simExtCustomUI_setLabelText(ui,1010,'IK_flag:off')
	end
	if IK_flag ==1 then 
		simExtCustomUI_setLabelText(ui,1010,'IK_flag:on')
	end
	
end

if (sim_call_type==sim_childscriptcall_sensing) then
	simExtCustomUI_setLabelText(ui,999,'Simulation time:'..curtime)
	simExtCustomUI_setLabelText(ui,1000,'State:'..state)
end

if (sim_call_type==sim_childscriptcall_cleanup) then
	simExtCustomUI_destroy(ui)
end