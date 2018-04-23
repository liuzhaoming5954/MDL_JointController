if (sim_call_type==sim_childscriptcall_initialization) then
    motorHandle=simGetObjectHandle('RG2_openCloseJoint')
end


if (sim_call_type==sim_childscriptcall_actuation) then
    local data=simGetStringSignal('UR5_rg2GripperData')
    if data then
        velocityAndForce=simUnpackFloats(data)
        simSetJointTargetVelocity(motorHandle,velocityAndForce[1])
        simSetJointForce(motorHandle,velocityAndForce[2])
        
    end
end

