function closeGripper(id,vrep,handles,Xspeed)
%% Author: Akshitha Pothamshetty
 res=vrep.simxSetJointForce(id,handles.EndGripper,150,vrep.simx_opmode_oneshot);
 vrchk(vrep, res, true);
 res=vrep.simxSetJointTargetVelocity(id,handles.EndGripper,-Xspeed,vrep.simx_opmode_oneshot);
 vrchk(vrep, res, true);
 pause(2);
end