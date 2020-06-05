
%% Information Section
% ENPM662 Modeling Final Project
% Author: Akshitha Pothamshetty
% Email-ID: apothams@umd.edu
% Section: 0101

%% About the Project
% Goal: In this project, I utilized a 6 DOF industrial robotic arm, UR5 to
% pick and place small known objects, similar to bin picking process. The
% project starts with the theoretical calculation of Forward Kinematics and
% Inverse Kinematics of UR5. With the assumption of slow movement of the
% joints, Dynamics of the arm were neglected. I used MATLAB and Vrep to
% demonstrate the simulation of my project. Practical implementation begins
% with making Vrep and MATLAB communicate to each other. I used MATLAB as
% the scripting language to implement FK and IK. Once, Vrep and MATLAB were
% setup, I validated my implementation using theoretical FK and IK events
% and matching them with Vrep Output. In the end, I have demonstrated the
% pick & place operation with the robot tasked to segregate between
% different blocks and placing them on different tables.


%% Step 1: Including Paths and .m Files 

addpath('..'); % Go to Subdirectory, containing Main Folder, VrepConnection and Mfunctions folder.

addpath('../VrepConnection','../Mfunctions/Robotics_Functions',...
    '../Mfunctions/URn_Functions','../Mfunctions/Vrep_Functions'); % Add to Path all the folders.
  
vrep=remApi('remoteApi'); % using the prototype file
vrep.simxFinish(-1); % Close Open Connections
id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (id < 0)
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end

fprintf('Connection %d to remote API server open.\n', id);


%% Step 2: GetObjectHandle for UR5 Joints, Blocks and Frames.

%%% For the UR5 Joints 
handles = struct('id',id);
jointNames={'UR5_joint1','UR5_joint2','UR5_joint3','UR5_joint4',...
    'UR5_joint5','UR5_joint6'};

handles.ur5Joints = -ones(1,6); 
for i = 1:6
    [res, handles.ur5Joints(i)] = vrep.simxGetObjectHandle(id, ...
                     jointNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
end


%%% For the Cuboids & Tables
CuboidNames={'Left_Cuboid','Front_Cuboid','Right_Cuboid'};

TableNames={'Left_Table','Front_Table','Right_Table'};

handles.ur5Cuboids = -ones(1,3); 
handles.ur5Tables = -ones(1,3); 

 for i = 1:3
    [res, handles.ur5Cuboids(i)] = vrep.simxGetObjectHandle(id,...
                    CuboidNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
    
    [res, handles.ur5Tables(i)] = vrep.simxGetObjectHandle(id,...
                    TableNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);       
end

%%% For the Ref & Gripper
  % Those Handle Not Used in the Simulation 
handles.ur5Ref= -1;
handles.ur5Gripper=-1;
[res, handles.ur5Ref] = vrep.simxGetObjectHandle(id, 'UR5', ...
    vrep.simx_opmode_oneshot_wait); 
vrchk(vrep, res);

[res, handles.ur5Gripper] = vrep.simxGetObjectHandle(id, 'UR5_connection', ...
    vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%%% For the Base frame [Frame0]
handles.base=-1;
[res, handles.base] = vrep.simxGetObjectHandle(id, ...
    'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%%% For the Target frame [Frame1]
handles.Target=-1;
[res, handles.Target] = vrep.simxGetObjectHandle(id, ...
    'Frame1', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%%% For the EndGripper And EndGripperJoints
handles.EndGripper= -1 ;
    [res, handles.EndGripper] = vrep.simxGetObjectHandle(id, ...
                  'RG2_openCloseJoint', vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);

%% Stream ...

%%% Cuboids [position/orientation & Tables [position/orientation]
relativToRef = handles.base;
for i = 1:3
    
    %Cuboids
    res = vrep.simxGetObjectPosition(id,handles.ur5Cuboids(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id,handles.ur5Cuboids(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);  
    
    % Tables
    res = vrep.simxGetObjectPosition(id,handles.ur5Tables(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id,handles.ur5Tables(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);  
end


%%% EndGripper & EndGripperJoints    
res = vrep.simxGetJointPosition(id,handles.EndGripper,...
         vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);

%%% The UR5 Joints
for i = 1:6
    res = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
               vrep.simx_opmode_streaming); 
    vrchk(vrep, res, true);
end


%% Start
vrep.simxGetPingTime(id);
vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

%% Simulation

% Set the threshold to check if the end effector has reached its destination
handles.threshold = 0.01;

% Set The Arm Parameters Using Peter Corke robotics toolbox
handles.ur5Robot= URnSerial_fwdtrans('UR5');
pause(0.5);
%Rest Joint for 1st time
handles.startingJoints = [0, 0, 0, 0, 0, 0];
        res = vrep.simxPauseCommunication(id, true);
        vrchk(vrep, res);
    for j = 1:6
            vrep.simxSetJointTargetPosition(id, handles.ur5Joints(j),...
                  handles.startingJoints(j),vrep.simx_opmode_oneshot);
            vrchk(vrep, res);
    end
        res = vrep.simxPauseCommunication(id, false);
        vrchk(vrep, res);
pause(1);
 
%% Simulating Pick and Place Operation. 

ConstValue=0.25;

iTable=1;  % 1 for 'Left_Table', 2 for'Front_Table', 3 for'Right_Table'
iCuboid=1; % 1 for 'Left_Cuboid', 2 for'Front_Cuboid', 3 for'Right_Cuboid'
XYZoffset= [0 0 0]; % [ X Y Z]

%%% 1st Sequence

iCuboid=1;  XYZoffset=[0 0 ConstValue];
GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
iTable=1;   XYZoffset=[0 ConstValue 0];
GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);

iCuboid=2;  XYZoffset=[0 0 ConstValue];
GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
iTable=2;   XYZoffset=[0 ConstValue 0];
GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);

iCuboid=3;  XYZoffset=[0 0 ConstValue];
GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
iTable=3;   XYZoffset=[0 -ConstValue 0];
GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);
  
% Return to Normal Orientation.
g= eye(4,4);
g(1:3,4)=[-0.25 1 0.7301];
moveFrame( id, vrep, g, handles.ur5Cuboids(1), handles.base );
g(1:3,4)=[0 1 0.7301];
moveFrame( id, vrep, g, handles.ur5Cuboids(2), handles.base );
g(1:3,4)=[0.25 1 0.7301];
moveFrame( id, vrep, g, handles.ur5Cuboids(3), handles.base ); 
pause(2);
   
%%% 2nd Sequence
% for i=1:3
%     
%     iCuboid=i;  XYZoffset=[0 0 ConstValue];
%     GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
%     iTable=1;   XYZoffset=[0 0 0.15*i];
%     GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);
% 
% end


%% END ..
vrep.delete;
clear; clc;

