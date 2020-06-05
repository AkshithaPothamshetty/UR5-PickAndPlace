function [ rot_mat ] = EulerZYX( theta )
%% Author: Akshitha Pothamshetty
    rot_mat = ROT('X',theta(1)) * ROT('Y',theta(2)) * ROT('Z',theta(3));
end