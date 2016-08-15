function [ pwms, errors ] = getpwms_at40gw(ljh, robot)
% Reads PWMS for all 4 joints
% INPUTS:
%   ljh - LabJack handle
%   robot - Robot definition (currently retrieved from machine definition
% file
% OUTPUTS:
%   pwms - from joint1...joint4 in 1x4 vector
%   errors - labjack error codes

pwms = zeros(1,4);
errors = zeros(1,4);
[errors(1), pwms(1)] = LabJack.LJM.eReadName(ljh,robot.Joint(1).RegNames.PWM_CfgA,0); % Read J1 PWM
[errors(2), pwms(2)] = LabJack.LJM.eReadName(ljh,robot.Joint(2).RegNames.PWM_CfgA,0); % Read J2 PWM
[errors(3), pwms(3)] = LabJack.LJM.eReadName(ljh,robot.Joint(3).RegNames.PWM_CfgA,0); % Read J3 PWM
[errors(4), pwms(4)] = LabJack.LJM.eReadName(ljh,robot.Joint(4).RegNames.PWM_CfgA,0); % Read J4 PWM

end

