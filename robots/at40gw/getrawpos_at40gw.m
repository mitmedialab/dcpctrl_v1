function [ jRawPos, errors ] = getrawpos_at40gw( ljh, robot )
%GETRAWPOS_AT40GW Returns the current raw sensor readings on the AT40GW via
%   LabJack, joint1 is in encoder counts, joints2 to 4 are analog voltages
%   corresponding to potentiometer readings, see machine definition for
%   details

errors = zeros(1,4);
jRawPos = zeros(1,4);

[errors(1), jRawPos(1)] = LabJack.LJM.eReadName(ljh, robot.Joint(1).RegNames.REnc_ReadAF, 0); % Read J1. Can't do in a FOR loop b/c J1 reads differently than other axes
[errors(2), jRawPos(2)] = LabJack.LJM.eReadName(ljh, robot.Joint(2).RegNames.AIN_Read, 0); % Read J2
[errors(3), jRawPos(3)] = LabJack.LJM.eReadName(ljh, robot.Joint(3).RegNames.AIN_Read, 0); % Read J3
[errors(4), jRawPos(4)] = LabJack.LJM.eReadName(ljh, robot.Joint(4).RegNames.AIN_Read, 0); % Read J4

end