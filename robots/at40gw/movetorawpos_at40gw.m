function errors = moveToRawPos_at40gw( ljh, robot, jRawPos_des )
% MOVETORAWPOS_AT40GW Moves the boom arm to the given raw joint positions
% (these must be in encoder counts/voltage units)
% INPUTS:
%   ljh - LabJack Handle
%   robot - robot definition
%   jRawPos_des - 1x4 vector of desired raw joint positions, must be the
%       same units as those returned by getRawPos_at40gw. jRawPos(1) is
%       joint 1 and must be encoder counts, jRawPos(2:4) must be voltage
%       values

% Must convert to radians or degrees and then back to encoder/potentiometer
% readings so that we can figure out necessary relative speeds of all the
% joints, otherwise cannot plan smooth trajectory in joint space. Must be
% able to estimate time for the slowest joint to reach its destination.