function [ errors ] = setpwms_at40gw( ljh, robot, pwms )
%SETPWM_AT40GW Set all 4 AT40GW PWM signals for joints
%   WARNING: Must set all 4 joint PWMs at once with this function, this
%   function does not do anything "clever", immediately sets all raw values
%   on the LabJack. Can use values < 0 to ignore setting a joint value
%   (simply maintains current value).
%   TODO: Currently no error checking, but there should be! (Specifically
%   check if PWMs exceed limits)
% INPUTS:
%   ljh - LabJack handle
%   robot - robot definition/config
%   pwms - 1x4 vector containing joint1...joint4 PWM commands

% PWM values must be integers
pwms = round(pwms);

errors = zeros(1,4);

for i=1:4
    if pwms(i) >= 0
        errors(i) = LabJack.LJM.eWriteName(ljh,robot.Joint(i).RegNames.PWM_CfgA, pwms(i));
    end
end

end