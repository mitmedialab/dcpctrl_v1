function [ errs ] = triggerio_kukakrl( handle, robot, ljout, ljin )
%TRIGGERIO_KUKAKRL Get the pin value of the labjack digital in channel corresponding to a
%KUKA KRL command
% OUTPUT:
%   errs - 0 means no errors, 1 means KUKA not idle

% Verify that KUKA is currently idle
errs = 0;
[~, state] = LabJack.LJM.eReadName(handle, robot.DigIn(1).RegNames.DIO_Name,0);
if state ~= 0
    errs = 1;
    return;
end

% If state is idle, set pin high, wait .1 sec, and then start watching for
% program end
LabJack.LJM.eWriteName(handle, robot.DigOut(ljout).RegNames.DIO_Name, 1);
pause(0.1);
LabJack.LJM.eWriteName(handle, robot.DigOut(ljout).RegNames.DIO_Name, 0); % Set to 0 so that we don't continue triggering programs
while true
    [~, state] = LabJack.LJM.eReadName(handle, robot.DigIn(ljin).RegNames.DIO_Name, 0);
    if state == 0
        return;
    end
end
end
