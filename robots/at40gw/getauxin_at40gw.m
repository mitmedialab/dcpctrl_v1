function [Ai,Di] = getauxin_at40gw(handle, robot)
%GETAUXIN_AT40GW Read auxiliary inputs from AT40GW

%{
    [Ai,Di] = getauxin_at40gw(handle, robot)
    Julian Leland, MIT Media Lab, 2016-07-11
    Not sure who originally wrote this. Most recently modified by JL on
    date above.
%}
Ai = ones(1,length(robot.AnIn))*-1;
for n = 1:length(robot.AnIn)
    [~, Ai(n)] = LabJack.LJM.eReadName(handle,robot.AnIn(n).RegNames.AIN_Read,0); % Read AIN 1
end

Di = ones(1,length(robot.DigIn))*-1;
for n = 1:length(robot.DigIn)
    [~, Di(n)] = LabJack.LJM.eReadName(handle,robot.DigIn(n).RegNames.DIO_Name,0); % Read AIN 1
end

end

