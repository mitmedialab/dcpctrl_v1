function [] = resetsystem_at40gw( ljh, robot )
%RESETSYSTEM_AT40GW Resets LabJack to zero state (PWM at 50%, analog & digital
%outputs as defined in configuration file.

%   Julian Leland, MIT Media Lab, 2016-06-06
%   Resets PWM and digital/analog outputs for AT40GW. Same as
%   setstoppwms_at40gw, except for additional configuration of analog and
%   digital outputs.

%% Set up LabJack
temp_MD = robot;

ljasm = NET.addAssembly('LabJack.LJM'); % Make UD .NET assembly visible in MATLAB
[ljmerror, handle] = LabJack.LJM.OpenS('ANY','ANY','ANY',ljh);

for i = 1:4
    LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.PWM_CfgA, temp_MD.PWMZero); % Write PWM zero to all channels 
end
for i = 1:10
    LabJack.LJM.eWriteName(handle,temp_MD.DigOut(i).RegNames.DIO_Name,temp_MD.DigOut(i).DefaultVal); % Set all digital outputs to value defined in config file
end
for i = 1:2
    LabJack.LJM.eWriteName(handle,temp_MD.AnOut(i).RegNames.DAC_Name,temp_MD.AnOut(i).DefaultVal); % Set all analog outputs to value defined in config file
end
 

end

