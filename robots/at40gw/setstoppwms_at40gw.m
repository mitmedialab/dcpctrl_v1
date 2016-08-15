function [ errors ] = setstoppwms_at40gw( handle, robot )
%SETSTOPPWMS_AT40GW Stops all joints by setting all PWM signsl to deadband
%regions

errors = zeros(1,4);

for i=1:4
    errors(i) = LabJack.LJM.eWriteName(handle,robot.Joint(1).RegNames.PWM_CfgA, robot.PWMZero);
end
end