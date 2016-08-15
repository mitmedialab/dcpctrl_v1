function [ Q_raw ] = joint2raw_at40gw( robot, Q_joint )
%JOINT2RAW_AT40GW Convert from joint-space joint positions to raw
%encoder/voltage values.

% [ Q_raw ] = joint2raw_at40gw( robot, Q_joint )
% Julian Leland, MIT Media Lab, 2016-06-07
% INPUT:
%   robot - robot definition
%   Q_joint - Nx4 matrix of joint-space values in degrees and mm
% OUTPUT:
%   Q_raw - Nx4 matrix of raw joint values. J1 is in encoder counts
%   (737,280 counts/rev), J2-J4 are in volts.

Q_raw = zeros(size(Q_joint));

%% Joint 1: Degrees to encoder counts (measured relative to whatever start position was - we're still using an incremental encoder)

Q_raw(:,1) = round(Q_joint(:,1) ./ 360 .* robot.Joint(1).PosSensParams.CountsRev);


%% Joint 2: Degrees to volts

a = robot.Joint(2).PosSensParams.a_len; % Distance between joint axis and cylinder base axis
b = robot.Joint(2).PosSensParams.b_len; % Distance between joint axis and cylinder attachment point axis
c = robot.Joint(2).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(2).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(2).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(2).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(2).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);

Zero_Angle = acosd(((c^2)-(a^2)-(b^2))/(-2*a*b));
J_Len = sqrt(a^2 + b^2 - 2*a*b*cosd(Q_joint(:,2)+Zero_Angle));
J_dLen = J_Len-c; % Calculate change in distance from zero length of joint
J_dVolts = mapRange(J_dLen,(-sensLen/2),(sensLen/2),sensVMin,sensVMax);
%J_dVolts =
%((J_dLen-(-sensLen/2))*(sensVMax-sensVMin))/((sensLen/2)-(-sensLen/2))+
%sensVMin; % Use this line instead if we don't want to use mapRange.

Q_raw(:,2) = jointV_Zero + J_dVolts;

%% Joint 3: Degrees to volts

a = robot.Joint(3).PosSensParams.a_len; % Distance between joint axis and cylinder base axis
b = robot.Joint(3).PosSensParams.b_len; % Distance between joint axis and cylinder attachment point axis
c = robot.Joint(3).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(3).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(3).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(3).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(3).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);

Zero_Angle = acosd(((c^2)-(a^2)-(b^2))/(-2*a*b));
J_Len = sqrt(a^2 + b^2 - 2*a*b*cosd(Q_joint(:,3)-Zero_Angle)); % Joint 3 is treated differently than Joint 2 because of inverted relationship between triangle angle & "positive" angle as defined by DH parameters.
J_dLen = J_Len-c; % Calculate change in distance from zero length of joint
J_dVolts = mapRange(J_dLen,(-sensLen/2),(sensLen/2),sensVMin,sensVMax);
Q_raw(:,3) = jointV_Zero + J_dVolts;

%% Joint 4: Millimeters to volts

c = robot.Joint(4).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(4).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(4).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(4).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(4).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);

J_dLen = Q_joint(:,4)-c; % Calculate change in distance from zero length of joint
J_dVolts = mapRange(J_dLen,(-sensLen/2),(sensLen/2),sensVMin,sensVMax);
Q_raw(:,4) = jointV_Zero + J_dVolts;


end

