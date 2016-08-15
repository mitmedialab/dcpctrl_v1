function Q_joint = raw2joint_at40gw(robot, Q_raw)
% RAW2JOINT_AT40GW converts raw sensor readings to joint-space positions of
% the AT40GW, where revolute joints 1...3 are in degrees and prismatic
% joint 4 is in mm
% INPUT:
%   robot - robot definition
%   Q_raw - Nx4 matrix of raw joint values (joint 1 is encoder counts, 2..4
%       are in voltages
% OUTPUT:
%   Q_joint - Nx4 matrix of joint-space values in degrees and mm (use
%   deg2rad to convert to radians)

Q_joint = zeros(size(Q_raw));

% Joint 1: Encoder counts to degrees
counts2rev = 1/robot.Joint(1).PosSensParams.CountsRev;

Q_joint(:,1) = Q_raw(:,1) .* counts2rev .* 360;

% Joint 2: Voltages to degrees
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
% Find position along sensor of joint zero, relative to center
sensP_Zero = ((jointV_Zero-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);

% At current voltage, how far away from jointP_Zero are we?
sensP_Cur = ((Q_raw(:,2)-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;

Q_joint(:,2) = (acosd(((J_Len.^2)-(a^2)-(b^2))/(-2*a*b))-acosd(((c^2)-(a^2)-(b^2))/(-2*a*b)));

% Joint 3: Voltages to degrees
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

% Find position along sensor of joint zero, relative to center
sensP_Zero = ((jointV_Zero-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);

% At current voltage, how far away from jointP_Zero are we?
sensP_Cur = ((Q_raw(:,3)-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;
Q_joint(:,3) = (acosd(((c^2)-(a^2)-(b^2))/(-2*a*b))-acosd(((J_Len.^2)-(a^2)-(b^2))/(-2*a*b)));

% Joint 4: Voltage to mm
a = robot.Joint(4).PosSensParams.a_len; % Distance between joint axis and cylinder base axis
b = robot.Joint(4).PosSensParams.b_len; % Distance between joint axis and cylinder attachment point axis
c = robot.Joint(4).PosSensParams.c_len; % Length of cylinder at J_Angle = 0
jointV_Zero = robot.Joint(4).PosSensParams.ZeroVoltage; % Sensor voltage at zero position
jointV_Max = robot.Joint(4).PosLim.Max; % Maximum sensor voltage
jointV_Min = robot.Joint(4).PosLim.Min; % Minimum sensor voltage
sensLen = robot.Joint(4).PosSensParams.SensLen; % Length of joint sensor (allows us to figure out where we are along joint)

% We need to make the orientation of our max/min sensor values match the
% orientation of the joint we're working on.
sensVMax = 10*sign(jointV_Max); 
sensVMin = 10*sign(jointV_Min);

% Find position along sensor of joint zero, relative to center
sensP_Zero = ((jointV_Zero-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);

% At current voltage, how far away from jointP_Zero are we?
sensP_Cur = ((Q_raw(:,4)-sensVMin)*((sensLen/2)-(-sensLen/2)))/(sensVMax-sensVMin)+ (-sensLen/2);
dSensP = sensP_Zero - sensP_Cur;
J_Len = c - dSensP;
Q_joint(:,4) = J_Len;

end